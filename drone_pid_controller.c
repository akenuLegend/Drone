/*
 * =============================================================================
 * HỆ THỐNG ĐIỀU KHIỂN CÂN BẰNG DRONE — CASCADED PID CONTROLLER
 * =============================================================================
 * Tác giả   : Được phát triển dựa trên tài liệu "Thuật toán PID Drone"
 * Kiến trúc : Cascaded PID (Vòng lặp kép) + Complementary Filter + Motor Mixing
 * Phần cứng : Quadcopter X-frame, IMU MPU-6050 (Gyro + Accel), 4x ESC/Motor
 * Vi điều khiển: ESP32 (ESP-IDF) — không dùng FreeRTOS
 *
 * Luồng xử lý tổng thể:
 *     RC Input (góc mong muốn)
 *         → [Angle Loop - 1~2kHz] tính Desired Rate
 *             → [Rate Loop - 4~8kHz] tính lệnh điều khiển
 *                 → [Motor Mixing] phân phối PWM cho 4 động cơ
 *                     → Drone bay → IMU đọc thực tế → hồi tiếp
 *
 * Cách dùng file này:
 *     1. Copy vào project ESP-IDF, đổi tên thành main.c
 *     2. Kết nối MPU-6050 vào GPIO 21 (SDA) và GPIO 22 (SCL)
 *     3. Kết nối 4 ESC vào GPIO 13, 12, 14, 27
 *     4. Build: idf.py build && idf.py flash
 * =============================================================================
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"      /* Chỉ dùng để delay, không dùng task/mutex */
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "DroneFC";

/* ============================================================================
 * CẤU HÌNH PHẦN CỨNG — Thay đổi theo sơ đồ mạch của bạn
 * ============================================================================ */
#define PIN_SDA             21
#define PIN_SCL             22
#define PIN_MOTOR_M1        13      /* Trước-Trái  (CW)  */
#define PIN_MOTOR_M2        12      /* Trước-Phải  (CCW) */
#define PIN_MOTOR_M3        14      /* Sau-Phải    (CW)  */
#define PIN_MOTOR_M4        27      /* Sau-Trái    (CCW) */

/* Giới hạn PWM cho ESC (microseconds) */
#define PWM_MIN_US          1000    /* Motor dừng hoàn toàn (đã arm) */
#define PWM_MAX_US          2000    /* Full throttle (100%)           */
#define PWM_IDLE_US         1100    /* Tốc độ tối thiểu khi đang bay  */

/* Chuyển microsecond → duty 16-bit (period = 20000µs tại 50Hz) */
#define US_TO_DUTY(us)      ((uint32_t)((us) * 65535UL / 20000UL))

/* Góc và tốc độ tối đa */
#define MAX_ANGLE_DEG       45.0f
#define MAX_RATE_DEG_S      500.0f
#define MAX_YAW_RATE_DEG_S  200.0f

/* Chu kỳ vòng lặp chính (giây) */
#define DT_ANGLE            0.001f      /* 1ms  — Angle Loop 1 kHz  */
#define DT_RATE             0.00025f    /* 250µs — Rate Loop 4 kHz  */

#ifndef M_PI
#define M_PI 3.14159265358979f
#endif


/* ============================================================================
 * PHẦN 1: BỘ LỌC TÍN HIỆU (SIGNAL FILTERS)
 * ============================================================================ */

/*
 * LOW-PASS FILTER (Bộ lọc thông thấp)
 *
 * MỤC ĐÍCH:
 *     Động cơ quay nhanh tạo ra rung cơ học tần số cao (~200-400Hz).
 *     Rung này truyền qua khung carbon vào chip IMU, làm sai lệch dữ liệu.
 *     Nếu đưa thẳng vào khâu D của PID: D = (error - prev_error) / dt
 *     → Khâu D khuếch đại nhiễu lên hàng trăm lần → nóng/giật/cháy động cơ.
 *
 * NGUYÊN LÝ:
 *     output = α × input + (1 - α) × output_trước
 *
 *     Trong đó α = dt / (dt + τ),  τ = 1 / (2π × cutoff_freq)
 *
 *     α nhỏ → lọc mạnh hơn (mượt hơn nhưng chậm hơn)
 *     α lớn → lọc yếu hơn (phản ứng nhanh hơn nhưng còn nhiễu)
 *
 * GIÁ TRỊ THỰC TẾ:
 *     cutoff_freq ≈ 50-100 Hz cho hệ thống drone thông thường.
 *     Tần số rung động cơ ~200-400Hz sẽ bị triệt tiêu gần hoàn toàn.
 */
typedef struct {
    float cutoff_hz;        /* Tần số cắt (Hz). Tần số cao hơn giá trị này sẽ bị lọc bỏ. */
    float prev_output;      /* Giá trị đầu ra chu kỳ trước (trạng thái bộ nhớ) */
} LowPassFilter;

/* Khởi tạo bộ lọc */
void lpf_init(LowPassFilter *f, float cutoff_hz)
{
    f->cutoff_hz   = cutoff_hz;
    f->prev_output = 0.0f;
}

/*
 * Cập nhật bộ lọc với giá trị thô mới.
 *
 * Args:
 *     f         : Con trỏ tới LowPassFilter
 *     raw_input : Giá trị thô từ cảm biến (đã bị nhiễu)
 *     dt        : Chu kỳ lấy mẫu (giây). Phải > 0.
 *
 * Returns:
 *     Giá trị đã được lọc nhiễu.
 */
float lpf_update(LowPassFilter *f, float raw_input, float dt)
{
    /* τ = hằng số thời gian = 1 / (2π × f_cutoff) */
    float tau   = 1.0f / (2.0f * (float)M_PI * f->cutoff_hz);

    /* Hệ số pha trộn — phụ thuộc vào dt và τ
     * dt nhỏ → α nhỏ → tín hiệu "cũ" chiếm ưu thế → lọc tốt hơn */
    float alpha = dt / (dt + tau);

    /* Phương trình lọc: pha trộn giữa tín hiệu mới và tín hiệu cũ */
    float output = alpha * raw_input + (1.0f - alpha) * f->prev_output;
    f->prev_output = output;
    return output;
}


/*
 * COMPLEMENTARY FILTER (Bộ lọc bù) — tính góc tuyệt đối tối ưu.
 *
 * VẤN ĐỀ:
 *     • Gyroscope: Nhanh, nhạy, nhưng bị lỗi trôi góc (drift) theo thời gian.
 *       Tích phân nhỏ sai sẽ tích lũy thành sai lớn sau vài phút.
 *     • Accelerometer: Không bị drift, cho mốc góc tuyệt đối (dựa vào trọng lực).
 *       Nhưng rất nhạy với gia tốc tuyến tính → nhiễu khi drone đang bay cơ động.
 *
 * GIẢI PHÁP:
 *     Pha trộn hai nguồn với trọng số khác nhau:
 *
 *         Góc_mới = 0.98 × (Góc_cũ + Gyro_đã_lọc × dt) + 0.02 × Góc_Accel
 *
 *     → 98% tin vào Gyro (nhanh, ít nhiễu sau LPF)
 *     → 2%  tin vào Accel (bù trừ drift dài hạn)
 *
 * KẾT QUẢ:
 *     Góc ổn định, không drift, không giật, phản ứng nhanh.
 *     Đây là chuẩn công nghiệp cho hệ thống bay nhỏ (UAV, drone FPV).
 */
typedef struct {
    float alpha;    /* Trọng số cho Gyro (0.95–0.99 là dải thực tế) */
    float angle;    /* Góc hiện tại được ước lượng (độ)             */
} ComplementaryFilter;

/* Khởi tạo */
void comp_filter_init(ComplementaryFilter *cf, float alpha)
{
    cf->alpha = alpha;
    cf->angle = 0.0f;
}

/*
 * Cập nhật ước lượng góc.
 *
 * Args:
 *     cf                 : Con trỏ tới ComplementaryFilter
 *     gyro_rate_filtered : Tốc độ góc từ Gyro (đã qua LPF), đơn vị: °/s
 *     accel_angle        : Góc tính từ Accelerometer (°), dùng làm mốc tuyệt đối
 *     dt                 : Chu kỳ lấy mẫu (giây)
 *
 * Returns:
 *     Góc ước lượng tối ưu (độ).
 */
float comp_filter_update(ComplementaryFilter *cf,
                          float gyro_rate_filtered,
                          float accel_angle,
                          float dt)
{
    /* Thành phần Gyro: tích phân tốc độ góc để ước lượng thay đổi góc
     * Đây là phần nhanh, phản ứng ngay lập tức với chuyển động */
    float gyro_angle = cf->angle + gyro_rate_filtered * dt;

    /* Pha trộn: 98% Gyro (nhanh) + 2% Accel (mốc dài hạn)
     * Phần Accel nhỏ nhưng quan trọng: nó "kéo" góc về giá trị đúng
     * và ngăn Gyro bị trôi ra xa */
    cf->angle = cf->alpha * gyro_angle + (1.0f - cf->alpha) * accel_angle;
    return cf->angle;
}

/* Tính góc Pitch từ Accelerometer: pitch = atan2(ax, az) (độ) */
float accel_to_angle_pitch(float ax, float az)
{
    return atan2f(ax, az) * (180.0f / (float)M_PI);
}

/* Tính góc Roll từ Accelerometer: roll = atan2(ay, az) (độ) */
float accel_to_angle_roll(float ay, float az)
{
    return atan2f(ay, az) * (180.0f / (float)M_PI);
}


/* ============================================================================
 * PHẦN 2: BỘ ĐIỀU KHIỂN PID CƠ BẢN
 * ============================================================================
 *
 * BA KHÂU VÀ Ý NGHĨA VẬT LÝ:
 *
 * ① KHÂU P (Proportional — Tỷ lệ):
 *     Output_P = Kp × e(t)
 *
 *     Tạo ra lực sửa chữa tỷ lệ thuận với sai số hiện tại.
 *     → Drone đang nghiêng 10°? P tạo lực đẩy ngược lại để dựng lại.
 *     → Kp cao: phản ứng nhanh nhưng dễ dao động (oscillation).
 *     → Kp thấp: phản ứng chậm, khó đạt điểm đặt.
 *     ⚠ Vấn đề: chỉ dùng P → luôn tồn tại sai số xác lập (steady-state error).
 *
 * ② KHÂU I (Integral — Tích phân):
 *     Output_I = Ki × Σ[e(t) × dt]
 *
 *     Tích lũy sai số theo thời gian → triệt tiêu sai số xác lập.
 *     → Nếu drone bị lệch nhẹ do trọng tâm không cân, I sẽ dần dần tích lũy
 *       và cấp thêm lực để bù đắp.
 *     → Ki cao: khắc phục sai số nhanh nhưng dễ gây vượt ngưỡng (overshoot).
 *     ⚠ Vấn đề: "Integral windup" — khi bão hòa, I tích lũy quá lớn → phản ứng kỳ quặc.
 *
 * ③ KHÂU D (Derivative — Vi phân):
 *     Output_D = Kd × [e(t) - e(t-1)] / dt
 *
 *     Đo tốc độ thay đổi của sai số → dự đoán xu hướng → phanh chủ động.
 *     → Nếu drone đang dựng lại nhanh (sai số giảm gấp), D sẽ phanh bớt
 *       để không lao quá đà (overshoot).
 *     → Kd cao: phanh mạnh, ổn định nhưng nhạy với nhiễu.
 *     ⚠ Vấn đề: D khuếch đại nhiễu → bắt buộc phải dùng LPF trước khi vào D.
 *
 * TỔNG HỢP:
 *     Output = Kp×e + Ki×∫e·dt + Kd×(de/dt)
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float output_limit;     /* Giới hạn đầu ra tổng (anti-windup + bảo vệ động cơ) */
    float integral_limit;   /* Giới hạn riêng cho khâu I (chống integral windup)    */

    /* Trạng thái nội bộ — phải reset khi drone arm/disarm */
    float integral;         /* Tích lũy sai số (khâu I)       */
    float prev_error;       /* Sai số chu kỳ trước (khâu D)   */
} PIDController;

/* Khởi tạo bộ điều khiển PID */
void pid_init(PIDController *p,
              float kp, float ki, float kd,
              float output_limit, float integral_limit)
{
    p->kp             = kp;
    p->ki             = ki;
    p->kd             = kd;
    p->output_limit   = output_limit;
    p->integral_limit = integral_limit;
    p->integral       = 0.0f;
    p->prev_error     = 0.0f;
}

/*
 * Tính đầu ra PID cho một chu kỳ.
 *
 * Args:
 *     p           : Con trỏ tới PIDController
 *     setpoint    : Giá trị mong muốn (Set Point — SP)
 *     measurement : Giá trị thực đo được (Process Variable — PV)
 *     dt          : Thời gian từ lần tính trước (giây). Phải > 0.
 *
 * Returns:
 *     Giá trị điều khiển đầu ra (đã được giới hạn).
 */
float pid_compute(PIDController *p,
                   float setpoint,
                   float measurement,
                   float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* ── Tính sai số ─────────────────────────────────────────────────────
     * e(t) = SP - PV: dấu dương nghĩa là cần tăng output */
    float error = setpoint - measurement;

    /* ── KHÂU P ──────────────────────────────────────────────────────────*/
    float p_term = p->kp * error;

    /* ── KHÂU I ──────────────────────────────────────────────────────────
     * Tích phân số trị: diện tích hình chữ nhật dưới đường cong sai số
     * integral += e(t) × dt  →  xấp xỉ ∫e·dt */
    p->integral += error * dt;

    /* Chống Integral Windup: giới hạn giá trị tích lũy
     * Nếu không có anti-windup: khi drone bị khóa (throttle=0), I tích lũy
     * vô hạn → khi drone được thả ra, I xuất ra lực cực lớn → mất kiểm soát */
    if (p->integral >  p->integral_limit) p->integral =  p->integral_limit;
    if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;
    float i_term = p->ki * p->integral;

    /* ── KHÂU D ──────────────────────────────────────────────────────────
     * Đạo hàm số trị: tốc độ thay đổi của sai số
     * d_term = Kd × [e(t) - e(t-1)] / dt
     *
     * Lưu ý: ta dùng "Derivative on Error" — nâng cao có thể đổi sang
     * "Derivative on Measurement" để tránh "Derivative kick" khi Set Point
     * thay đổi đột ngột: d_term = -Kd × (measurement - prev_measurement) / dt */
    float derivative = (error - p->prev_error) / dt;
    float d_term = p->kd * derivative;

    /* Cập nhật sai số cho chu kỳ tiếp theo */
    p->prev_error = error;

    /* ── Tổng hợp đầu ra ─────────────────────────────────────────────────*/
    float output = p_term + i_term + d_term;

    /* Giới hạn đầu ra tổng (bảo vệ ESC và động cơ) */
    if (output >  p->output_limit) output =  p->output_limit;
    if (output < -p->output_limit) output = -p->output_limit;

    return output;
}

/* Reset trạng thái PID — gọi khi drone disarm hoặc khởi động lại */
void pid_reset(PIDController *p)
{
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
}


/* ============================================================================
 * PHẦN 3: CASCADED PID — VÒNG LẶP KÉP
 * ============================================================================
 *
 * TẠI SAO CẦN VÒNG LẶP KÉP?
 *
 * Hệ thống đơn (Single Loop):
 *     Set Angle → [PID] → Motor → Angle
 *
 *     Vấn đề: Drone có quán tính cơ học. Khi PID ra lệnh "tăng tốc M1",
 *     drone không dừng tức thì ở góc mong muốn mà tiếp tục xoay do quán tính
 *     → dao động → mất kiểm soát.
 *
 * Hệ thống kép (Cascaded):
 *     Set Angle → [Angle PID] → Desired Rate → [Rate PID] → Motor → Rate → Angle
 *
 *     Giải pháp: Rate Loop chạy nhanh hơn (4-8kHz) và trực tiếp kiểm soát
 *     TỐC ĐỘ QUAY (°/s) thay vì góc. Nó hoạt động như chiếc phanh tốc độ:
 *     triệt tiêu quán tính ngay tại nguồn.
 *
 * VÒNG NGOÀI — Angle Loop (1-2kHz):
 *     Input  : Góc mong muốn (từ RC) vs Góc thực tế (từ Comp. Filter)
 *     Output : Tốc độ góc mong muốn (Desired Rate, °/s)
 *     Thuật toán: Chỉ dùng khâu P (thường đủ để tính desired rate)
 *
 * VÒNG TRONG — Rate Loop (4-8kHz):
 *     Input  : Desired Rate vs Tốc độ góc thực tế (từ Gyro)
 *     Output : Giá trị điều khiển động cơ
 *     Thuật toán: PID đầy đủ (P + I + D) để triệt tiêu quán tính
 */
typedef struct {
    PIDController angle_pid;    /* Vòng ngoài — chỉ P                  */
    PIDController rate_pid;     /* Vòng trong — PID đầy đủ             */
    float         max_rate;     /* Tốc độ góc tối đa (°/s) — an toàn   */
} CascadedPIDAxis;

/*
 * Khởi tạo Cascaded PID cho một trục (Pitch, Roll).
 *
 * Args:
 *     c              : Con trỏ tới CascadedPIDAxis
 *     angle_kp       : Hệ số P của vòng ngoài (Angle Loop)
 *     rate_kp/ki/kd  : Hệ số PID của vòng trong (Rate Loop)
 *     max_rate_deg_s : Tốc độ góc tối đa cho phép (°/s)
 */
void cascaded_pid_init(CascadedPIDAxis *c,
                        float angle_kp,
                        float rate_kp, float rate_ki, float rate_kd,
                        float max_rate_deg_s)
{
    /* Vòng ngoài: chỉ có khâu P
     * Ki=0, Kd=0 vì: steady-state error trong angle loop được xử lý bởi rate loop */
    pid_init(&c->angle_pid, angle_kp, 0.0f, 0.0f, max_rate_deg_s, 0.0f);

    /* Vòng trong: PID đầy đủ */
    pid_init(&c->rate_pid, rate_kp, rate_ki, rate_kd, 400.0f, 150.0f);

    c->max_rate = max_rate_deg_s;
}

/*
 * Tính đầu ra điều khiển cho một trục.
 *
 * Args:
 *     c               : Con trỏ tới CascadedPIDAxis
 *     angle_setpoint  : Góc mong muốn (từ stick RC), đơn vị: độ
 *     angle_measured  : Góc thực tế (từ Complementary Filter), đơn vị: độ
 *     rate_measured   : Tốc độ góc thực tế (từ Gyro, đã qua LPF), đơn vị: °/s
 *     dt_angle        : Delta time của vòng ngoài (giây)
 *     dt_rate         : Delta time của vòng trong (giây)
 *
 * Returns:
 *     Giá trị điều khiển (sẽ được đưa vào Motor Mixing)
 */
float cascaded_pid_compute(CascadedPIDAxis *c,
                            float angle_setpoint,
                            float angle_measured,
                            float rate_measured,
                            float dt_angle,
                            float dt_rate)
{
    /* VÒNG NGOÀI: Tính Desired Rate từ sai số góc
     * Kp × (angle_setpoint - angle_measured) = desired angular rate
     * Ví dụ: nghiêng 10°, Kp=15 → desired rate = 150 °/s */
    float desired_rate = pid_compute(&c->angle_pid,
                                      angle_setpoint,
                                      angle_measured,
                                      dt_angle);

    /* VÒNG TRONG: Điều khiển tốc độ góc thực tế theo desired_rate
     * Rate PID so sánh tốc độ thực với tốc độ mong muốn
     * và xuất lệnh điều chỉnh động cơ NGAY LẬP TỨC */
    float control_output = pid_compute(&c->rate_pid,
                                        desired_rate,
                                        rate_measured,
                                        dt_rate);

    return control_output;
}

/* Reset cả hai vòng PID */
void cascaded_pid_reset(CascadedPIDAxis *c)
{
    pid_reset(&c->angle_pid);
    pid_reset(&c->rate_pid);
}


/* ============================================================================
 * PHẦN 4: MOTOR MIXING — PHÂN PHỐI TÍN HIỆU CHO 4 ĐỘNG CƠ
 * ============================================================================
 *
 * SƠ ĐỒ DRONE X-FRAME (nhìn từ trên):
 *
 *                     PHÍA TRƯỚC (PITCH-)
 *                         M1 (CW)   M2 (CCW)
 *                        /               \
 *                       /                 \
 *                      /                   \
 *                    M4 (CCW)             M3 (CW)
 *                     PHÍA SAU (PITCH+)
 *
 * QUY LUẬT ĐIỀU KHIỂN:
 *
 * THROTTLE (Ga):  Tất cả 4 motor tăng/giảm đều nhau → lên/xuống
 *
 * PITCH (Ngả trước/sau):
 *     Muốn ngả về phía trước (pitch+):
 *     → Tăng M3, M4 (sau) + Giảm M1, M2 (trước)
 *     → Đuôi nâng lên, mũi hạ xuống → bay về phía trước
 *
 * ROLL (Nghiêng trái/phải):
 *     Muốn nghiêng phải (roll+):
 *     → Tăng M1, M4 (trái) + Giảm M2, M3 (phải)
 *
 * YAW (Xoay tại chỗ):
 *     Dựa vào phản lực của rotor:
 *     → Motor CW (M1, M3) tạo mô-men xoắn CCW lên thân drone
 *     → Motor CCW (M2, M4) tạo mô-men xoắn CW lên thân drone
 *     Muốn xoay CW: Tăng M1+M3 (CW) + Giảm M2+M4 (CCW)
 *
 * BẢNG TRỘN TÍN HIỆU:
 * ┌────────┬───────────┬──────────┬──────────┬──────────┐
 * │ Motor  │ Throttle  │  Pitch   │   Roll   │   Yaw    │
 * ├────────┼───────────┼──────────┼──────────┼──────────┤
 * │  M1    │    +1     │   -1     │   +1     │   -1     │
 * │  M2    │    +1     │   -1     │   -1     │   +1     │
 * │  M3    │    +1     │   +1     │   -1     │   -1     │
 * │  M4    │    +1     │   +1     │   +1     │   +1     │
 * └────────┴───────────┴──────────┴──────────┴──────────┘
 */

/*
 * Tính PWM cho từng motor dựa trên tín hiệu điều khiển.
 *
 * Args:
 *     throttle   : Tín hiệu ga (1000–2000), đơn vị µs
 *     pitch_ctrl : Đầu ra PID của trục Pitch (-400 đến +400)
 *     roll_ctrl  : Đầu ra PID của trục Roll  (-400 đến +400)
 *     yaw_ctrl   : Đầu ra PID của trục Yaw   (-200 đến +200)
 *     m1..m4     : Con trỏ đầu ra PWM cho từng motor (µs)
 */
void motor_mix(float throttle,
               float pitch_ctrl, float roll_ctrl, float yaw_ctrl,
               uint16_t *m1, uint16_t *m2, uint16_t *m3, uint16_t *m4)
{
    /* ── Bảng trộn tín hiệu ──────────────────────────────────────────────
     * Mỗi motor = Throttle ± Pitch ± Roll ± Yaw
     * Dấu ± xác định trục nào được phép điều khiển motor đó */
    float fm1 = throttle - pitch_ctrl + roll_ctrl - yaw_ctrl;  /* Trước-Trái  (CW)  */
    float fm2 = throttle - pitch_ctrl - roll_ctrl + yaw_ctrl;  /* Trước-Phải  (CCW) */
    float fm3 = throttle + pitch_ctrl - roll_ctrl - yaw_ctrl;  /* Sau-Phải    (CW)  */
    float fm4 = throttle + pitch_ctrl + roll_ctrl + yaw_ctrl;  /* Sau-Trái    (CCW) */

    /* ── Chuẩn hóa (Normalization) ───────────────────────────────────────
     * Vấn đề: Nếu tổng vượt quá 2000µs, ESC không thể thực hiện
     * Giải pháp: Tìm motor cao nhất, nếu vượt ngưỡng → kéo tất cả xuống đều nhau
     * Điều này giữ NGUYÊN SỰ CHÊNH LỆCH giữa các motor → không mất attitude */
    float max_pwm = fm1;
    if (fm2 > max_pwm) max_pwm = fm2;
    if (fm3 > max_pwm) max_pwm = fm3;
    if (fm4 > max_pwm) max_pwm = fm4;

    if (max_pwm > (float)PWM_MAX_US) {
        float offset = max_pwm - (float)PWM_MAX_US;
        fm1 -= offset;
        fm2 -= offset;
        fm3 -= offset;
        fm4 -= offset;
    }

    /* ── Giới hạn cuối cùng ──────────────────────────────────────────────
     * Bảo vệ phần cứng: không để PWM vượt ngoài dải cho phép
     * PWM_IDLE_US là tốc độ tối thiểu khi đang bay (để motor không dừng giữa chừng) */
    #define CLAMP_PWM(x) ((uint16_t)( \
        (x) < (float)PWM_IDLE_US ? PWM_IDLE_US : \
        ((x) > (float)PWM_MAX_US ? PWM_MAX_US : (uint16_t)(x))))

    *m1 = CLAMP_PWM(fm1);
    *m2 = CLAMP_PWM(fm2);
    *m3 = CLAMP_PWM(fm3);
    *m4 = CLAMP_PWM(fm4);

    #undef CLAMP_PWM
}


/* ============================================================================
 * PHẦN 5: DRIVER MPU-6050 (I2C)
 * ============================================================================
 *
 * MPU-6050 là IMU 6-DOF:
 *     - 3-axis Gyroscope  : đo tốc độ góc (°/s)
 *     - 3-axis Accelerometer: đo gia tốc (g)
 *
 * Giao tiếp: I2C tại địa chỉ 0x68 (AD0=GND) hoặc 0x69 (AD0=VCC)
 *
 * Trình tự khởi tạo:
 *     1. Wake up chip (tắt sleep mode)
 *     2. Chọn PLL Gyro làm clock source (ổn định hơn oscillator nội)
 *     3. Cấu hình DLPF phần cứng (tầng lọc đầu tiên trước LPF phần mềm)
 *     4. Cấu hình độ nhạy Gyro và Accel
 *
 * DLPF (Digital Low-Pass Filter nội của chip):
 *     Cfg=3 → BW=44Hz — tầng lọc phần cứng đầu tiên,
 *     phối hợp với LPF phần mềm 70Hz để lọc nhiễu 2 tầng.
 */

/* Địa chỉ và thanh ghi quan trọng */
#define MPU6050_ADDR        0x68
#define REG_PWR_MGMT_1      0x6B
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_SMPLRT_DIV      0x19
#define REG_DATA_START      0x3B    /* ACCEL_XOUT_H — đọc 14 bytes liên tiếp */

/* Độ nhạy Gyro ±500 °/s → 65.5 LSB/(°/s) */
#define GYRO_SCALE          65.5f
/* Độ nhạy Accel ±2g → 16384 LSB/g */
#define ACCEL_SCALE         16384.0f

#define I2C_PORT            I2C_NUM_0
#define I2C_TIMEOUT_MS      10

/* Struct chứa dữ liệu thô đã quy đổi đơn vị */
typedef struct {
    float gyro_x;       /* Roll rate  (°/s) */
    float gyro_y;       /* Pitch rate (°/s) */
    float gyro_z;       /* Yaw rate   (°/s) */
    float accel_x;      /* (g)              */
    float accel_y;      /* (g)              */
    float accel_z;      /* (g)              */
} IMURaw;

/* Ghi 1 byte vào thanh ghi */
static esp_err_t mpu_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, MPU6050_ADDR,
                                       buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* Đọc nhiều byte liên tiếp từ thanh ghi */
static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, MPU6050_ADDR,
                                         &reg, 1, data, len,
                                         pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

/* Khởi tạo I2C bus */
static void i2c_init(void)
{
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = PIN_SDA,
        .scl_io_num       = PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,  /* 400 kHz Fast Mode */
    };
    i2c_param_config(I2C_PORT, &cfg);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

/* Khởi tạo MPU-6050 */
static void mpu6050_init(void)
{
    /* 1. Wake up + dùng PLL Gyro X làm clock source (0x01) */
    mpu_write(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 2. DLPF phần cứng: Cfg=3 → Bandwidth 44Hz */
    mpu_write(REG_CONFIG, 0x03);

    /* 3. Gyro: ±500 °/s → GYRO_CONFIG[4:3] = 01 → 0x08 */
    mpu_write(REG_GYRO_CONFIG, 0x08);

    /* 4. Accel: ±2g → ACCEL_CONFIG[4:3] = 00 → 0x00 */
    mpu_write(REG_ACCEL_CONFIG, 0x00);

    /* 5. Sample Rate = 1000Hz (SMPLRT_DIV=0, DLPF bật → gyro rate=1kHz) */
    mpu_write(REG_SMPLRT_DIV, 0x00);
}

/*
 * Đọc dữ liệu thô từ MPU-6050 (burst read 14 bytes).
 *
 * Thứ tự bytes (big-endian, 2 bytes/axis):
 *   [0-1] AX  [2-3] AY  [4-5] AZ
 *   [6-7] TEMP (bỏ qua)
 *   [8-9] GX  [10-11] GY  [12-13] GZ
 */
static esp_err_t mpu6050_read(IMURaw *out)
{
    uint8_t buf[14];
    esp_err_t ret = mpu_read(REG_DATA_START, buf, 14);
    if (ret != ESP_OK) return ret;

    /* Ghép 2 bytes big-endian → int16 */
    #define TO_INT16(h, l) ((int16_t)(((uint16_t)(h) << 8) | (l)))

    int16_t ax = TO_INT16(buf[0],  buf[1]);
    int16_t ay = TO_INT16(buf[2],  buf[3]);
    int16_t az = TO_INT16(buf[4],  buf[5]);
    int16_t gx = TO_INT16(buf[8],  buf[9]);
    int16_t gy = TO_INT16(buf[10], buf[11]);
    int16_t gz = TO_INT16(buf[12], buf[13]);

    #undef TO_INT16

    /* Chuyển sang đơn vị vật lý */
    out->accel_x = (float)ax / ACCEL_SCALE;    /* (g) */
    out->accel_y = (float)ay / ACCEL_SCALE;
    out->accel_z = (float)az / ACCEL_SCALE;
    out->gyro_x  = (float)gx / GYRO_SCALE;     /* (°/s) */
    out->gyro_y  = (float)gy / GYRO_SCALE;
    out->gyro_z  = (float)gz / GYRO_SCALE;

    return ESP_OK;
}

/*
 * Hiệu chỉnh Gyro (Gyro Calibration) — tính offset khi khởi động.
 *
 * Gyro có offset nhà máy (bias) — cần trừ đi trước khi dùng.
 * QUAN TRỌNG: Drone phải nằm hoàn toàn yên trong quá trình này!
 *
 * Args:
 *     gx_off/gy_off/gz_off : Đầu ra — offset cần trừ đi sau này
 *     n_samples            : Số mẫu lấy trung bình (thường 500–2000)
 */
static void gyro_calibrate(float *gx_off, float *gy_off, float *gz_off,
                             int n_samples)
{
    double sx = 0, sy = 0, sz = 0;
    IMURaw raw;

    for (int i = 0; i < n_samples; i++) {
        if (mpu6050_read(&raw) == ESP_OK) {
            sx += raw.gyro_x;
            sy += raw.gyro_y;
            sz += raw.gyro_z;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    *gx_off = (float)(sx / n_samples);
    *gy_off = (float)(sy / n_samples);
    *gz_off = (float)(sz / n_samples);
}


/* ============================================================================
 * PHẦN 6: PWM CHO ESC — LEDC PERIPHERAL CỦA ESP32
 * ============================================================================
 *
 * ESP32 dùng peripheral LEDC (vốn dùng cho LED PWM) để tạo xung PWM
 * cho ESC. Tần số 50Hz (period 20ms) là chuẩn cho ESC analog.
 *
 * Duty cycle 16-bit:
 *     duty = (pulse_µs / 20000µs) × 65535
 *     1000µs → duty = 3277  (motor dừng)
 *     2000µs → duty = 6554  (full throttle)
 */

/* Khởi tạo LEDC PWM cho 4 motor — tất cả bắt đầu ở 1000µs */
static void motor_pwm_init(void)
{
    /* Cấu hình Timer: 50Hz, 16-bit */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    /* Cấu hình 4 channel */
    const int pins[4]     = { PIN_MOTOR_M1, PIN_MOTOR_M2, PIN_MOTOR_M3, PIN_MOTOR_M4 };
    const int channels[4] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1,
                               LEDC_CHANNEL_2, LEDC_CHANNEL_3 };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch = {
            .gpio_num   = pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = channels[i],
            .timer_sel  = LEDC_TIMER_0,
            .duty       = US_TO_DUTY(PWM_MIN_US),
            .hpoint     = 0,
        };
        ledc_channel_config(&ch);
    }
}

/* Ghi PWM ra 4 motor cùng lúc */
static void motor_pwm_write(uint16_t m1, uint16_t m2,
                              uint16_t m3, uint16_t m4)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, US_TO_DUTY(m1));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, US_TO_DUTY(m2));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, US_TO_DUTY(m3));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, US_TO_DUTY(m4));

    /* Cập nhật tất cả cùng lúc để tránh jitter */
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

/* Tắt toàn bộ motor — gọi khi DISARM */
static void motor_pwm_disarm(void)
{
    motor_pwm_write(PWM_MIN_US, PWM_MIN_US, PWM_MIN_US, PWM_MIN_US);
}


/* ============================================================================
 * PHẦN 7: HỆ THỐNG ĐIỀU KHIỂN TỔNG THỂ
 * ============================================================================
 *
 * Struct này tích hợp tất cả thành phần và phản ánh đúng class
 * DroneFlightController trong file Python gốc.
 *
 * LUỒNG DỮ LIỆU TRONG MỖI CHU KỲ:
 *
 * 1. Đọc cảm biến thô (Gyro + Accel) từ IMU
 * 2. Trừ offset calibration
 * 3. Lọc Gyro qua Low-Pass Filter (khử nhiễu tần số cao)
 * 4. Tính góc Accel (Pitch, Roll) làm mốc tuyệt đối
 * 5. Cập nhật Complementary Filter → góc tối ưu
 * 6. Nhận tín hiệu RC (góc mong muốn, throttle)
 * 7. Cascaded PID tính đầu ra cho Pitch, Roll, Yaw
 * 8. Motor Mixing phân phối PWM cho 4 motor
 * 9. Gửi PWM đến ESC
 */
typedef struct {
    /* ── Bộ lọc nhiễu: mỗi trục cần bộ lọc riêng ─────────────────────── */
    LowPassFilter lpf_pitch;
    LowPassFilter lpf_roll;
    LowPassFilter lpf_yaw;

    /* ── Complementary Filter (chỉ cần cho Pitch và Roll) ──────────────
     * Yaw không dùng Comp Filter vì Accel không cho góc Yaw
     * (trọng lực chỉ cho thông tin về pitch/roll, không về heading) */
    ComplementaryFilter comp_pitch;
    ComplementaryFilter comp_roll;

    /* ── Cascaded PID cho 3 trục ─────────────────────────────────────── */
    CascadedPIDAxis pid_pitch;
    CascadedPIDAxis pid_roll;

    /* YAW: Không có Angle Loop (dùng Rate mode cho Yaw)
     * Vì: Drone không có mốc heading tuyệt đối từ IMU (cần Compass)
     * Rate mode: stick điều khiển trực tiếp TỐC ĐỘ XOAY, không phải hướng */
    PIDController pid_yaw;

    /* ── Trạng thái hệ thống ────────────────────────────────────────── */
    float angle_pitch;          /* Góc pitch hiện tại (độ) */
    float angle_roll;           /* Góc roll hiện tại  (độ) */
    float gyro_offset_x;        /* Offset calibration Gyro */
    float gyro_offset_y;
    float gyro_offset_z;
    int   is_armed;

} DroneFlightController;

/* Khởi tạo toàn bộ Flight Controller */
void fc_init(DroneFlightController *fc)
{
    /* ── Low-Pass Filter: cutoff 70Hz ──────────────────────────────────
     * Lọc bỏ rung động cơ >70Hz, giữ lại chuyển động thực <70Hz */
    lpf_init(&fc->lpf_pitch, 70.0f);
    lpf_init(&fc->lpf_roll,  70.0f);
    lpf_init(&fc->lpf_yaw,   70.0f);

    /* ── Complementary Filter: alpha=0.98 ──────────────────────────────
     * 98% Gyro + 2% Accel là tỷ lệ chuẩn công nghiệp */
    comp_filter_init(&fc->comp_pitch, 0.98f);
    comp_filter_init(&fc->comp_roll,  0.98f);

    /* ── Cascaded PID: Pitch ────────────────────────────────────────────
     * Các hệ số này là starting point điển hình cho drone 5" FPV racing.
     * Bắt buộc phải TUNING cho từng drone cụ thể sau khi lắp ráp.
     *
     * HƯỚNG DẪN TUNING (bắt đầu từ đây):
     *   Kp_angle cao  → phản ứng nhanh với stick, nhưng dễ "bật"
     *   rate_Kp cao   → cứng, chống nhiễu tốt, nhưng rung nếu quá cao
     *   rate_Ki cao   → bù drift tốt, nhưng overshoot nếu quá cao
     *   rate_Kd cao   → phanh tốt, nhưng khuếch đại nhiễu nếu quá cao */
    cascaded_pid_init(&fc->pid_pitch,
        /* angle_kp     = */ 15.0f,
        /* rate_kp      = */ 0.65f,
        /* rate_ki      = */ 2.5f,
        /* rate_kd      = */ 0.018f,
        /* max_rate_°/s = */ 500.0f);

    /* ── Cascaded PID: Roll ─────────────────────────────────────────────
     * Giống Pitch vì drone đối xứng X-frame */
    cascaded_pid_init(&fc->pid_roll,
        15.0f, 0.65f, 2.5f, 0.018f, 500.0f);

    /* ── PID đơn: Yaw ───────────────────────────────────────────────────
     * Kd = 0 cho Yaw: ít quán tính hơn + không muốn khuếch đại nhiễu heading */
    pid_init(&fc->pid_yaw,
        /* kp             = */ 3.5f,
        /* ki             = */ 8.0f,
        /* kd             = */ 0.0f,
        /* output_limit   = */ 200.0f,
        /* integral_limit = */ 100.0f);

    fc->angle_pitch    = 0.0f;
    fc->angle_roll     = 0.0f;
    fc->gyro_offset_x  = 0.0f;
    fc->gyro_offset_y  = 0.0f;
    fc->gyro_offset_z  = 0.0f;
    fc->is_armed       = 0;
}

/* ARM drone: reset tất cả PID và cho phép motor chạy */
void fc_arm(DroneFlightController *fc)
{
    cascaded_pid_reset(&fc->pid_pitch);
    cascaded_pid_reset(&fc->pid_roll);
    pid_reset(&fc->pid_yaw);
    fc->angle_pitch = 0.0f;
    fc->angle_roll  = 0.0f;
    fc->is_armed    = 1;
    ESP_LOGI(TAG, "[FC] Drone ARMED");
}

/* DISARM drone: dừng tất cả motor */
void fc_disarm(DroneFlightController *fc)
{
    fc->is_armed = 0;
    motor_pwm_disarm();
    ESP_LOGI(TAG, "[FC] Drone DISARMED");
}

/*
 * Chu kỳ xử lý chính của Flight Controller.
 * Gọi hàm này liên tục trong vòng lặp while(1).
 *
 * Args:
 *     fc          : Con trỏ tới DroneFlightController
 *     gyro_x/y/z  : Tốc độ góc thô từ Gyroscope (°/s): x=Roll, y=Pitch, z=Yaw
 *     accel_x/y/z : Gia tốc thô từ Accelerometer (g)
 *     rc_throttle : Tín hiệu ga từ RC (1000–2000 µs)
 *     rc_pitch    : Góc pitch mong muốn từ RC (-MAX_ANGLE đến +MAX_ANGLE, độ)
 *     rc_roll     : Góc roll  mong muốn từ RC (-MAX_ANGLE đến +MAX_ANGLE, độ)
 *     rc_yaw      : Tốc độ yaw mong muốn từ RC (-MAX_YAW_RATE đến +MAX_YAW_RATE, °/s)
 *     dt_angle    : Chu kỳ của Angle Loop (giây)
 *     dt_rate     : Chu kỳ của Rate Loop  (giây)
 */
void fc_update(DroneFlightController *fc,
               float gyro_x,    float gyro_y,    float gyro_z,
               float accel_x,   float accel_y,   float accel_z,
               float rc_throttle,
               float rc_pitch,  float rc_roll,   float rc_yaw,
               float dt_angle,  float dt_rate)
{
    if (!fc->is_armed) {
        motor_pwm_disarm();
        return;
    }

    /* Throttle quá thấp: giữ motor idle, reset PID để tránh windup tích lũy */
    if (rc_throttle < 1050.0f) {
        motor_pwm_disarm();
        cascaded_pid_reset(&fc->pid_pitch);
        cascaded_pid_reset(&fc->pid_roll);
        pid_reset(&fc->pid_yaw);
        return;
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 1: TRỪ OFFSET CALIBRATION
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Gyro có bias nhà máy → trừ đi offset đã tính lúc khởi động */
    gyro_x -= fc->gyro_offset_x;
    gyro_y -= fc->gyro_offset_y;
    gyro_z -= fc->gyro_offset_z;

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 2: LỌC DỮ LIỆU GYRO (Tầng 1 — Low-Pass Filter)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Dữ liệu thô từ Gyro chứa nhiễu rung từ motor (>100Hz).
     * LPF giữ lại tín hiệu thực (<70Hz) và loại bỏ nhiễu cao tần. */
    float gyro_pitch_f = lpf_update(&fc->lpf_pitch, gyro_y, dt_angle);  /* y = Pitch */
    float gyro_roll_f  = lpf_update(&fc->lpf_roll,  gyro_x, dt_angle);  /* x = Roll  */
    float gyro_yaw_f   = lpf_update(&fc->lpf_yaw,   gyro_z, dt_angle);  /* z = Yaw   */

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 3: TÍNH GÓC TỪ ACCELEROMETER (Mốc tuyệt đối)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Accel đo hướng của vector trọng lực → suy ra góc nghiêng so với ngang
     * Công thức: pitch = atan2(ax, az),  roll = atan2(ay, az) */
    float accel_pitch = accel_to_angle_pitch(accel_x, accel_z);
    float accel_roll  = accel_to_angle_roll (accel_y, accel_z);

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 4: COMPLEMENTARY FILTER (Tầng 2 — Ước lượng góc tối ưu)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Kết hợp Gyro (nhanh, không drift) + Accel (chậm, có mốc tuyệt đối) */
    fc->angle_pitch = comp_filter_update(&fc->comp_pitch,
                                          gyro_pitch_f, accel_pitch, dt_angle);
    fc->angle_roll  = comp_filter_update(&fc->comp_roll,
                                          gyro_roll_f,  accel_roll,  dt_angle);

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 5: CASCADED PID — PITCH & ROLL
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    float pitch_ctrl = cascaded_pid_compute(&fc->pid_pitch,
        rc_pitch,           /* Góc pitch mong muốn từ RC    */
        fc->angle_pitch,    /* Góc pitch thực từ Comp Filter */
        gyro_pitch_f,       /* Tốc độ pitch thực từ Gyro     */
        dt_angle, dt_rate);

    float roll_ctrl = cascaded_pid_compute(&fc->pid_roll,
        rc_roll,
        fc->angle_roll,
        gyro_roll_f,
        dt_angle, dt_rate);

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 6: PID YAW (Rate mode — điều khiển tốc độ xoay)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * Yaw dùng Single Loop: rc_yaw là TỐC ĐỘ XOAY mong muốn (°/s),
     * không phải góc tuyệt đối (vì không có Compass) */
    float yaw_ctrl = pid_compute(&fc->pid_yaw,
        rc_yaw,
        gyro_yaw_f,
        dt_rate);

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 7: MOTOR MIXING — Phân phối tín hiệu cho 4 motor
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    uint16_t m1, m2, m3, m4;
    motor_mix(rc_throttle, pitch_ctrl, roll_ctrl, yaw_ctrl,
              &m1, &m2, &m3, &m4);

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     * BƯỚC 8: GỬI PWM ĐẾN ESC
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    motor_pwm_write(m1, m2, m3, m4);

    /* Debug: in thông tin ra UART mỗi 500 chu kỳ */
    static uint32_t dbg_count = 0;
    if (++dbg_count % 500 == 0) {
        ESP_LOGI(TAG, "P=%.1f° R=%.1f° | ctrl(p=%.1f r=%.1f y=%.1f) | M1=%d M2=%d M3=%d M4=%d",
                 fc->angle_pitch, fc->angle_roll,
                 pitch_ctrl, roll_ctrl, yaw_ctrl,
                 m1, m2, m3, m4);
    }
}


/* ============================================================================
 * PHẦN 8: VÒNG LẶP CHÍNH (APP_MAIN)
 * ============================================================================
 *
 * Thay thế hàm simulate_imu_sensor() và run_simulation() trong Python
 * bằng đọc phần cứng thực tế và vòng lặp while(1) trên ESP32.
 *
 * Kịch bản khởi động:
 *   1. Khởi tạo I2C và MPU-6050
 *   2. Chờ 2 giây, đặt drone nằm yên, calibrate Gyro
 *   3. Khởi tạo PWM cho 4 ESC
 *   4. Arm drone
 *   5. Vòng lặp điều khiển 4 kHz
 *
 * Tín hiệu RC:
 *   Phiên bản này dùng giá trị mock cố định để dễ test.
 *   Thay bằng đọc SBUS/PPM từ UART khi có receiver thực tế.
 *
 * Hướng dẫn PID Tuning (tương tự Python):
 *   Kp quá thấp → phản ứng chậm, drone lắc nhẹ
 *   Kp quá cao  → dao động, rung lắc mạnh (oscillation)
 *   Ki quá cao  → vượt ngưỡng (overshoot), khó ổn định
 *   Kd quá cao  → khuếch đại nhiễu, giật mạnh
 *   Kd quá thấp → không đủ phanh, overshoot
 */
void app_main(void)
{
    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "  ESP32 Drone Flight Controller — Cascaded PID");
    ESP_LOGI(TAG, "==============================================");

    /* ── Khởi tạo phần cứng ──────────────────────────────────────────── */
    i2c_init();
    mpu6050_init();
    motor_pwm_init();
    ESP_LOGI(TAG, "Hardware initialized");

    /* ── Khởi tạo Flight Controller ──────────────────────────────────── */
    DroneFlightController fc;
    fc_init(&fc);

    /* ── Calibrate Gyro ──────────────────────────────────────────────────
     * Đặt drone nằm yên trên mặt phẳng — tuyệt đối không được chạm!
     * Lấy 2000 mẫu ≈ 2 giây */
    ESP_LOGI(TAG, "Calibrating gyro... Keep drone STILL for 2 seconds!");
    gyro_calibrate(&fc.gyro_offset_x, &fc.gyro_offset_y, &fc.gyro_offset_z, 2000);
    ESP_LOGI(TAG, "Gyro offset: gx=%.3f gy=%.3f gz=%.3f",
             fc.gyro_offset_x, fc.gyro_offset_y, fc.gyro_offset_z);

    /* ── ARM ─────────────────────────────────────────────────────────────
     * Trong ứng dụng thực: chỉ arm khi nhận tín hiệu arm từ RC
     * (ví dụ: throttle thấp + yaw phải giữ 2 giây) */
    fc_arm(&fc);

    /* ── Vòng lặp điều khiển chính ───────────────────────────────────── */
    IMURaw    imu;
    int64_t   t_prev = esp_timer_get_time();
    uint32_t  angle_count = 0;

    while (1) {

        /* Đo dt thực tế (microseconds → seconds) */
        int64_t t_now = esp_timer_get_time();
        float   dt    = (float)(t_now - t_prev) * 1e-6f;
        t_prev = t_now;

        /* Đọc IMU */
        if (mpu6050_read(&imu) != ESP_OK) {
            ESP_LOGE(TAG, "IMU read error — skip cycle");
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /*
         * ── TÍN HIỆU RC (MOCK) ──────────────────────────────────────────
         * Thay thế bằng đọc SBUS/PPM thực tế từ UART khi có receiver.
         *
         * Ví dụ SBUS (100kbps, inverted, 25 bytes/frame):
         *   uart_read_bytes(UART_NUM_1, buf, 25, pdMS_TO_TICKS(5));
         *   Rồi parse frame → channel values → map về µs.
         *
         * Giá trị mock: drone hover thẳng tại throttle 50%.
         */
        float rc_throttle = 1500.0f;    /* 50% throttle */
        float rc_pitch    = 0.0f;       /* Muốn bay thẳng, pitch 0° */
        float rc_roll     = 0.0f;
        float rc_yaw      = 0.0f;

        /*
         * ── dt CHO HAI VÒNG ──────────────────────────────────────────────
         * Trong firmware đơn giản (không RTOS), cả hai vòng chạy cùng chu kỳ.
         * Dùng dt thực đo được cho Angle Loop và DT_RATE cố định cho Rate Loop.
         * Kết quả đủ tốt cho drone thông thường.
         * Nâng cao: Rate Loop có thể chạy trong interrupt timer riêng ở 4kHz.
         */
        angle_count++;
        float dt_angle = (angle_count % 4 == 0) ? dt : DT_RATE;
        float dt_rate  = DT_RATE;

        /* ── Chạy một chu kỳ điều khiển ───────────────────────────────── */
        fc_update(&fc,
                  imu.gyro_x,  imu.gyro_y,  imu.gyro_z,
                  imu.accel_x, imu.accel_y, imu.accel_z,
                  rc_throttle,
                  rc_pitch, rc_roll, rc_yaw,
                  dt_angle, dt_rate);

        /* ── Giữ tần số ~4kHz (250µs/chu kỳ) ───────────────────────────
         * esp_timer_get_time() đo µs — bận chờ chính xác hơn vTaskDelay */
        while ((esp_timer_get_time() - t_now) < 250) {
            /* busy-wait ngắn */
        }
    }
}
