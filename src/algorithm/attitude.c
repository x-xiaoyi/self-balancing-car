#include "algorithm/attitude.h"

#include <math.h>       /* atan2f / sqrtf —— 算角度就该出现在这一层 */

/* 互补滤波权重：
 * α 越大越信陀螺（更平滑，但漂移纠得慢）；α 越小越信加速度（更跟手，但更抖）。
 * dt=10ms 时时间常数 τ = α·dt/(1-α) ≈ 0.49s —— 意即"陀螺漂出来的偏差，
 * 大约半秒被加速度拽回来一次"。 */
#define ATT_ALPHA       0.98f

/* 弧度转角度（180/π）。不用 M_PI：它在严格 C 标准里不保证存在，直接写常量最省心 */
#define ATT_RAD_TO_DEG  57.29577951f

/* 滤波状态：必须跨帧保存！
 * 要是写成函数里的局部变量，每帧都从 0 开始，那叫"每帧重新算一遍"，不叫滤波。 */
static Attitude_t s_att;

/* 加速度算静态角（前提：此时主要是重力，没有大的运动加速度）。
 *   Roll  = atan2(ay, az)                     绕 X 轴
 *   Pitch = atan2(-ax, sqrt(ay*ay + az*az))   绕 Y 轴
 * Pitch 为什么要 sqrt "打包"：板子同时 Roll 时，重力会散到 X/Y/Z 三轴上，
 * 直接用 az 当参考是错的；sqrt(ay²+az²) 才是"垂直于 X 的平面里的重力投影总长"。
 * 那根 -ax 的负号来自三个约定：X 轴指前、重力永远向下、"仰头定为正 Pitch"。 */
static void accel_to_roll_pitch(const MpuData_t *imu, float *roll, float *pitch)
{
    float ax = imu->accel[0];
    float ay = imu->accel[1];
    float az = imu->accel[2];

    *roll  = atan2f(ay, az) * ATT_RAD_TO_DEG;
    *pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * ATT_RAD_TO_DEG;
}

void Attitude_Init(const MpuData_t *first)
{
    /* 初值直接取第一帧的加速度角，而不是 0 ——
     * 否则滤波器要从 0 慢慢"爬"到真实姿态，开头那一段角度是假的。 */
    accel_to_roll_pitch(first, &s_att.roll_reg, &s_att.pitch_reg);
    s_att.yaw_reg = 0.0f;
}

void Attitude_Update(const MpuData_t *imu, float dt_s, Attitude_t *out)
{
    float roll_acc;
    float pitch_acc;
    float roll_from_gyro;
    float pitch_from_gyro;

    accel_to_roll_pitch(imu, &roll_acc, &pitch_acc);

    /* 陀螺积分：θ = θ_prev + ω·dt
     * 量纲：ω 是 °/s，dt 是 s，乘起来才是 °。
     * Roll 绕 X 轴 → 用 gyro[0]；Pitch 绕 Y 轴 → 用 gyro[1]。
     * ⚠️ 如果上板发现角度往反方向跑，就是这里的符号反了，把该项取负。 */
    roll_from_gyro  = s_att.roll_reg  + imu->gyro[0] * dt_s;
    pitch_from_gyro = s_att.pitch_reg + imu->gyro[1] * dt_s;

    /* 互补滤波：陀螺管短时、加速度管长期 */
    s_att.roll_reg  = ATT_ALPHA * roll_from_gyro  + (1.0f - ATT_ALPHA) * roll_acc;
    s_att.pitch_reg = ATT_ALPHA * pitch_from_gyro + (1.0f - ATT_ALPHA) * pitch_acc;

    *out = s_att;
}
