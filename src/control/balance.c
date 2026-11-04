#include "control/balance.h"

#include "algorithm/attitude.h"
#include "algorithm/pid.h"
#include "bsp/bsp_mpu.h"
#include "motor/motor.h"

/* ============================================================================
 * 控制拍本体（TIM4，200Hz）
 *
 * 一定要记住这里的执行环境：这是中断，不是主循环。
 *  - 不许 printf（一句浮点打印 ~100us 起，5ms 周期经不起几下；而且 printf 是阻塞的，
 *    会把整拍拖长，dt 直接失真）；
 *  - 不许 HAL_Delay（同理会锁死节拍）；
 *  - 所有和主循环共享的变量都按"中断写、主循环读快照"的方式处理，
 *    快照用 PRIMASK 短暂关中断来保证不撕裂，见 Balance_GetTelemetry。
 *
 * I2C 读在这里是阻塞的：14 字节突发读 @100kHz 大约 1.5ms，占 5ms 周期的 ~30%。
 * 能接受，但这也意味着"控制拍里再塞别的重活"就没有余量了。
 * ========================================================================== */

/* 标称控制周期。为什么用常数而不是"HAL_GetTick 差值"：
 *   ① 5ms 里只可能跨 0 或 1 个 SysTick，差值量化误差最大 100%（要么 4ms 要么 5ms），
 *      拿它当 dt 去积分/微分，等于每拍给控制量加一次随机扰动；
 *      定时器的 5ms 是硬件精确的（72MHz / 72 / 5000，零取整误差），用标称值反而更准。
 *   ② 顺带把"dt = 0 → 除零"这类问题从根上消掉了：常数永远不是 0。
 *   HAL_GetTick 只用来数"丢了几拍"，不参与运算。 */
#define BAL_CTRL_DT_S               (0.005f)

/* 丢拍判定阈值（ms）：正常一拍 5ms；超过 8ms 说明至少漏了一整拍。
 * 之所以不是 >5 就判：SysTick 是 1ms 粒度，5ms 的拍子量出来经常是 5 或 6，
 * 拿 5 当门限会天天误报。 */
#define BAL_OVERRUN_MS              (8U)

/* ---------------- 模块私有状态 ---------------- */

static Pid_t              s_pid_angle;          /* 直立环：PD，D 用陀螺角速度 */
static Pid_t              s_pid_steer;          /* 转向环：纯 P */

static BalanceTelemetry_t s_tele;
static uint8_t            s_state    = BAL_STATE_INIT;
static uint8_t            s_fault    = BAL_FAULT_NONE;
static uint8_t            s_fault_pending = BAL_FAULT_NONE;

static uint8_t            s_inited   = 0U;
static uint32_t           s_ramp_start_ms = 0U;
static uint32_t           s_last_tick_ms  = 0U;
static uint8_t            s_imu_fail_run  = 0U;   /* 连续失败计数（成功一次就清零） */

/* ---------------- 小工具 ---------------- */

/* 不引 math.h：整个工程只需要一个绝对值，没必要为了 fabsf 把 libm 拖进来 */
static float bal_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* 对称限幅 */
static float bal_clampf(float v, float lim)
{
    if (v >  lim) { return  lim; }
    if (v < -lim) { return -lim; }
    return v;
}

/* 唯一的输出口。DRY_RUN 下这里什么都不做 —— 整个工程里只有这一处能驱动电机，
 * 这样"只算不输出"才是一个可以一眼看死的开关，而不是散落在各处的 #if。 */
static void bal_output(int16_t left, int16_t right)
{
#if (CAR_DRY_RUN != 0)
    (void)left;
    (void)right;
#else
    Motor_SetSigned(left, right);
#endif
}

/* 锁存故障：进 FAULT 后不再接受任何恢复路径（除了 Balance_ClearFault / 重新上电）。
 * 顺序讲究：先停机，再清 PID —— 反过来的话，停机那一拍还是带着旧积分算出来的量。 */
static void bal_latch_fault(uint8_t code)
{
    if (s_state == BAL_STATE_FAULT)
    {
        return;                     /* 已经是 FAULT，保留第一次的原因，不要被后续故障覆盖 */
    }

    s_fault         = code;
    s_fault_pending = code;         /* 交给主循环打印一次 */
    s_state         = BAL_STATE_FAULT;

    bal_output(0, 0);
    Pid_Reset(&s_pid_angle);
    Pid_Reset(&s_pid_steer);

    /* 遥测里的状态/故障码也在这里一起落下来。
     * 不写这两行的话，快照里 state 会是 FAULT 但 fault 还是 NONE ——
     * 打印出来看不出原因，事后看日志会以为是"无缘无故锁死"。 */
    s_tele.state     = s_state;
    s_tele.fault     = code;
    s_tele.duty_left  = 0;
    s_tele.duty_right = 0;
    s_tele.angle_out  = 0.0f;
    s_tele.steer_out  = 0.0f;
    s_tele.ramp_pct   = 0U;
}

/* ---------------- 对外接口：初始化 ---------------- */

HAL_StatusTypeDef Balance_Init(void)
{
    /* 直立环：微分作用于测量值，并用外部角速度（陀螺）当微分源。
     * out_max = ±3600（CCR 满量程）；integ_max = 900，即积分最多贡献 1/4 满量程
     * ——即使有人把 Ki 打开了，也不至于攒出一个"自己在跑"的偏置。 */
    Pid_Init(&s_pid_angle,
             BAL_ANGLE_KP, BAL_ANGLE_KI, BAL_ANGLE_KD,
             BAL_MAX_DUTY, BAL_MAX_DUTY * 0.25f,
             1U);

    /* 转向环：纯 P（Ki=Kd=0）。out_max 用 BAL_STEER_LIMIT，
     * 这样"转向最多能抢 600/3600 的差速"是写死在 PID 里的，不靠后面再夹一次。 */
    Pid_Init(&s_pid_steer,
             BAL_STEER_KP, 0.0f, 0.0f,
             BAL_STEER_LIMIT, 0.0f,
             1U);

    s_tele.pitch_deg      = 0.0f;
    s_tele.pitch_rate_dps = 0.0f;
    s_tele.yaw_rate_dps   = 0.0f;
    s_tele.angle_out      = 0.0f;
    s_tele.steer_out      = 0.0f;
    s_tele.duty_left      = 0;
    s_tele.duty_right     = 0;
    s_tele.state          = BAL_STATE_WAIT_UPRIGHT;
    s_tele.fault          = BAL_FAULT_NONE;
    s_tele.ramp_pct       = 0U;
    s_tele.cycle_count    = 0U;
    s_tele.overrun_count  = 0U;
    s_tele.imu_fail_count = 0U;

    s_state          = BAL_STATE_WAIT_UPRIGHT;
    s_fault          = BAL_FAULT_NONE;
    s_fault_pending  = BAL_FAULT_NONE;
    s_imu_fail_run   = 0U;
    s_ramp_start_ms  = HAL_GetTick();
    s_last_tick_ms   = HAL_GetTick();
    s_inited         = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef Balance_StartControl(void)
{
    if (s_inited == 0U)
    {
        return HAL_ERROR;           /* 没 Init 就起节拍 = 拿着一堆未初始化的 PID 去驱动电机 */
    }
    return BspTim_StartControlTick(Balance_Tick);
}

void Balance_Stop(void)
{
    BspTim_StopControlTick();
    bal_output(0, 0);
    Pid_Reset(&s_pid_angle);
    Pid_Reset(&s_pid_steer);
}

/* ---------------- 控制拍 ---------------- */

void Balance_Tick(void)
{
    MpuData_t  imu = {0};
    Attitude_t att = {0};
    uint32_t   now;
    uint32_t   dms;
    float      pitch;
    float      rate;
    float      yaw_rate;
    int16_t    out_left  = 0;
    int16_t    out_right = 0;

    if (s_inited == 0U)
    {
        return;
    }

    s_tele.cycle_count++;

    /* ① 丢拍统计：只用 HAL_GetTick 的差值判"漏了几拍"，不用它算 dt（理由见文件头） */
    now = HAL_GetTick();
    dms = now - s_last_tick_ms;     /* 无符号相减，49 天回绕也是对的 */
    s_last_tick_ms = now;
    if (dms > BAL_OVERRUN_MS)
    {
        s_tele.overrun_count++;
    }

    /* ② 采样。这是全工程唯一在读 MPU 的地方（起控之后），主循环不许再碰。 */
    if (BspMpu_GetData(&imu) != HAL_OK)
    {
        s_tele.imu_fail_count++;
        if (s_imu_fail_run < 255U)
        {
            s_imu_fail_run++;
        }

        /* 读失败了：本拍绝对不能沿用上一拍的角度去算输出（那等于闭眼开车下坡）。
         * 直接判零输出；连续失败到阈值就锁存停机。 */
        bal_output(0, 0);
        s_tele.duty_left  = 0;
        s_tele.duty_right = 0;

        if (s_imu_fail_run >= BAL_IMU_FAIL_LIMIT)
        {
            bal_latch_fault(BAL_FAULT_IMU);     /* 里面会把 state/fault 一起写进快照 */
        }
        else
        {
            s_tele.state = s_state;
        }
        return;
    }
    s_imu_fail_run = 0U;

    /* ③ 姿态融合：用标称 dt（5ms 恒定），滤波器的时间常数才是设计值 */
    Attitude_Update(&imu, BAL_CTRL_DT_S, &att);

    /* 符号统一在这里处理：翻转 BAL_PITCH_DIR 等于同时翻 pitch 和 gyro_y，
     * P 项和 D 项的符号才不会打架。 */
    pitch    = att.pitch_reg * BAL_PITCH_DIR;
    rate     = imu.gyro[1]   * BAL_PITCH_DIR;   /* 绕 Y 轴角速度 = pitch 的变化率 */
    yaw_rate = imu.gyro[2]   * BAL_YAW_DIR;     /* 绕 Z 轴角速度 = 转向 */

    s_tele.pitch_deg      = pitch;
    s_tele.pitch_rate_dps = rate;
    s_tele.yaw_rate_dps   = yaw_rate;

    /* ④ 状态机 */
    switch ((BalanceState_t)s_state)
    {
    case BAL_STATE_WAIT_UPRIGHT:
        /* 只有"被扶正"才允许起控。侧躺/倒扣的时候永远停在这一步，输出保持 0。 */
        if (bal_absf(pitch) <= BAL_UPRIGHT_ARM_DEG)
        {
            /* 起控前把 PID 历史清干净：上一轮（上一次倾倒前）攒的积分和 prev_measure
             * 如果带进来，第 2 拍就会算出一个巨大的假微分。 */
            Pid_Reset(&s_pid_angle);
            Pid_Reset(&s_pid_steer);
            s_ramp_start_ms = now;
            s_state = BAL_STATE_RUNNING;
        }
        s_tele.angle_out = 0.0f;
        s_tele.steer_out = 0.0f;
        s_tele.ramp_pct  = 0U;
        break;

    case BAL_STATE_RUNNING:
        /* 倾倒保护：先判死，再算输出。放在 PID 之前，是为了保证"这一拍已经判定倒了"
         * 就不会再输出任何力矩 —— 哪怕只有一拍满占空比，车也会被打翻。 */
        if (bal_absf(pitch) > BAL_TILT_LIMIT_DEG)
        {
            bal_latch_fault(BAL_FAULT_TILT);    /* 里面已停机 + 清零 + 写快照，直接返回 */
            return;
        }

        /* 直立环：设定值恒为 0（"竖直"），D 用陀螺原始角速度，不做差分 → 没有微分噪声放大 */
        s_tele.angle_out = Pid_UpdateWithRate(&s_pid_angle, 0.0f, pitch, rate, BAL_CTRL_DT_S);

        /* 转向环：纯 P。没有转向输入时目标 = BAL_STEER_TARGET_DPS（默认 0） */
        s_tele.steer_out = Pid_Update(&s_pid_steer, BAL_STEER_TARGET_DPS, yaw_rate, BAL_CTRL_DT_S);

        {
            float ramp;
            float total;
            float steer;
            float l;
            float r;

            /* 软启动：把"直立 + 转向"整体从 0 线性放大到 100%。
             * 用无符号减法算 elapsed，32 位毫秒计数回绕也不会算错。 */
            if ((now - s_ramp_start_ms) >= BAL_SOFT_START_MS)
            {
                ramp = 1.0f;
            }
            else
            {
                ramp = (float)(now - s_ramp_start_ms) / (float)BAL_SOFT_START_MS;
            }
            s_tele.ramp_pct = (uint8_t)(ramp * 100.0f);

            total = s_tele.angle_out * ramp;
            steer = s_tele.steer_out * ramp;

            /* 差速：左轮减、右轮加。理由见 balance.h 顶部框图下的推导 ——
             * yaw_rate 为正 = 向左转，纠它就得让左轮更快地"往右顶"。 */
            l = total - steer;
            r = total + steer;

            l = bal_clampf(l, BAL_MAX_DUTY);
            r = bal_clampf(r, BAL_MAX_DUTY);

#if (BAL_MOTOR_RIGHT_INVERT != 0)
            r = -r;
#endif

            /* 四舍五入而不是截断：截断会让 ±0.4 CCR 的误差在零点附近来回跳，
             * 表现为"车停了但轮子一直微微叫"。 */
            out_left  = (int16_t)((l >= 0.0f) ? (l + 0.5f) : (l - 0.5f));
            out_right = (int16_t)((r >= 0.0f) ? (r + 0.5f) : (r - 0.5f));
        }
        break;

    case BAL_STATE_FAULT:
    default:
        /* 锁存态：恒定零输出。这里每拍都重写一次 0，是靠"幂等"抗干扰 ——
         * 万一别处动了引脚，下一拍就会被拉回来。 */
        s_tele.angle_out = 0.0f;
        s_tele.steer_out = 0.0f;
        s_tele.ramp_pct  = 0U;
        out_left  = 0;
        out_right = 0;
        break;
    }

    /* ⑤ 输出（DRY_RUN 下 bal_output 是空函数，只有遥测会变） */
    bal_output(out_left, out_right);

    s_tele.duty_left  = out_left;
    s_tele.duty_right = out_right;
    s_tele.state      = s_state;
    s_tele.fault      = s_fault;
}

/* ---------------- 对外接口：遥测 / 故障 ---------------- */

uint8_t Balance_GetTelemetry(BalanceTelemetry_t *out)
{
    uint32_t primask;

    if ((out == 0) || (s_inited == 0U))
    {
        return 0U;
    }

    /* 整块拷贝必须原子：结构体里有 float 和 int16，中断如果正好在拷到一半时
     * 更新，主循环会拿到"新角度配旧占空比"的混合体 —— 调试时会被这种数据骗死。
     * 关中断只有几十条指令（约 2us），对 200Hz 的节拍毫无影响。 */
    primask = __get_PRIMASK();
    __disable_irq();
    *out = s_tele;
    __set_PRIMASK(primask);

    return 1U;
}

uint8_t Balance_TakeFaultEvent(void)
{
    uint8_t code = s_fault_pending;
    s_fault_pending = BAL_FAULT_NONE;
    return code;
}

const char *Balance_FaultName(uint8_t fault)
{
    switch (fault)
    {
    case BAL_FAULT_NONE: return "NONE";
    case BAL_FAULT_TILT: return "TILT(倾倒 >45deg)";
    case BAL_FAULT_IMU:  return "IMU(MPU6050 连续读失败)";
    default:             return "UNKNOWN";
    }
}

const char *Balance_StateName(uint8_t state)
{
    switch (state)
    {
    case BAL_STATE_INIT:         return "INIT";
    case BAL_STATE_WAIT_UPRIGHT: return "WAIT";
    case BAL_STATE_RUNNING:      return "RUN";
    case BAL_STATE_FAULT:        return "FAULT";
    default:                     return "?";
    }
}

void Balance_ClearFault(void)
{
    if (s_state != BAL_STATE_FAULT)
    {
        return;
    }

    s_fault         = BAL_FAULT_NONE;
    s_fault_pending = BAL_FAULT_NONE;
    s_imu_fail_run  = 0U;
    Pid_Reset(&s_pid_angle);
    Pid_Reset(&s_pid_steer);

    /* 退回 WAIT_UPRIGHT，而不是直接 RUNNING：必须重新满足"扶正"条件才起控。
     * 清故障 ≠ 立刻给力。 */
    s_state = BAL_STATE_WAIT_UPRIGHT;
}
