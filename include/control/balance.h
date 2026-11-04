#ifndef __CONTROL_BALANCE_H__
#define __CONTROL_BALANCE_H__

#include "stm32f1xx_hal.h"
#include "bsp/bsp_tim.h"        /* BSP_TIM_PWM_PERIOD：占空比的满量程 */

/* ============================================================================
 * 直立 + 转向控制环（200Hz，由 TIM4 中断驱动）
 *
 * 结构（并联，两环的输出直接在占空比上叠加）：
 *
 *   pitch ──► [角度环 PD]  ──┐                 ┌─► L = Θ - Ψ ─┐
 *   gyro_y ─► (陀螺阻尼)     ├─► Θ (直立分量)   │              ├─► L298N
 *   yaw_rate ─► [转向环 P] ──┴─► Ψ (差速分量)   └─► R = Θ + Ψ ─┘
 *
 *   为什么直立用 PD 不是 PID：直立环的"稳态误差"在物理上等于"车歪着走"，
 *   一旦积分上去，车会为了消掉那点角度一路加速冲出去（没有编码器就收不回来）。
 *   Ki 默认给 0，别乱加，理由见 README 的"已知限制"。
 *
 *   为什么转向要减不要加：yaw_rate 为正 = 车在逆时针（左）转，
 *   要纠回来必须让左轮比右轮快（右转）。所以 L = Θ - Ψ、R = Θ + Ψ。
 *   写成加号会变成正反馈，车会自己越转越快——这个错上电就炸，别试。
 * ========================================================================== */

/* ============================ 总开关 ============================
 * CAR_DRY_RUN = 1（默认）：只把算出来的占空比打到串口，一根线都不驱动电机。
 *   理由：这套参数没有实测过，符号（往前倒该往前冲还是往后冲）和量级
 *   （1° 误差对应多少占空比）都必须在串口上先核对。直接上电 = 大概率原地
 *   满速打转或者窜出去撞墙。
 * CAR_DRY_RUN = 0：真正驱动电机。切换步骤见 README「DRY_RUN → 上板」。
 *
 * 两种改法（任选其一）：
 *   ① 直接把下面这个 1 改成 0；
 *   ② 留在代码里不动，在 platformio.ini 的 build_flags 里加 -DCAR_DRY_RUN=0
 *      （命令行临时验证更方便：pio run -t upload 前改一行，不影响源码）。
 */
#ifndef CAR_DRY_RUN
#define CAR_DRY_RUN                 1
#endif

/* ============================ 直立环参数 ============================
 * 输出单位是 CCR（0..3600）；输入单位是 度 和 度/秒。
 * 所以 Kp 的物理含义是"每偏 1 度，给多少 CCR"，Kd 是"每 1°/s 的角速度，给多少 CCR"。
 * 整定顺序见 README：先只有 Kp，再加 Kd，最后才谈 Ki（一般不需要）。 */
#define BAL_ANGLE_KP                (45.0f)     /* 起步值：1° → 45 CCR（满量程 3600 的 1.25%） */
#define BAL_ANGLE_KI                (0.0f)      /* 默认 0：加了就是"歪着加速"，没有编码器收不住 */
#define BAL_ANGLE_KD                (1.5f)      /* 1°/s 的角速度 → 1.5 CCR 的反向阻尼 */

/* 直立环输出的最大幅值 = PWM 满量程。超过它没有物理意义（CCR 只能到 ARR） */
#define BAL_MAX_DUTY                ((float)BSP_TIM_PWM_PERIOD)     /* 3600 */

/* ============================ 转向环参数 ============================ */
#define BAL_STEER_KP                (8.0f)      /* 1°/s 的偏航角速度 → 8 CCR 的差速 */
#define BAL_STEER_LIMIT             (600.0f)    /* 转向最多只许占 ±600/3600 ≈ 17% 的差速 */

/* 转向目标偏航角速度（°/s）。没有遥控输入时给 0 = "保持不转"。
 * 调试时改成 ±30 之类的小值，可以在 DRY_RUN 下看清楚差速方向对不对；
 * 千万不要在上电瞬间给大值（比如 90），那是一次原地甩尾。 */
#define BAL_STEER_TARGET_DPS        (0.0f)

/* ============================ 安全逻辑 ============================ */
/* 倾倒角：|pitch| 超过它就判定"车已经倒了"，立即停机并锁存。
 * 45° 是个折中：太小 → 正常晃动就误停；太大 → 车已经拍到地上才开始停。 */
#define BAL_TILT_LIMIT_DEG          (45.0f)

/* 起控角：上电后只有 |pitch| 落在这个范围内才允许进入 RUNNING。
 * 没有这一条的话，车侧躺在地上也能起控 → 直接一个满占空比。
 * 10° 意味着"手扶着、大致竖直"才算准备好；比 8° 宽一点是为了台面上
 * 用手扶着做 DRY_RUN 核对时，手抖不会老是把它卡在 WAIT 状态。 */
#define BAL_UPRIGHT_ARM_DEG         (10.0f)

/* 软启动时长（ms）：把最终占空比从 0 线性放大到 100%。
 * 防止"刚一使能就是满占空比"把车弹出去。1.2s 内爬起来，手感上不会太迟钝。 */
#define BAL_SOFT_START_MS           (1200U)

/* 连续多少拍 I2C 读失败就判 IMU 故障停机。
 * 偶发单拍失败很常见（总线被电机干扰），不能一拍就停；但连 5 拍（25ms）都没数据，
 * 等于闭着眼睛输出，必须停。 */
#define BAL_IMU_FAIL_LIMIT          (5U)

/* ============================ 符号开关（先别改） ============================
 * 上板发现"越往前倒反而往后冲"，说明这套信号链的符号约定和你手上的接线相反。
 * 正确处理是改这里的 ±1，而不是去改接线或改 PID 的符号 —— 改这里只有一个改动点，
 * 改接线则会同时影响串口打印出来的 pitch 正负，以后看日志会一直拧巴。
 * 翻转方式：把 +1.0f 改成 -1.0f，重新编译。
 * ⚠️ 注意 BAL_PITCH_DIR 同时作用于 pitch 和 gyro_y，两者必须一起翻，
 *    否则 P 和 D 会互相打架（表现为高频抖动）。 */
#define BAL_PITCH_DIR               (1.0f)
#define BAL_YAW_DIR                 (1.0f)

/* 右电机机械反装（差速方向反了：应该原地右转却原地左转）。
 * L298N 的 OUT1/OUT2 接反、或者左右减速箱镜像，都会导致这个现象。
 * 改这里的 1 比重新焊线快，也不会影响别的逻辑。 */
#define BAL_MOTOR_RIGHT_INVERT      (0)

/* ============================ 数据类型 ============================ */

typedef enum
{
    BAL_STATE_INIT = 0,         /* 还没开始跑控制节拍 */
    BAL_STATE_WAIT_UPRIGHT,     /* 等车被扶正（|pitch| < BAL_UPRIGHT_ARM_DEG） */
    BAL_STATE_RUNNING,          /* 闭环运行中 */
    BAL_STATE_FAULT             /* 已锁存故障：输出恒 0，只能复位/重新上电 */
} BalanceState_t;

typedef enum
{
    BAL_FAULT_NONE = 0,
    BAL_FAULT_TILT = 1,         /* |pitch| > 45°：车倒了 */
    BAL_FAULT_IMU  = 2          /* 连续多拍读不到 MPU6050 */
} BalanceFault_t;

/* 遥测快照：中断每拍更新，主循环整块拷走。
 * 主循环绝对不要直接读 MPU / 直接调 Attitude_Update —— I2C1 是共享资源，
 * 中断在做 14 字节突发读，主循环插进去读一半，两边都会拿到错数据。 */
typedef struct
{
    float    pitch_deg;         /* 当前 pitch（已乘 BAL_PITCH_DIR），单位度 */
    float    pitch_rate_dps;    /* 参与阻尼的角速度（gyro_y，已乘符号），单位°/s */
    float    yaw_rate_dps;      /* 参与转向的偏航角速度（gyro_z），单位°/s */
    float    angle_out;         /* 直立环输出（未乘软启动斜坡），单位 CCR */
    float    steer_out;         /* 转向环输出（未乘软启动斜坡），单位 CCR */
    int16_t  duty_left;         /* 最终左轮占空比，-3600..+3600（DRY_RUN 下是真值但不输出） */
    int16_t  duty_right;        /* 最终右轮占空比 */
    uint8_t  state;             /* BalanceState_t */
    uint8_t  fault;             /* BalanceFault_t */
    uint8_t  ramp_pct;          /* 软启动进度 0..100 */
    uint32_t cycle_count;       /* 已执行的控制拍数（200Hz → 每秒 +200） */
    uint32_t overrun_count;     /* 丢拍计数：某拍距上一拍 > 8ms 就 +1 */
    uint32_t imu_fail_count;    /* 累计 I2C 读失败次数 */
} BalanceTelemetry_t;

/* ============================ 对外接口 ============================ */

/* 只做初始化（PID 复位、状态机置 WAIT_UPRIGHT、清遥测），不会启动定时器。
 * 必须放在 BspMpu_Init / BspMpu_CalibrateGyro / Attitude_Init 之后：
 * 它假设 MPU 已经能出数、姿态滤波已经有初值。 */
HAL_StatusTypeDef Balance_Init(void);

/* 启动 200Hz 控制节拍（内部会配 TIM4 并把 Balance_Tick 挂到中断上）。
 * ⚠️ 调用之后，MPU6050 就被中断独占了，主循环里不许再出现 BspMpu_GetData。 */
HAL_StatusTypeDef Balance_StartControl(void);

/* 停止控制节拍，并把输出清零（软件停机用；FAULT 锁存走的是另一条路） */
void Balance_Stop(void);

/* 控制拍本体，只允许由 TIM4 中断调用。
 * 里面做：采样 → 融合 → 状态机/PID → 写 CCR。
 * 绝对不要在这个函数里加 printf —— 一句 115200 的浮点打印要 ~100us 起，
 * 200Hz 的周期只有 5000us，多打几句就把节拍直接撑爆。 */
void Balance_Tick(void);

/* 取一份一致的遥测快照（内部短暂关中断，几十字节拷贝，约 2us）。
 * 返回 1 = 有效；0 = 还没初始化。 */
uint8_t Balance_GetTelemetry(BalanceTelemetry_t *out);

/* 取一次性的故障事件：有未上报的故障返回故障码并清标志，否则返回 BAL_FAULT_NONE。
 * 设计成"取走即清"是为了让主循环 printf 恰好打一次 —— 中断里不能打印，
 * 故障又必须让用户看到原因，只能这样交接。 */
uint8_t Balance_TakeFaultEvent(void);

/* 故障名（给串口打印用），未知码返回 "UNKNOWN" */
const char *Balance_FaultName(uint8_t fault);

/* 状态名，未知返回 "?" */
const char *Balance_StateName(uint8_t state);

/* 手动清故障：只清锁存，不会自动起控（还要重新满足"扶正"条件）。
 * main 里没有任何地方自动调它 —— 这是故意的：车倒了以后自己爬起来重新满速，
 * 比停下来更危险。要恢复就断电重启，或者接一个按键/串口命令明确地调它。 */
void Balance_ClearFault(void);

#endif /* __CONTROL_BALANCE_H__ */
