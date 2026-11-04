#include "bsp/bsp_system.h"
#include "bsp/bsp_uart.h"
#include "bsp/bsp_tim.h"
#include "bsp/bsp_i2c.h"
#include "bsp/bsp_mpu.h"
#include "motor/motor.h"
#include "algorithm/attitude.h"
#include "algorithm/pid.h"
#include "control/balance.h"
#include "error/error_handle.h"

#include <stdio.h>

/* printf 的重定向：newlib 的 printf 最终把字符交给 _write 吐出去，
 * 我们把它接到 USART1，printf 就直接写串口了。
 * ⚠️ 这是阻塞发送（一句几十微秒起），只能用在主循环/初始化阶段，绝不能放进中断 ——
 *    塞进定时器中断会直接毁掉控制周期。 */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    (void)BspUart_Write((const uint8_t *)ptr, (uint16_t)len, 100U);
    return len;
}

/* 初始化结果上报：打印 + 失败即停
 * 为什么不用原来那种 `if (A() && B() && C())` 写法：
 * && 是短路的，前面一步挂掉，后面根本不执行，你只能看到"整体失败"，
 * 分不清是哪一步。逐个上报，串口上一眼看出卡在谁身上。 */
static void init_report(const char *name, HAL_StatusTypeDef status)
{
    printf("[init] %-6s : %s\r\n", name, (status == HAL_OK) ? "OK" : "FAIL");
    if (status != HAL_OK)
    {
        Error_Handler();
    }
}

int main(void)
{
    HAL_Init();                 /* 必须最先：SysTick、NVIC 分组、Flash 预取 */
    SystemClock_Config();       /* 再切到 72MHz */

    /* UART 单独处理，且必须第一个上：后面所有上报都靠它打出来，
     * 它自己要是挂了，连 "FAIL" 都打不出来。 */
    if (BspUart_Init() != HAL_OK)
    {
        Error_Handler();
    }

    printf("\r\n===== boot =====\r\n");

    /* 自检：int 和 float 各打一个。浮点那栏要是空的，是 platformio.ini 里
     * -u _printf_float 没生效（不是你的变量等于 0）。 */
    printf("[check] int=%d float=%.2f\r\n", 123, 3.14f);

    /* 顺序有依赖：
     *   tim 必须在 motor 之前（Motor_Init 只负责使能已配好的 PWM 通道）；
     *   i2c 必须在 mpu 之前（BspMpu_Init 假设 I2C 已就绪）。 */
    init_report("tim",   BspTim_Init());

    /* 等 MPU6050 上电就绪：数据手册要求 VDD 稳定后约 30ms 才能通信。
     * 过早让 I2C 上电，可能在总线还没稳时就把 BUSY 标志锁住。 */
    HAL_Delay(100U);
    init_report("i2c",   BspI2c_Init());
    init_report("mpu",   BspMpu_Init());
    init_report("motor", Motor_Init());

    printf("[init] all done\r\n");

    /* 陀螺静止校零：这一步期间车必须完全静止（第一次最好把车架空），
     * 否则求出来的不是"零偏"，而是当时的运动量，会被永久减掉。 */
    printf("[cal]  keep still ...\r\n");
    if (BspMpu_CalibrateGyro(200U) != HAL_OK)
    {
        printf("[cal]  FAIL\r\n");
        Error_Handler();
    }
    printf("[cal]  done\r\n");

    /* 取一帧作为姿态初值：此刻车静止，这一帧就是"当前真实姿态"。
     * Attitude_Init 拿加速度角定初值，避免滤波器从 0 慢慢爬上去。 */
    MpuData_t init_d = {0};
    if (BspMpu_GetData(&init_d) != HAL_OK)
    {
        printf("[att]  init sample FAIL\r\n");
        Error_Handler();
    }
    Attitude_Init(&init_d);

    /* 控制环初始化：只准备 PID 和状态机，不会动电机，也不会启动定时器 */
    init_report("bal", Balance_Init());

    /* ---------- 把人肉需要核对的东西先摊在串口上 ---------- */
    /* 符号和量级是"上电前"必须确认的两件事：
     * 符号错 → 车会追着自己的倒向加速（正反馈）；量级错 → 一歪就满占空比。
     * 所以下面这几行不是装饰，是 DRY_RUN 阶段的对照表。 */
#if (CAR_DRY_RUN != 0)
    printf("[mode] CAR_DRY_RUN=1  只计算不输出（电机不会动）\r\n");
    printf("[mode] 核对方法：手扶车体前倾/后仰，看 P 的正负是否与倾倒方向相反\r\n");
#else
    printf("[mode] CAR_DRY_RUN=0  真实驱动电机！先把车架空再上电\r\n");
#endif
    printf("[pid ] 直立: Kp=%.2f Ki=%.2f Kd=%.2f -> ±%.0f\r\n",
           (double)BAL_ANGLE_KP, (double)BAL_ANGLE_KI, (double)BAL_ANGLE_KD, (double)BAL_MAX_DUTY);
    printf("[pid ] 转向: Kp=%.2f 目标 %.1f deg/s 限幅 ±%.0f\r\n",
           (double)BAL_STEER_KP, (double)BAL_STEER_TARGET_DPS, (double)BAL_STEER_LIMIT);
    printf("[safe] 倾倒锁存 %.0f deg / 起控 %.0f deg / 软启动 %u ms / IMU 连失 %u 拍\r\n",
           (double)BAL_TILT_LIMIT_DEG, (double)BAL_UPRIGHT_ARM_DEG,
           (unsigned)BAL_SOFT_START_MS, (unsigned)BAL_IMU_FAIL_LIMIT);

    /* ---------- 起控制节拍 ----------
     * ⚠️ 从这一行开始，MPU6050 与 I2C1 就被 200Hz 的中断独占了。
     *    主循环里再出现 BspMpu_GetData / Attitude_Update 就是两条线抢一条 I2C：
     *    中断做 14 字节突发读做到一半，主循环插进去发 START，两边都会拿到错数据，
     *    而且症状是"偶发角度跳变"，极难查。主循环只许读 Balance_GetTelemetry 的快照。 */
    init_report("tick", Balance_StartControl());

    printf("[run ] 200Hz 控制节拍已启动，下面 10Hz 打印遥测\r\n\r\n");

    uint32_t last_print = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* 主循环只做低频遥测 + 状态机的外围（打印故障、给人看）。
         * 100ms 一帧是给人眼看的；控制全在中断里，主循环慢一点不影响控制周期。 */
        if ((now - last_print) >= 100U)
        {
            BalanceTelemetry_t t = {0};
            last_print = now;

            /* 故障事件"取走即清"，所以这里只会打印一次 —— 中断里不能 printf，
             * 故障原因必须由主循环替它说出来。 */
            uint8_t fault = Balance_TakeFaultEvent();
            if (fault != BAL_FAULT_NONE)
            {
                printf("[fault] %s → 输出已锁死（0 占空比），必须断电重启才恢复\r\n",
                       Balance_FaultName(fault));
            }

            if (Balance_GetTelemetry(&t) != 0U)
            {
                /* 列意：P=pitch(度) GY=pitch 角速度 YR=yaw 角速度
                 *       A=直立环输出 S=转向环输出 L/R=最终占空比(带符号)
                 *       RMP=软启动进度 N=控制拍数 OV=丢拍数 IMU=I2C 失败次数 */
                printf("[ctl] %-5s P%7.2f GY%8.2f YR%8.2f | A%7.1f S%7.1f | L%6d R%6d | RMP%3u | N%lu OV%lu IMU%lu\r\n",
                       Balance_StateName(t.state),
                       (double)t.pitch_deg, (double)t.pitch_rate_dps, (double)t.yaw_rate_dps,
                       (double)t.angle_out, (double)t.steer_out,
                       (int)t.duty_left, (int)t.duty_right,
                       (unsigned)t.ramp_pct,
                       (unsigned long)t.cycle_count,
                       (unsigned long)t.overrun_count,
                       (unsigned long)t.imu_fail_count);
            }
            else
            {
                printf("[ctl] telemetry not ready\r\n");
            }
        }

        /* 这一句只是让主循环别空转烧 CPU；它会被中断打断，不影响 5ms 节拍。
         * 别在这里写长延时（比如 500ms）：故障打印的延迟会跟着变长。 */
        HAL_Delay(10U);
    }
}
