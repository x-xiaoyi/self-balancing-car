#include "bsp/bsp_system.h"
#include "bsp/bsp_uart.h"
#include "bsp/bsp_tim.h"
#include "bsp/bsp_i2c.h"
#include "bsp/bsp_mpu.h"
#include "motor/motor.h"
#include "algorithm/attitude.h"
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

    Attitude_t att = {0};
    uint32_t last_tick  = HAL_GetTick();
    uint32_t sample_cnt = 0U;

    /* 主循环：读一帧 → 融合 → 打印一帧。
     * 100ms 一帧（10Hz）只是为了肉眼观察；平衡车真正跑控制时，
     * 这里要换成定时器节拍，不能靠 HAL_Delay 硬拖。 */
    while (1)
    {
        MpuData_t d = {0};

        if (BspMpu_GetData(&d) == HAL_OK)
        {
            uint32_t now = HAL_GetTick();
            sample_cnt++;

            /* 第一帧没有"上一帧"，dt 无从谈起 → 直接丢弃：不融合、不打印。
             * 从第 2 帧起 dt 才是真实间隔，这样也就不需要再编一个假的占位值。 */
            if (sample_cnt >= 2U)
            {
                float dt_s = (float)(now - last_tick) / 1000.0f;   /* ms → s，量纲才配得上 °/s */

                Attitude_Update(&d, dt_s, &att);

                printf("A %6.3f %6.3f %6.3f | G %7.2f %7.2f %7.2f | T%5.2f | R%7.2f P%7.2f | DT%4.0f\r\n",
                       d.accel[0], d.accel[1], d.accel[2],
                       d.gyro[0],  d.gyro[1],  d.gyro[2],
                       d.temp,
                       att.roll_reg, att.pitch_reg,
                       dt_s * 1000.0f);
            }

            last_tick = now;
        }
        else
        {
            printf("[loop] mpu read FAIL\r\n");
        }

        HAL_Delay(100U);
    }
}
