#include "bsp/bsp_system.h"
#include "bsp/bsp_uart.h"
#include "bsp/bsp_tim.h"
#include "bsp/bsp_i2c.h"
#include "motor/motor.h"
#include "error/error_handle.h"

#include <stdio.h>

/* 测试速度：2700 / 3600 = 75% 占空比 */
#define MOTOR_TEST_SPEED    2700U

/* printf 的重定向：newlib 的 printf 最终会把字符交给 _write 吐出去，
 * 我们把它接到 USART1，printf 就直接写串口了。
 * ⚠️ 这是阻塞发送（一句几十微秒起），只能用在主循环里，绝不能放进中断 ——
 *    塞进定时器中断会直接毁掉控制周期。中断里最多置个标志位，打印留给主循环。 */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    (void)BspUart_Write((const uint8_t *)ptr, (uint16_t)len, 100U);
    return len;
}

int main(void)
{
    HAL_Init();                 /* 必须最先：SysTick、NVIC 分组、Flash 预取 */
    SystemClock_Config();       /* 再切到 72MHz */

    /* 注意 && 是短路的：前面一个失败，后面的初始化根本不会执行。
     * 顺序有依赖：tim 必须在 motor 之前（Motor_Init 要用已经配好的 PWM 通道）。 */
    if (BspUart_Init() == HAL_OK &&
        BspTim_Init()  == HAL_OK &&
        BspI2c_Init()  == HAL_OK &&
        Motor_Init()   == HAL_OK)
    {
        printf("Init_ok\r\n");

        /* 自检：int 和 float 各打一个，浮点那栏要是空的，就是 platformio.ini 里
         * -u _printf_float 没生效（不是你的变量等于 0） */
        printf("[check] int=%d  float=%.2f\r\n", 123, 3.14f);

        HAL_Delay(2000U);       /* 留 2 秒：看清打印、把车放稳（第一次一定要架空！） */

        Motor_Forward(MOTOR_TEST_SPEED);    /* 方向 + 占空比一次给完 */
        printf("motor forward, speed=%u\r\n", (unsigned)MOTOR_TEST_SPEED);
    }
    else
    {
        /* 如果是 UART 自己失败，这句也打不出来，那就得靠调试器看是哪一步过的 */
        printf("Init_fail\r\n");
        Error_Handler();
    }

    while (1)
    {
    }
}
