#include "bsp/bsp_system.h"
#include "error/error_handle.h"

/*
 * 时钟树配置：HSE 8MHz × 9 = 72MHz（F103 用 8MHz 外部晶振 + PLL×9）
 * APB1 预分频 = 2（36MHz），APB2 预分频 = 1（72MHz）
 * APB1 预分频 ≠ 1 时，TIM2~7 的时钟 = APB1 × 2 = 72MHz
 * SysTick 定时器驱动 HAL_Delay，走 HCLK/8 = 9MHz
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 使能 HSE（外部高速晶振 8MHz） */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;    /* 8MHz × 9 = 72MHz */
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* AHB = SYSCLK/1 = 72MHz, APB1 = HCLK/2 = 36MHz, APB2 = HCLK/1 = 72MHz */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    /* Flash 等待周期必须 ≥ 2（72MHz 对应 2 个等待周期） */
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* SysTick 每 1ms 进一次，给 HAL_GetTick 计数，HAL_Delay 依赖它 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}
