/**
 * Copyright (c) 2015 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#include <stdlib.h>
#include "stm32f1xx_hal.h"

CAN_HandleTypeDef can =
{
  .Instance       = CAN1,
  .Init.Prescaler = 16,
  .Init.Mode      = CAN_MODE_NORMAL,
  .Init.SJW       = CAN_SJW_1TQ,
  .Init.BS1       = CAN_BS1_1TQ,
  .Init.BS2       = CAN_BS2_1TQ,
  .Init.TTCM      = DISABLE,
  .Init.ABOM      = DISABLE,
  .Init.AWUM      = DISABLE,
  .Init.NART      = DISABLE,
  .Init.RFLM      = DISABLE,
  .Init.TXFP      = DISABLE
};

int
  main( int  argc,
        char *argv[] )
{
  (void)argc, (void)argv; /**< unused */

  if( HAL_Init() == HAL_OK )
  {
    static RCC_OscInitTypeDef const oc =
    {
      .OscillatorType = RCC_OSCILLATORTYPE_HSE,
      .HSEState       = RCC_HSE_ON,
      .HSEPredivValue = RCC_HSE_PREDIV_DIV1,
      .PLL.PLLState   = RCC_PLL_ON,
      .PLL.PLLSource  = RCC_PLLSOURCE_HSE,
      .PLL.PLLMUL     = RCC_PLL_MUL9
    };
    HAL_RCC_OscConfig( &oc );

    static RCC_ClkInitTypeDef const ci =
    {
      .ClockType      = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1,
      .SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK,
      .AHBCLKDivider  = RCC_SYSCLK_DIV1,
      .APB1CLKDivider = RCC_HCLK_DIV2,
      .APB2CLKDivider = RCC_HCLK_DIV1
    };
    HAL_RCC_ClockConfig( &ci, FLASH_LATENCY_2 );

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq()/1000 );
    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    HAL_CAN_Init( &can );

    static GPIO_InitTypeDef gi =
    {
      .Pin    = GPIO_PIN_13,
      .Mode   = GPIO_MODE_OUTPUT_PP,
      .Speed  = GPIO_SPEED_LOW
    };

    __GPIOC_CLK_ENABLE( );
    HAL_GPIO_Init( GPIOC, &gi );

    for(;;)
    {
      HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_RESET );  /**< on  */
      HAL_Delay( 300 );
      HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );    /**< off */
      HAL_Delay( 700 );
    }
  }
  return( EXIT_FAILURE );
}