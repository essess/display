/**
 * Copyright (c) 2015 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#include <assert.h>
#include <stdlib.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

CAN_HandleTypeDef can =
{
  .Instance       = CAN1,
  .Init.Prescaler = 3,
  .Init.Mode      = CAN_MODE_NORMAL,
  .Init.SJW       = CAN_SJW_4TQ,
  .Init.BS1       = CAN_BS1_5TQ,
  .Init.BS2       = CAN_BS2_6TQ,
  .Init.TTCM      = DISABLE,
  .Init.ABOM      = DISABLE,
  .Init.AWUM      = DISABLE,
  .Init.NART      = DISABLE,
  .Init.RFLM      = DISABLE,
  .Init.TXFP      = DISABLE
};

static void
  blinky( void *parg )
{
  (void)parg;           /**< unused */

  GPIO_InitTypeDef gi =
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
    vTaskDelay( 300 );
    HAL_GPIO_WritePin( GPIOC, GPIO_PIN_13, GPIO_PIN_SET );    /**< off */
    vTaskDelay( 700 );
  }
}

int
  main( int  argc,
        char *argv[] )
{
  (void)argc, (void)argv; /**< unused */

  HAL_StatusTypeDef hs = HAL_Init( ); assert( hs == HAL_OK );
  if( hs == HAL_OK )
  {
    hs = HAL_CAN_Init( &can ); assert( hs == HAL_OK );
    if( hs == HAL_OK )
    {
      TaskHandle_t t = 0;
      BaseType_t const ts =
        xTaskCreate( &blinky,
                     "blinky",
                     512,
                     0,
                     tskIDLE_PRIORITY+1,
                     &t );
      assert( ts == pdPASS ); (void)sizeof(ts);
      assert( t != 0 );
      vTaskStartScheduler( );
      hs = HAL_CAN_DeInit( &can ); assert( hs == HAL_OK );
    }
    hs = HAL_DeInit( ); assert( hs == HAL_OK );
  }
  return EXIT_FAILURE;
}