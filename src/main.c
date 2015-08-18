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

#include "task/task_can.h"
#include "disp.h"

extern void xPortSysTickHandler( void );

#define TASK_BUTTON_PRIO      ( 1 )
#define TASK_BUTTON_STKSIZE   ( 256 )

static void
  task_button( void *parg )
{
  (void)parg;           /**< unused */

  vTaskDelay( 250 );
  for(;;)
  {
    if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET )
    {
      vTaskDelay( 30 );   /**< debounce */
      if( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET )
      {
        disp_next();
        vTaskDelay( 400 );  /**< repeat lockout */
      }
    }
    vTaskDelay( 50 );
  }
}

static int
  create_task_button( void )
{
  TaskHandle_t t = 0;
  BaseType_t const stat =
    xTaskCreate( &task_button,
                 "task_button",
                 TASK_BUTTON_STKSIZE,
                 0,
                 TASK_BUTTON_PRIO,
                 &t );
  assert( stat == pdPASS );
  assert( t );

  return ((stat == pdPASS) && t) ? 0 : ~0 ;
}

int
  main( int  argc,
        char *argv[] )
{
  (void)argc, (void)argv; /**< unused */

  HAL_StatusTypeDef hs = HAL_Init( );
  assert( hs == HAL_OK );

  if( hs == HAL_OK )
  {
    int stat = 0;

    disp_init( );

    stat |= create_task_button( );
    assert( stat == 0 );

    stat |= create_task_can_rx( );
    assert( stat == 0 );

    if( stat == 0 )
      vTaskStartScheduler( );
    hs = HAL_DeInit( );
    assert( hs == HAL_OK );
  }

  return EXIT_FAILURE;
}

/**
* @brief systick handler
*/
void
  SysTick_Handler( void )
{
  HAL_IncTick( );
  if ( xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED )
    xPortSysTickHandler( );
}