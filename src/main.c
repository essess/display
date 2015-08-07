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

extern void xPortSysTickHandler( void );

#define TASK_BURN_CYCLES_PRIO     ( 1 )
#define TASK_BURN_CYCLES_STKSIZE  ( 1024 )

static void
  burn_cycles( void *parg )
{
  (void)parg;           /**< unused */

  /* watch load indicator led while performing the following: */
  for(;;)
  {
    /* dominate core for a few ms ... */
    HAL_Delay( 300 );
    /* be nice for a few ms ...       */
    vTaskDelay( 700 );
  }
}

static int
  create_task_burn_cycles( void )
{
  TaskHandle_t t = 0;
  BaseType_t const stat =
    xTaskCreate( &burn_cycles,
                 "burn_cycles",
                 TASK_BURN_CYCLES_STKSIZE,
                 0,
                 TASK_BURN_CYCLES_PRIO,
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

    stat |= create_task_burn_cycles( );
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