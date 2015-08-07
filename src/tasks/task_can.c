/**
 * @file       task_can.c
 * @headerfile task_can.h
 * @author     sstasiak
 * @brief      can task(s)
 *
 * Copyright (c) 2015 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "task/task_can.h"
#include <debugio.h>
#include <assert.h>

/* --| INTERNAL |--------------------------------------------------------- */

typedef struct
{
  struct
  {
    uint32_t pkts;
  } rx;
  struct
  {
    uint32_t pkts;
  } tx;
} can_cntrs_t;

static can_cntrs_t cc;

static void
  task_can_rx( void *const parg );

static CanRxMsgTypeDef rx_msg;
static CanTxMsgTypeDef tx_msg =
{
  .StdId = 0x5f0,
  .IDE = CAN_ID_STD,
  .RTR = CAN_RTR_DATA,
  .DLC = 0
};

/**
 * @internal
 * @brief CAN adt for HAL - 1MBit timing
 */
static CAN_HandleTypeDef can =
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
  .Init.TXFP      = DISABLE,

  .pTxMsg = &tx_msg,
  .pRxMsg = &rx_msg
};

/**
* @brief This function handles USB high priority or CAN TX interrupts.
*/
void
  USB_HP_CAN1_TX_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &can );
}

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void
  USB_LP_CAN1_RX0_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &can );
}

/**
* @brief This function handles CAN RX1 interrupt.
*/
void
  CAN1_RX1_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &can );
}

/**
* @brief This function handles CAN SCE interrupt.
*/
void
  CAN1_SCE_IRQHandler( void )
{
  HAL_CAN_IRQHandler( &can );
}

/* --| PUBLIC   |--------------------------------------------------------- */
/**
 * @public
 * @brief create can receive task
 * @retval int non-zero on error
 */
int
  create_task_can_rx( void )
{
  TaskHandle_t t = 0;
  BaseType_t const stat =
    xTaskCreate( &task_can_rx,
                 "task_can_rx",
                 TASK_CAN_RX_STKSIZE,
                 &can,
                 TASK_CAN_RX_PRIO,
                 &t );
  assert( stat == pdPASS );
  assert( t );

  return ((stat == pdPASS) && t) ? 0 : ~0 ;
}

static void
  task_can_rx( void *const parg )
{
  CAN_HandleTypeDef *const pcan = parg;
  assert( pcan == &can );

  HAL_StatusTypeDef hs = HAL_CAN_Init( pcan );
  assert( hs == HAL_OK );

  static CAN_FilterConfTypeDef fc =
  {
    .FilterIdHigh = 0x0600<<5,       /**< ALL low pri, high vol pkts allowed */
    .FilterIdLow = 0x0600<<5,
    .FilterMaskIdHigh = 0x0600<<5,   /**< 1100.0000.0000b - ignore rtr/ide/etc */
    .FilterMaskIdLow = 0x0600<<5,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterNumber = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterActivation = ENABLE,
    .BankNumber = 0 /**< always 0 */
  };
  hs = HAL_CAN_ConfigFilter( pcan, &fc );
  assert( hs == HAL_OK );

  /* TODO: grab first pkt buffer from pool and
           hook before turning on int's */
  assert( pcan->pRxMsg == &rx_msg );

  hs = HAL_CAN_Receive_IT( pcan, CAN_FIFO0 );
  assert( hs == HAL_OK );

  for( ;; )
  {
    assert( pcan == &can );
    assert( pcan->pRxMsg );
    assert( pcan->pTxMsg );

    debug_printf( "cc.rx.pkts: %d\r\n", cc.rx.pkts );
    debug_printf( "   tx.pkts: %d\r\n", cc.tx.pkts );
    vTaskDelay( 500 );

    hs = HAL_CAN_Transmit_IT( pcan );
    assert( hs == HAL_OK );
  }
}

void  /* FROM IRQ CONTEXT */
  HAL_CAN_TxCpltCallback( CAN_HandleTypeDef *const pcan )
{
  assert( pcan == &can );
  assert( pcan->pTxMsg == &tx_msg );

  cc.tx.pkts++;
  pcan->pTxMsg->StdId++;
  if( pcan->pTxMsg->StdId > 0x7ff )
    pcan->pTxMsg->StdId = 0;
}

void  /* FROM IRQ CONTEXT */
  HAL_CAN_RxCpltCallback( CAN_HandleTypeDef *const pcan )
{
  assert( pcan == &can );
  cc.rx.pkts++;

  assert( pcan->pRxMsg == &rx_msg );
  HAL_StatusTypeDef const hs = HAL_CAN_Receive_IT( pcan, pcan->pRxMsg->FIFONumber );
  assert( hs == HAL_OK );
}

void  /* FROM IRQ CONTEXT */
  HAL_CAN_ErrorCallback( CAN_HandleTypeDef *const pcan )
{
  assert( pcan == &can );
  static uint32_t cnt = 0;

  uint32_t const err = HAL_CAN_GetError(pcan);
  debug_printf( "[%d] HAL_CAN_ErrorCallback( ):\r\n", ++cnt );
  if( err == HAL_CAN_ERROR_NONE )
    debug_runtime_error( "     HAL_CAN_ERROR_NONE(!)" );
  else
  {
    if( err & HAL_CAN_ERROR_EWG )
      debug_puts( "     HAL_CAN_ERROR_EWG" );
    if( err & HAL_CAN_ERROR_EPV )
      debug_puts( "     HAL_CAN_ERROR_EPV" );
    if( err & HAL_CAN_ERROR_BOF )
      debug_puts( "     HAL_CAN_ERROR_BOF" );
    if( err & HAL_CAN_ERROR_STF )
      debug_puts( "     HAL_CAN_ERROR_STF" );
    if( err & HAL_CAN_ERROR_FOR )
      debug_puts( "     HAL_CAN_ERROR_FOR" );
    if( err & HAL_CAN_ERROR_ACK )
      debug_puts( "     HAL_CAN_ERROR_ACK" );
    if( err & HAL_CAN_ERROR_BR )
      debug_puts( "     HAL_CAN_ERROR_BR" );
    if( err & HAL_CAN_ERROR_BD )
      debug_puts( "     HAL_CAN_ERROR_BD" );
    if( err & HAL_CAN_ERROR_CRC )
      debug_puts( "     HAL_CAN_ERROR_CRC" );

    __HAL_CAN_CLEAR_FLAG( pcan, CAN_FLAG_ERRI );
  }
}