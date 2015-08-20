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
#include "semphr.h"
#include "stm32f1xx_hal.h"
#include "task/task_can.h"
#include "disp.h"
#include <assert.h>

/* --| INTERNAL |--------------------------------------------------------- */

typedef struct
{
  struct
  {
    uint32_t rx;
    uint32_t tx;
  } pkt;
  struct
  {
    uint32_t sce;
    uint32_t tx;
    uint32_t rx0;
    uint32_t rx1;
  } irq;
  struct
  {
    uint32_t none;
    uint32_t ewg;
    uint32_t epv;
    uint32_t bof;
    uint32_t stf;
    uint32_t form;
    uint32_t ack;
    uint32_t br;
    uint32_t bd;
    uint32_t crc;
  } err;
  uint32_t unh_id;
} cnt_t;

static cnt_t cnt;

static SemaphoreHandle_t sh;

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
  .Init.NART      = DISABLE, /**< always disabled until I can figure out IRQ bug (self recoverable this way) */
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
  cnt.irq.tx++;
  HAL_CAN_IRQHandler( &can );
}

/**
* @brief This function handles USB low priority or CAN RX0 interrupts.
*/
void
  USB_LP_CAN1_RX0_IRQHandler( void )
{
  cnt.irq.rx0++;
  HAL_CAN_IRQHandler( &can );
}

/**
* @brief This function handles CAN RX1 interrupt.
*/
void
  CAN1_RX1_IRQHandler( void )
{
  cnt.irq.rx1++;
  HAL_CAN_IRQHandler( &can );
}

/**
* @brief This function handles CAN SCE interrupt.
*/
void
  CAN1_SCE_IRQHandler( void )
{
  cnt.irq.sce++;
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
  sh = xSemaphoreCreateBinary();
  assert( sh );

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
  assert( hs == HAL_OK ); (void)sizeof(hs);

  static CAN_FilterConfTypeDef fc =
  {
    .FilterIdHigh = 0x0000<<5,
    .FilterIdLow = 0x0000<<5,
    .FilterMaskIdHigh = 0x0000<<5,   /**< let 'em all through */
    .FilterMaskIdLow = 0x0000<<5,
    .FilterFIFOAssignment = CAN_FILTER_FIFO0,
    .FilterNumber = 0,
    .FilterMode = CAN_FILTERMODE_IDMASK,
    .FilterScale = CAN_FILTERSCALE_16BIT,
    .FilterActivation = ENABLE,
    .BankNumber = 0 /**< always 0 */
  };
  hs = HAL_CAN_ConfigFilter( pcan, &fc );
  assert( hs == HAL_OK );

  assert( pcan->pRxMsg == &rx_msg );

  for( ;; )
  {
    assert( pcan == &can );
    assert( pcan->pRxMsg );
    assert( pcan->pTxMsg );

    hs = HAL_CAN_Receive_IT( pcan, CAN_FIFO0 );
    assert( hs == HAL_OK );
    if( xSemaphoreTake(sh, portMAX_DELAY) == pdTRUE )
    {
      assert( pcan->pRxMsg->FIFONumber == CAN_FIFO0 );
      if( (pcan->pRxMsg->IDE == CAN_ID_STD) &&
          (pcan->pRxMsg->RTR == CAN_RTR_DATA) )
      {
        static char lb[16];     /**< line buffer */
        switch( pcan->pRxMsg->StdId )
        {
          case 0x000:
            disp( SCRN0, "SCRN0           ", "Time            ");
        //    break;
          case 0x101:
            disp( SCRN1, "SCRN1           ", "0x101           ");
        //    break;
          case 0x102:
            disp( SCRN2, "SCRN2           ", "0x102           ");
            break;
          default:
            cnt.unh_id++;
        }
      }
    }
  }
}

void  /* FROM IRQ CONTEXT */
  HAL_CAN_TxCpltCallback( CAN_HandleTypeDef *const pcan )
{
  assert( pcan == &can );
  assert( pcan->pTxMsg == &tx_msg );
  cnt.pkt.tx++;
}

void  /* FROM IRQ CONTEXT */
  HAL_CAN_RxCpltCallback( CAN_HandleTypeDef *const pcan )
{
  assert( pcan == &can );
  assert( pcan->pRxMsg == &rx_msg );
  cnt.pkt.rx++;
  xSemaphoreGiveFromISR( sh, 0 );
}

void  /* FROM IRQ CONTEXT */
  HAL_CAN_ErrorCallback( CAN_HandleTypeDef *const pcan )
{
  assert( pcan == &can );

  uint32_t const err = HAL_CAN_GetError(pcan);
  if( err == HAL_CAN_ERROR_NONE )
    cnt.err.none++;
  else
  {
    if( err & HAL_CAN_ERROR_EWG )
      cnt.err.ewg++;
    if( err & HAL_CAN_ERROR_EPV )
      cnt.err.epv++;
    if( err & HAL_CAN_ERROR_BOF )
      cnt.err.bof++;
    if( err & HAL_CAN_ERROR_STF )
      cnt.err.stf++;
    if( err & HAL_CAN_ERROR_FOR )
      cnt.err.form++;
    if( err & HAL_CAN_ERROR_ACK )
      cnt.err.ack++;
    if( err & HAL_CAN_ERROR_BR )
      cnt.err.br++;
    if( err & HAL_CAN_ERROR_BD )
      cnt.err.bd++;
    if( err & HAL_CAN_ERROR_CRC )
      cnt.err.crc++;

    __HAL_CAN_CLEAR_FLAG( pcan, CAN_FLAG_ERRI );
  }
}