/**
  ******************************************************************************
  * @file    stm32f1xx_hal_msp_template.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    31-July-2015
  * @brief   HAL BSP module.
  *          This file template is located in the HAL folder and should be copied
  *          to the user folder.
  *
  @verbatim
 ===============================================================================
                     ##### How to use this driver #####
 ===============================================================================
    [..]
    This file is generated automatically by MicroXplorer and eventually modified
    by the user

  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <debugio.h>
#include <assert.h>
#include "stm32f1xx_hal.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @defgroup HAL_MSP HAL_MSP
  * @brief HAL MSP module.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HAL_MSP_Exported_Functions HAL MSP Exported Functions
  * @{
  */

/**
  * @brief  Initializes the Global MSP.
  * @retval None
  */
void HAL_MspInit(void)
{
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  __HAL_RCC_AFIO_CLK_ENABLE( );
  HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

  RCC_OscInitTypeDef oi =
  {
    .OscillatorType = RCC_OSCILLATORTYPE_HSE,
    .HSEState       = RCC_HSE_ON,
    .HSEPredivValue = RCC_HSE_PREDIV_DIV1,
    .PLL.PLLState   = RCC_PLL_ON,
    .PLL.PLLSource  = RCC_PLLSOURCE_HSE,
    .PLL.PLLMUL     = RCC_PLL_MUL9
  };
  HAL_RCC_OscConfig( &oi );

  RCC_ClkInitTypeDef ci =
  {
    .ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1,
    .SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK,
    .AHBCLKDivider  = RCC_SYSCLK_DIV1,
    .APB1CLKDivider = RCC_HCLK_DIV2,
    .APB2CLKDivider = RCC_HCLK_DIV1
  };
  HAL_RCC_ClockConfig( &ci, FLASH_LATENCY_2 );

  HAL_NVIC_SetPriority( SysTick_IRQn, 15, 0 );
  HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );
  HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq()/1000 );

  __GPIOC_CLK_ENABLE( );
  GPIO_InitTypeDef gi =
  {
    .Pin    = GPIO_PIN_13,
    .Mode   = GPIO_MODE_OUTPUT_PP,
    .Speed  = GPIO_SPEED_LOW
  };
  HAL_GPIO_Init( GPIOC, &gi );          /**< load LED: PC13 */

}

/**
  * @brief  DeInitializes the Global MSP.
  * @retval None
  */
void HAL_MspDeInit(void)
{
  /* NOTE : This function is generated automatically by MicroXplorer and eventually
            modified by the user
   */
}

void
  HAL_CAN_MspInit( CAN_HandleTypeDef *pcan )
{
  assert( pcan->Instance == CAN1 );
  if( pcan->Instance == CAN1 )
  {
    GPIO_InitTypeDef gi;

    __CAN1_CLK_ENABLE( );
    __GPIOB_CLK_ENABLE( );
    __HAL_CAN_DBG_FREEZE( pcan, ENABLE );

    gi.Pin  = GPIO_PIN_8;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init( GPIOB, &gi );          /**< CAN_RX: PB8 */

    gi.Pin   = GPIO_PIN_9;
    gi.Mode  = GPIO_MODE_AF_PP;
    gi.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init( GPIOB, &gi );          /**< CAN_TX: PB9 */

    __HAL_AFIO_REMAP_CAN1_2( );           /**< leave USB pins available */

    HAL_NVIC_SetPriority( USB_HP_CAN1_TX_IRQn, 10, 0 );
    HAL_NVIC_EnableIRQ( USB_HP_CAN1_TX_IRQn );
    HAL_NVIC_SetPriority( USB_LP_CAN1_RX0_IRQn, 10, 0 );
    HAL_NVIC_EnableIRQ( USB_LP_CAN1_RX0_IRQn );
    HAL_NVIC_SetPriority( CAN1_RX1_IRQn, 10, 0 );
    HAL_NVIC_EnableIRQ( CAN1_RX1_IRQn );
    HAL_NVIC_SetPriority( CAN1_SCE_IRQn, 10, 0 );
    HAL_NVIC_EnableIRQ( CAN1_SCE_IRQn );
  }
}

void
  HAL_CAN_MspDeInit( CAN_HandleTypeDef *pcan )
{
  assert( pcan->Instance == CAN1 );
  if( pcan->Instance == CAN1 )
  {
    __CAN1_CLK_DISABLE();
    HAL_GPIO_DeInit( GPIOB, GPIO_PIN_8 | GPIO_PIN_9 );
    HAL_NVIC_DisableIRQ( USB_HP_CAN1_TX_IRQn );
    HAL_NVIC_DisableIRQ( USB_LP_CAN1_RX0_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_RX1_IRQn );
    HAL_NVIC_DisableIRQ( CAN1_SCE_IRQn );
  }
}

#ifdef    USE_FULL_ASSERT
void
  assert_failed( uint8_t *file,
                 uint32_t line )
{
  if( debug_enabled() )
  {
    debug_printf( "HAL ASSERT : [%s], L:%d\n", file, line );
    debug_abort( );
  }
}
#endif // USE_FULL_ASSERT

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
