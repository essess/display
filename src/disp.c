/**
 * @file       disp.c
 * @headerfile disp.h
 * @author     sstasiak
 * @brief      2x16 alpha display interface
 *
 * Copyright (c) 2015 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#include "stm32f1xx_hal.h"
#include "disp.h"

/* --| INTERNAL |--------------------------------------------------------- */

static GPIO_InitTypeDef gi;

/**
 * @internal
 * @brief
 */
static inline void
  _e_hi( void )
{
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_3, GPIO_PIN_SET );
}

/**
 * @internal
 * @brief
 */
static inline void
  _e_lo( void )
{
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_3, GPIO_PIN_RESET );
}

/**
 * @internal
 * @brief
 */
static inline void
  _wr( uint16_t d )
{
  /* a4->a7 - output, _e_lo() on entry */
  gi.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init( GPIOA, &gi );
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_2, GPIO_PIN_RESET ); /**< rw/a2 - 0     */
  HAL_GPIO_WritePin( GPIOA,     gi.Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( GPIOA, d & gi.Pin, GPIO_PIN_SET );
  _e_hi();                                                /**< clkin hi data */
  _e_lo();
  d <<= 4;
  HAL_GPIO_WritePin( GPIOA,     gi.Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( GPIOA, d & gi.Pin, GPIO_PIN_SET );
  _e_hi();                                                /**< clkin lo data */
  _e_lo();
}

/**
 * @internal
 * @brief
 */
static inline uint8_t
  _rd( void )
{
  /* a4->a7 - input, _e_lo() on entry */
  gi.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init( GPIOA, &gi );
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_2, GPIO_PIN_SET );    /* rw/a2 - 1 */
  _e_hi();
  GPIO_PinState ps =
    HAL_GPIO_ReadPin( GPIOA, gi.Pin );        /**< d7->d4 emitted first */
  ps <<= 4;
  _e_lo();
  _e_hi();
  ps |= HAL_GPIO_ReadPin( GPIOA, gi.Pin );
  ps >>= 4;
  _e_lo();
  return (uint8_t)ps;
}

/**
 * @internal
 * @brief
 */
static inline void
  _ir( void )
{
  /* rs/a1 - 0 = ir */
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_RESET );
}

/**
 * @internal
 * @brief
 */
static inline void
  _dr( void )
{
  /* rs/a1 - 1 = dr */
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_1, GPIO_PIN_SET );
}

/**
 * @internal
 * @brief
 */
static inline uint8_t
  _rd_ir( void )
{
  _ir();
  return _rd();
}

/**
 * @internal
 * @brief
 */
static inline uint8_t
  _rd_dr( void )
{
  _dr();
  return _rd();
}

/**
 * @internal
 * @brief
 */
static inline int
  _busy( )
{
  return _rd_ir() & (1<<7);
}

/**
 * @internal
 * @brief
 */
static inline void
  _wr_ir( uint8_t ir )
{
  _ir();
  _wr( ir );
  while( _busy() );
}

/**
 * @internal
 * @brief
 */
static inline void
  _wr_dr( uint8_t dr )
{
  _dr();
  _wr( dr );
}

/**
 * @internal
 * @brief
 */
static inline uint8_t
  _ac( )
{
  return _rd_ir() & ~(1<<7);
}

/* --| PUBLIC   |--------------------------------------------------------- */
/**
 * @public
 * @brief initialize display
 */
void
  disp_init( void )
{
  /*
    rs - a1
    rw - a2
     e - a3

    4bit i/o (leave floating initially):
      d0 - x    d4 - a4
      d1 - x    d5 - a5
      d2 - x    d6 - a6
      d3 - 1    d7 - a7
  */

  __GPIOA_CLK_ENABLE( );
  gi.Pin    = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
              GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gi.Mode   = GPIO_MODE_OUTPUT_PP;
  gi.Speed  = GPIO_SPEED_MEDIUM;
  HAL_GPIO_WritePin( GPIOA, gi.Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET );
  HAL_GPIO_Init( GPIOA, &gi );
  /* preselect pin settings for data i/o */
  gi.Pin  = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gi.Pull = GPIO_PULLUP;
  HAL_Delay( 50 );
  _e_hi();
  _e_lo();
  HAL_Delay( 6 );
  _e_hi();
  _e_lo();
  HAL_Delay( 2 );
  _e_hi();
  _e_lo();
  HAL_Delay( 2 );
  HAL_GPIO_WritePin( GPIOA, GPIO_PIN_4, GPIO_PIN_RESET );
  _e_hi();
  _e_lo();          /**< finalize 4bit mode         */
  _wr_ir( 0x28 );   /**< 2line 5x8 font             */
  _wr_ir( 0x0c );
  _wr_ir( 0x06 );
  _wr_ir( 0x01 );   /**< clear display              */
}

/**
 * @public
 * @brief cycle to next displayed value
 */
void
  disp_next( void )
{
  // test code junk
  _wr_dr('*');
}