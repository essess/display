/**
 * @file   disp.h
 * @author sstasiak
 * @brief  2x16 alpha display interface
 *
 * Copyright (c) 2015 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#ifndef   __disp_h
#define   __disp_h

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @public
 * @brief
 */
typedef enum
{
  SCRN0 = 0,
  SCRN1,
  SCRN2
} disp_idx_t;

/**
 * @public
 * @brief initialize display
 */
void
  disp_init( void );

/**
 * @public
 * @brief cycle to next displayed value
 */
void
  disp_next( void );

/**
 * @public
 * @brief cycle to next displayed value
 * @param[in] i screen index
 * @param[in] pline0 first line
 * @param[in] pline1 second line
 */
void
  disp( disp_idx_t i,
        char *pline0,
        char *pline1 );

#ifdef __cplusplus
}
#endif

#endif // __disp_h