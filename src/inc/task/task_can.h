/**
 * @file   task_can.h
 * @author sstasiak
 * @brief  header for can task(s) information
 *
 * Copyright (c) 2015 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#ifndef   __task_can_h
#define   __task_can_h

#ifdef __cplusplus
extern "C"
{
#endif

#define TASK_CAN_RX_PRIO      ( 2 )
#define TASK_CAN_RX_STKSIZE   ( 1024 )

/**
 * @public
 * @brief create can receive task
 * @retval int non-zero on error
 */
int
  create_task_can_rx( void );

#ifdef __cplusplus
}
#endif

#endif // __task_can_h