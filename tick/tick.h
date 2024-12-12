/*
 * tick.h
 *
 *  Created on: Aug 18, 2021
 *      Author: btrud
 */

#ifndef INC_TICK_H_
#define INC_TICK_H_

/******************************************************************************
 * Public Constants
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * Tick callback type
 */
typedef void (*tick_callback_t)(void);

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
/**
 * Timer Callback
 */
void TICK_TimerCallback();

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * Module initialization
 */
void TICK_Init();

/**
 * Registers a callback for the timer tick
 */
void TICK_CallbackRegister(tick_callback_t func);

#endif /* INC_TICK_H_ */
