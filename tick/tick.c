/*
 * tick.c
 *
 *  Created on: Aug 18, 2021
 *      Author: btrud
 */

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stddef.h>
#include <stdint.h>
#include "definitions.h"
#include "tick.h"
#include "debug.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Maximum number of callback functions
#define CALLBACK_MAX	16

// Debug options
#define ENABLE_DEBUG        false
#define DEBUG_TAG           "TICK"

#if ENABLE_DEBUG == true
#define DEBUG_PRINTF(format, ...) Debug_Print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#define PRINTF(format, ...) Debug_Print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) Debug_Print(0, false, format, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(format, ...)
#define PRINTF(format, ...)
#define PRINT(format, ...)
#endif

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states
 */
typedef enum
{
	STATE_INIT,
	STATE_IDLE
}states_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
	states_t state;							// Local state variable

	tick_callback_t callback[CALLBACK_MAX];	// Callbacks array

	uint32_t call_idx;						// Callback index

}this = {
	.call_idx = 0
};

/******************************************************************************
 * Local Functions
 ******************************************************************************/

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
void TICK_TimerCallback(TC_TIMER_CALLBACK status, uintptr_t context )
{
	int i;

	for(i = 0; i < this.call_idx; i++)
	{
		if(this.callback[i] != NULL) this.callback[i]();
	}
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void TICK_Init()
{ 
    TC0_TimerCallbackRegister((TC_TIMER_CALLBACK)TICK_TimerCallback,(uintptr_t)NULL);
    TC0_TimerStart();
    
    DEBUG_PRINTF("Tick registered to timer callback");
    DEBUG_PRINTF("Tmr Started");
    
}

void TICK_CallbackRegister(tick_callback_t func)
{
	if(this.call_idx >= CALLBACK_MAX-1) return;

	this.callback[this.call_idx++] = func;
    DEBUG_PRINTF("Callback registered to tick module");
}
