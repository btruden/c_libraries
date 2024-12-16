/*******************************************************************************
 * @file        tick.c
 * @brief       This file provides a 1 ms tick functionality.
 * @author      Blas Truden
 * @date        20241210
 * @version     v1
 * 
 * @copyright   -
 * 
 * @note		This module is part of the BSI BSP core.
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "definitions.h"
    #warning Pending to adapt
#include "template.h"		// TODO: change
//#include "debug.h"		// TODO: include if necessary

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        false
    #warning Pending to adapt
#define DEBUG_TAG           "-"

#if ENABLE_DEBUG == true
#define DEBUG_PRINTF(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#define PRINTF(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(format, ...)
#define PRINTF(format, ...)
#define PRINT(format, ...)
#endif


/******************************************************************************
 * Local Types
 ******************************************************************************/

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{

}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
    #warning Pending to adapt
void TEMPLATE_init()
{ 

}

    #warning Pending to adapt
void TEMPLATE_tasks()
{ 

}