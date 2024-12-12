/***************************************************************************//**
 * @file        debug.h
 * @brief       Provides the necessary interface elements for a successful use 
 *              of the functionality.
 * @author      Blas Truden
 * @date        20241210
 * @version     v1
 * 
 * @copyright   -
 * 
 * @details     This module is part of the BSI BSP core.
 ******************************************************************************/

#ifndef INC_DEBUG_DEBUG_H_
#define INC_DEBUG_DEBUG_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/******************************************************************************
 * Public Constants
 ******************************************************************************/
#define RX_STREAM_ENABLE   false

/******************************************************************************
 * Public Types
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * @brief Initializes the debug module
 *
 */
void DEBUG_init();

/**
 * @brief main debug task machine
 *
 */
void DEBUG_tasks();

/**
 * Creates a debug string and send it to all the active suscriptos
 * @param tag special string identifying the module that is messaging.
 *            If the tag is NULL, the message will be printed without tag.
 * @param carryRet adds a carry return to the line
 * @param format of the message
 * @param ... variables
 */
void DEBUG_print(char *tag, bool carryRet, char *format, ...);

/**
 * @brief Gets a char form the debug stream
 *
 * @return uint8_t '0' if empty
 */
uint8_t DEBUG_getchar();

#if RX_STREAM_ENABLE == true
/**
 * @brief Clears the RX stream
 *
 */
void DEBUG_ClearRX();
#endif

#endif /* INC_DEBUG_DEBUG_H_ */
