/***************************************************************************//**
 * @file        tca8418.h
 * @brief       Provides the necessary interface elements for a successful use 
 *              of the driver.
 * @author      Blas Truden
 * @date        20250128
 * @version     v1
 * 
 * @copyright   -
 * 
 * @details     This module is part of the BSI BSP core.
 ******************************************************************************/
#ifndef TCA8418_H_
#define TCA8418_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Public Constants
 ******************************************************************************/

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * TCA8418 key event containing information about the key ID and press status
 */
typedef struct
{
    uint8_t key_id;
    bool pressed;
}TCA8418_key_evt_t;

/**
 * TCA8418 event callback type that is executed when new events have been read
 * from the IC.
 * 
 * After this callback, the user must get the events by using the 
 * TCA8418_get_evt() recursivelly until there are no more events.
 */
typedef void (*TCA8418_evt_callback_t)(void);

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * Module initialization. Must be called once during startup.
 */
void TCA8418_init();

/**
 * Module tasks. Must be called periodically.
 */
void TCA8418_tasks();

/**
 * This function starts a read transaction for getting any pending key events.
 * 
 * It's recommended to call this function right uppon the INT signal triggers.
 * 
 * If there is a callback function registered, it'll be called when the event
 * reading process has finished.
 */
void TCA8418_request_events();

/**
 * Registers a callback function to be called on pending events
 */
void TCA8418_evt_callback_register(TCA8418_evt_callback_t cb);

/**
 * Unregisters the callback function called on pending events
 */
void TCA8418_evt_callback_unregister();

/**
 * This function gets any pending key event. 
 * 
 * @param evt   pointer to event structure where to store the event
 * 
 * @return      "true" if a new event has been got. "false" if no events
 */
bool TCA8418_get_evt(TCA8418_key_evt_t *evt);


#endif /* TCA8418_H_ */
