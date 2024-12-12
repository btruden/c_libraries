#ifndef _BUTTON_H
#define _BUTTON_H

/*********************************************************************************
 * Includes
 ********************************************************************************/
#include "definitions.h"

/*********************************************************************************
 * Public data types
 ********************************************************************************/
/**
 * list of the buttons used in the project
 */
typedef enum button_id
{
    BUTTON_1 = 0,

    BUTTON_MAX
}button_id_t;

/*********************************************************************************
 * Public constants and defines
 ********************************************************************************/

/*********************************************************************************
 * Public functions
 ********************************************************************************/
/**
 * @brief button initialization
 */
void BUTTON_init();

/**
 * button tasks
 */
void BUTTON_tasks();

/**
 * @brief returns if the button has been pressed. If was pressed, this function
 * clears the pressed flag
 * 
 * @param id button_id_t to check
 */
bool BUTTON_is_pressed(button_id_t id);

/**
 * @brief Returns if the button is being pressed
 * 
 * @param id button_id_t to check
 */
bool BUTTON_was_pressed(button_id_t id);

/**
 * returns if the button has been released. If was released, this function
 * clears the released flag
 * 
 * @param id button_id_t to check
 */
bool BUTTON_is_released(button_id_t id);

/**
 * @brief Returns if the button is being released
 * 
 * @param id button_id_t to check
 */
bool BUTTON_was_released(button_id_t id);

/**
 * @brief Returns the amount of time in milliseconds that the button has been
 * pressed. If the button is not pressed, this function return 0.
 * 
 * @param id button id
 * @return uint32_t time in milliseconds
 */
uint32_t BUTTON_get_press_time_ms(button_id_t id);

#endif