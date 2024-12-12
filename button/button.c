/*********************************************************************************
 * Includes
 ********************************************************************************/
#include "boardio.h"
#include "button.h"
#include "tick.h"

/*********************************************************************************
 * Local data types
 ********************************************************************************/
/**
 * Local button states
 */
typedef enum state
{
    STATE_RELEASED = 0,
    STATE_PRESS_DEBOUNCE,
    STATE_PRESSED,
    STATE_RELEASE_DEBOUNCE
}state_t;

/**
 * Button data type
 */
typedef struct button
{
    state_t state;          // button machine states
    bool active_logic;      // true: active high, false: active low
    bool (*read_pin)();     // pointer to the function getting the pin value
    bool pressed;           // Flag indicating button has been pressed
    bool released;          // Flag indicating button has been released
    int32_t hold_timer;     // Timer for counting the press time
}button_t;

/*********************************************************************************
 * Local constants and defines
 ********************************************************************************/
// Button debounce time, used to refresh the button machine
#define BUTTON_DEBOUNCE_TIME_mS     20      // Debounce time in milliseconds

/*********************************************************************************
 * Local variables
 ********************************************************************************/
/**
 * Local data container
 */
static struct local_data
{
    button_t btn[BUTTON_MAX];       // Array of buttons used
    uint32_t timer;                 // Local timer
}this;

/*********************************************************************************
 * Callback functions
 ********************************************************************************/
/**
 * Timer callback function
 */
static void timer_callback()
{
    if(this.timer) this.timer--;
    for(int i = 0; i < BUTTON_MAX; i++)
    {
        if(this.btn[i].hold_timer >=0) this.btn[i].hold_timer++;
    }
}

/*********************************************************************************
 * Local functions
 ********************************************************************************/
// Local timer functions
static void SetTimer(uint32_t ms) {this.timer = ms;}
static bool Timeout() {return this.timer == 0;}

/*********************************************************************************
 * Public functions
 ********************************************************************************/
void BUTTON_init()
{
    int i;

    // tick initialization
    TICK_CallbackRegister((tick_callback_t)timer_callback);

    // Initialize each button
    this.btn[BUTTON_1].read_pin = BOARDIO_button_get;
    this.btn[BUTTON_1].active_logic = false;

    // Initialize all the common parammeters
    for(i = 0; i < BUTTON_MAX; i++)
    {
        this.btn[i].state = STATE_RELEASED;
        this.btn[i].pressed = false;
        this.btn[i].released = false;
        this.btn[i].hold_timer = -1;
    }
}

void BUTTON_tasks()
{
    int i;

    if(!Timeout()) return;

    SetTimer(BUTTON_DEBOUNCE_TIME_mS);

    for(i = 0; i < BUTTON_MAX; i++)
    {
        switch(this.btn[i].state)
        {           
            case STATE_RELEASED:
                if(this.btn[i].read_pin() == this.btn[i].active_logic)
                {
                    this.btn[i].state = STATE_PRESS_DEBOUNCE;
                    this.btn[i].hold_timer = 0;     // enable hold timer   
                }
                break;

            case STATE_PRESS_DEBOUNCE:
                if(this.btn[i].read_pin() == this.btn[i].active_logic)
                {
                    this.btn[i].pressed = true;
                    this.btn[i].released = false;
                    this.btn[i].state = STATE_PRESSED;
                }
                else
                {
                    this.btn[i].state = STATE_RELEASED;
                    this.btn[i].hold_timer = -1;    // disable hold timer   
                }
                break;

            case STATE_PRESSED:
                if(this.btn[i].read_pin() != this.btn[i].active_logic)
                {
                    this.btn[i].state = STATE_RELEASE_DEBOUNCE;
                }
                break;

            case STATE_RELEASE_DEBOUNCE:
                if(this.btn[i].read_pin() != this.btn[i].active_logic)
                {
                    this.btn[i].pressed = false;
                    this.btn[i].released = true;
                    this.btn[i].state = STATE_RELEASED;
                    this.btn[i].hold_timer = -1;    // disable hold timer   
                }
                else
                {
                    this.btn[i].state = STATE_PRESSED;
                }
                break;

            default: break;
        }
    }
}

bool BUTTON_is_pressed(button_id_t id)
{
    return this.btn[id].state == STATE_PRESSED;
}

bool BUTTON_was_pressed(button_id_t id)
{
    if(this.btn[id].pressed)
    {
        this.btn[id].pressed = false;
        return true;    
    }
    else
    {
        return false;
    }
}

bool BUTTON_is_released(button_id_t id)
{
    return this.btn[id].state == STATE_RELEASED;
}

bool BUTTON_was_released(button_id_t id)
{
    if(this.btn[id].released)
    {
        this.btn[id].released = false;
        return true;    
    }
    else
    {
        return false;
    }
}

uint32_t BUTTON_get_press_time_ms(button_id_t id)
{
    if(this.btn[id].hold_timer >= 0)
    {
        return (uint32_t)this.btn[id].hold_timer;
    }
    else
    {
        return 0;
    }
}
