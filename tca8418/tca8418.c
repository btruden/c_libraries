/*******************************************************************************
 * @file        tca8418.c
 * @brief       This file provides a driver for operating the TCA8418 keypad 
 *              scan IC
 * @author      Blas Truden
 * @date        20250128
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
#include "tca8418.h"
#include "tca8418_defs.h"
#include "debug.h"
#include "tick.h"
#include "i2c.h"
#include "queue.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "TCA8418"

#if ENABLE_DEBUG == true
#define DEBUG_MSG(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

#define PRINT_LINE(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)

// Timer defines
#define PWR_ON_TIMEOUT_MS       100

// Event queue
#define EVENTS_QUEUE_SIZE       32

/**
 * Default configuration array containing register <address,value> pairs
 */
static uint8_t config_array[] = 
{
    // Auto-increment disabled, GPI evts tracked w kpd locked, ovflw disabled,
    // INT reaserted after 50uS if pending ints, ovflw int disabled, kypad lock
    // int disabled, GPI int disabled, key evts int enabled
    TCA8418_REG_CFG, TCA8418_CFG_INT_CFG_MASK | TCA8418_CFG_KE_IEN_MASK,

    // Clear key evt interrupts
    TCA8418_REG_INT_STAT, TCA8418_INT_STAT_K_INT_MASK,

    // Keypad/GPIO select 1 configured with all rows dedicated to keypad
    TCA8418_REG_KP_GPIO1, 0xFF,

    // Keypad/GPIO select 2 configured with all columns dedicated to keypad
    TCA8418_REG_KP_GPIO2, 0xFF,

    // Keypad/GPIO select 3 configured with all columns dedicated to keypad
    TCA8418_REG_KP_GPIO3, 0x03,
};

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states type
 */
typedef enum
{
    STATE_INIT = 0,
    STATE_CONFIGURING,
    STATE_RUNNING
}state_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
    state_t state;
    uint32_t tmr;

    // Event callback function
    TCA8418_evt_callback_t cb;

    // Events queue
    queue_t q;
    TCA8418_key_evt_t evts_buf[EVENTS_QUEUE_SIZE];

    bool read_evts;
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Loads the local timer with ms
 */
static void timer_set(uint32_t ms)
{
    this.tmr = ms;
}

/**
 * Returns true if the timer is zero
 */
static bool timer_out()
{
    return this.tmr == 0;
}

/**
 * Writes multiple registers based on an array containing <address,value> pairs
 * 
 * @param data pointer to the data buffer containing register <address,value>
 * pairs
 * @param length amount of registers to write (array_size/2)
 * @param init true if the function is called for the first time
 * 
 * @return true if the registers were read
 */
static bool write_regs(uint8_t *data, uint8_t length, bool init)
{
    bool ret = false;
    static uint8_t *tx_data;
    static uint16_t total_regs, cnt;
    static I2C_transaction_t t;

    static enum
    {
        WRSTATE_INIT = 0,
        WRSTATE_WRITING_REGS
    }wrState = WRSTATE_INIT;

    if(init)
    {
        tx_data = data;
        total_regs = length;
        cnt = 0;
        t.address = TCA8418_I2C_ADDR;
        t.tx_data = tx_data;
        t.tx_length = 2;
        t.type = I2C_TRANSACTION_WRITE;
        wrState = WRSTATE_INIT;
    }

    switch (wrState)
    {
        case WRSTATE_INIT:
            if(I2C_add_transaction(&t))
            {
                wrState = WRSTATE_WRITING_REGS;
                DEBUG_MSG("Writing regiter 0x%02x with value 0x%02x", t.tx_data[0], t.tx_data[1]);
            }
            else
            {
                DEBUG_MSG("Error adding transaction");
                ret = true;
                wrState = WRSTATE_INIT;
            }

            break;
        
        case WRSTATE_WRITING_REGS:
            switch(t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    cnt++;
                    if(cnt < total_regs)
                    {
                        tx_data += 2;
                        t.tx_data = tx_data;
                        DEBUG_MSG("Writing regiter 0x%02x with value 0x%02x", t.tx_data[0], t.tx_data[1]);
                        if(!I2C_add_transaction(&t))
                        {
                            DEBUG_MSG("Error adding transaction");
                            ret = true;
                            wrState = WRSTATE_INIT;
                        }
                    }
                    else
                    {
                        DEBUG_MSG("Registers written");
                        ret = true;
                        wrState = WRSTATE_INIT;
                    }
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error writing register");
                    ret = true;
                    wrState = WRSTATE_INIT;
                    break;
                
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return ret;
}

/**
 * State machine that configures the chip.
 * 
 * @param init  Must be true the first call to initialize the state machine
 * 
 * @return "true" when finished
 */
static bool config(bool init)
{
    return write_regs(config_array,sizeof(config_array)/2,init);
}

/**
 * Monitors the keypad keys
 */
static void key_monitor(bool init)
{
    static I2C_transaction_t t;
    static uint8_t txbuf[8], rxbuf[8];
    static uint8_t evts_qty;
    static TCA8418_key_evt_t evt;

    static enum
    {
        MSTATE_IDLE = 0,
        MSTATE_GETTING_EVT_QTY,
        MSTATE_GETTING_EVT,
        MSTATE_CLR_INT,
        MSTATE_ERROR,

    }mState = MSTATE_IDLE; 

    if(init)
    {
        DEBUG_MSG("key monitor init");
        mState = MSTATE_IDLE;

        // Init I2C transaction structure
        t.address = TCA8418_I2C_ADDR;
        t.tx_data = txbuf;
        t.rx_data = rxbuf;
        QUEUE_create(&this.q,this.evts_buf,EVENTS_QUEUE_SIZE,sizeof(TCA8418_key_evt_t));
        return;
    }

    switch(mState)
    {
        case MSTATE_IDLE:
            if(this.read_evts)
            {
                this.read_evts = false;
                txbuf[0] = TCA8418_REG_KEY_LCK_EC;
                t.tx_length = 1;
                t.rx_length = 1;
                t.type = I2C_TRANSACTION_WRITE_READ;
                if(I2C_add_transaction(&t))
                {
                    DEBUG_MSG("Getting evt qty");
                    mState = MSTATE_GETTING_EVT_QTY;
                }
                else
                {
                    DEBUG_MSG("Unable to start I2C transaction");
                    mState = MSTATE_ERROR;
                }
            }
            break;
        
        case MSTATE_GETTING_EVT_QTY:
            switch (t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    // Get the events qty
                    evts_qty = rxbuf[0] & TCA8418_KEY_LCK_EC_KEC_MASK;

                    DEBUG_MSG("%d new key events found");

                    if(evts_qty > 0)
                    {
                        txbuf[0] = TCA8418_REG_KEY_EVENT_A;
                        if(I2C_add_transaction(&t))
                        {
                            mState = MSTATE_GETTING_EVT;
                        }
                        else
                        {
                            DEBUG_MSG("Unable to start I2C transaction");
                            mState = MSTATE_ERROR;
                        }
                    }
                    else
                    {
                        DEBUG_MSG("No new events. waiting...");
                        mState = MSTATE_IDLE;
                    }
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error during I2C transaction");
                    mState = MSTATE_ERROR;
                    break;
                
                default:
                    break;
            }
            break;

        case MSTATE_GETTING_EVT:
            switch (t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:

                    // Save the new event in the queue
                    evt.pressed = ((rxbuf[0] & TCA8418_KEY_EVENT_A_KEY_PRESS_MASK) > 0);
                    evt.key_id = rxbuf[0] & TCA8418_KEY_EVENT_A_KEY_ID_MASK;
                
                    if(!QUEUE_push(&this.q,&evt))
                        DEBUG_MSG("Unable to add evt to queue");
                        
                    // Check evt qty
                    evts_qty--;
                    if(evts_qty > 0)
                    {
                        txbuf[0] = TCA8418_REG_KEY_EVENT_A;
                        if(!I2C_add_transaction(&t))
                        {
                            DEBUG_MSG("Unable to start I2C transaction");
                            mState = MSTATE_ERROR;
                        }
                    }
                    else
                    {
                        DEBUG_MSG("No more events");

                        // Call the callback function if registered
                        if(this.cb != NULL)
                            this.cb();

                        DEBUG_MSG("Claring int flag");
                        
                        t.tx_length = 2;
                        t.rx_length = 0;
                        t.type = I2C_TRANSACTION_WRITE;
                        txbuf[0] = TCA8418_REG_INT_STAT;
                        txbuf[1] = TCA8418_INT_STAT_K_INT_MASK;

                        if(I2C_add_transaction(&t))
                        {
                            mState = MSTATE_CLR_INT;
                        }
                        else
                        {
                            DEBUG_MSG("Unable to start I2C transaction");
                            mState = MSTATE_ERROR;
                        }

                        mState = MSTATE_CLR_INT;
                    }
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error during I2C transaction");
                    mState = MSTATE_ERROR;
                    break;
                
                default:
                    break;
            }
            break;

        case MSTATE_CLR_INT:
            switch (t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    DEBUG_MSG("Inerrupts cleared");
                    mState = MSTATE_IDLE;
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error during I2C transaction");
                    mState = MSTATE_ERROR;
                    break;
                
                default:
                    break;
            }
            break;

        case MSTATE_ERROR:
        default:
            break;
    }
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
/**
 * Timer tick callback used to handle local timer
 */
static void tick_callback()
{
    if(this.tmr)
        this.tmr--;
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void TCA8418_init()
{ 
    TICK_callback_register((TICK_callback_t)tick_callback); 
    timer_set(PWR_ON_TIMEOUT_MS);
    this.state = STATE_INIT;
    this.read_evts = false;
    this.cb = NULL;
}

void TCA8418_tasks()
{ 
    switch (this.state)
    {
        case STATE_INIT:
            if(timer_out())
            {
                DEBUG_MSG("Configuring...");
                config(true);
                this.state = STATE_CONFIGURING;
            }            
            break;
        
        case STATE_CONFIGURING:
            if(config(false))
            {
                DEBUG_MSG("Configured. Running...");
                key_monitor(true);
                this.state = STATE_RUNNING;
            }
            break;

        case STATE_RUNNING:
            key_monitor(false);
            break;
        
        default:
            break;
    }
}

void TCA8418_request_events()
{
    this.read_evts = true;
}

void TCA8418_evt_callback_register(TCA8418_evt_callback_t cb)
{
    this.cb = cb;
}

void TCA8418_evt_callback_unregister()
{
    this.cb = NULL;
}

bool TCA8418_get_evt(TCA8418_key_evt_t *evt)
{
    return QUEUE_pull(&this.q,evt);
}