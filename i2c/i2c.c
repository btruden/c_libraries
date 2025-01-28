/*******************************************************************************
 * @file        i2c.c
 * @brief       This file provides a generic i2c functionality that is capable
 *              of handling multiple i2c clients/instances.
 * @author      Blas Truden
 * @date        20241219
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
#include "i2c.h"
#include "queue.h"
#include "debug.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        false
#define DEBUG_TAG           "I2C"

#if ENABLE_DEBUG == true
#define DEBUG_MSG(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

#define PRINT_LINE(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)

// I2C queue size
#define I2C_QUEUE_SIZE      64


/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states type
 */
typedef enum
{
    STATE_IDLE = 0,
    STATE_PROCESSING,
    STATE_ERROR
}state_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
    // Main states
    state_t state;

    // Queue for the i2c transactions
    queue_t q;
    
    // Buffer of transaction pointers
    I2C_transaction_t *tr_buf[I2C_QUEUE_SIZE];  

    // Current transaction
    I2C_transaction_t *t;  
    
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
void I2C_init()
{ 
    this.state = STATE_IDLE;
    QUEUE_create(&this.q, this.tr_buf, I2C_QUEUE_SIZE, sizeof(I2C_transaction_t *));
}

void I2C_tasks()
{ 
    switch (this.state)
    {
        case STATE_IDLE:
            if(QUEUE_pull(&this.q, &this.t))
            {
                DEBUG_MSG("Transaction pulled");
                switch (this.t->type)   
                {
                    case I2C_TRANSACTION_WRITE:
                        if(SERCOM3_I2C_Write(this.t->address, this.t->tx_data, this.t->tx_length))
                        {
                            DEBUG_MSG("Write transaction started");
                            DEBUG_MSG("address: 0x%02X", this.t->address);
                            DEBUG_MSG("tx_length: %d", this.t->tx_length);
                            this.state = STATE_PROCESSING;
                            this.t->status = I2C_TRANSACTION_STATUS_BUSY;
                        }
                        else
                        {
                            DEBUG_MSG("Error starting write transaction");
                            this.t->status = I2C_TRANSACTION_STATUS_ERROR;
                        }
                        break;
                    
                    case I2C_TRANSACTION_READ:
                        if(SERCOM3_I2C_Read(this.t->address, this.t->rx_data, this.t->rx_length))
                        {
                            DEBUG_MSG("Read transaction started");
                            DEBUG_MSG("address: 0x%02X", this.t->address);
                            DEBUG_MSG("rx_length: %d", this.t->rx_length);
                            this.state = STATE_PROCESSING;
                            this.t->status = I2C_TRANSACTION_STATUS_BUSY;
                        }
                        else
                        {
                            DEBUG_MSG("Error starting read transaction");
                            this.t->status = I2C_TRANSACTION_STATUS_ERROR;
                        }
                        break;
                    
                    case I2C_TRANSACTION_WRITE_READ:
                        if(SERCOM3_I2C_WriteRead(this.t->address, this.t->tx_data, this.t->tx_length, this.t->rx_data, this.t->rx_length))
                        {
                            DEBUG_MSG("WriteRead transaction started");
                            DEBUG_MSG("address: 0x%02X", this.t->address);
                            DEBUG_MSG("tx_length: %d", this.t->tx_length);
                            DEBUG_MSG("rx_length: %d", this.t->rx_length);
                            this.state = STATE_PROCESSING;
                            this.t->status = I2C_TRANSACTION_STATUS_BUSY;
                        }
                        else
                        {
                            DEBUG_MSG("Error starting WriteRead transaction");
                            this.t->status = I2C_TRANSACTION_STATUS_ERROR;
                        }
                        break;
                    
                    default:
                        break;
                }
            }   
            break;

        case STATE_PROCESSING:
            if(!SERCOM3_I2C_IsBusy())
            {
                SERCOM_I2C_ERROR e = SERCOM3_I2C_ErrorGet();

                switch(e)
                {
                    case SERCOM_I2C_ERROR_NONE:
                        DEBUG_MSG("Transaction done");
                        this.t->status = I2C_TRANSACTION_STATUS_DONE;
                        break;
                    
                    case SERCOM_I2C_ERROR_NAK:
                        DEBUG_MSG("Transaction NAK");
                        this.t->status = I2C_TRANSACTION_STATUS_ERROR;
                        break;
                    
                    case SERCOM_I2C_ERROR_BUS:
                        DEBUG_MSG("Transaction BUS error");
                        this.t->status = I2C_TRANSACTION_STATUS_ERROR;
                        break;
                    
                    default:
                        DEBUG_MSG("Transaction error");
                        this.t->status = I2C_TRANSACTION_STATUS_ERROR;
                        break;
                }

                this.state = STATE_IDLE;
            }
            break;
        
        default:
            break;
    }
}

bool I2C_add_transaction(I2C_transaction_t *t)
{
    t->status = I2C_TRANSACTION_STATUS_IDLE;

    if(QUEUE_push(&this.q, &t))
    {
        DEBUG_MSG("Transaction added");
        return true;
    }
    else
    {
        DEBUG_MSG("Error adding transaction");
        return false;
    }
}