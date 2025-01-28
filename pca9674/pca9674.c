/*******************************************************************************
 * @file        pca9674.c
 * @brief       This file is a driver for operating the PCA9674 I2C I/O 
 *              expander. This module makes use of the i2c.h generic module.
 * @author      Blas Truden
 * @date        20241220
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
#include "pca9674.h"
#include "debug.h"
#include "i2c.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        false
#define DEBUG_TAG           "PCA9674"

#if ENABLE_DEBUG == true
#define DEBUG_MSG(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

#define PRINT_LINE(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)

#define MAX_INSTANCES       64

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states type
 */
typedef enum
{
    STATE_UNCONFIGURED = -1,
    STATE_IDLE,
    STATE_CONFIGURING,
    STATE_WRITING,
    STATE_READING,
}state_t;

/**
 * PCA9674 instance context
 */
typedef struct pca9674_instance_context
{
    state_t state;                      // Current state of the instance machine
    PCA9674_status_t status;            // Status of the instance operation
    PCA9674_config_t config;            // Configuration structure
    uint8_t port_state;                 // Port state
    I2C_transaction_t i2c_transaction;  // I2C transaction object
    uint8_t write_data;                 // I2C write data buffer (1 Byte)
    uint8_t read_data;                  // I2C read data buffer (1 Byte)
    PCA9674_read_callback_t read_cb;    // Read callback function
}pca9674_instance_context_t;


/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
    pca9674_instance_context_t instances[MAX_INSTANCES];
    uint32_t instance_idx;
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/

/**
 * Local task machine
 */
static void task()
{
    for(uint32_t i = 0; i < this.instance_idx; i++)
    {
        switch (this.instances[i].state)
        {   
            case STATE_WRITING:
            case STATE_CONFIGURING:
                switch(this.instances[i].i2c_transaction.status)
                {
                    case I2C_TRANSACTION_STATUS_DONE:
                        this.instances[i].status = PCA9674_STATUS_OK;
                        this.instances[i].state = STATE_IDLE;
                        break;
                    
                    case I2C_TRANSACTION_STATUS_ERROR:
                        DEBUG_MSG("I2C transaction error while writing port");
                        this.instances[i].status = PCA9674_STATUS_ERROR;
                        this.instances[i].state = STATE_IDLE;
                        break;
                    
                    default:
                        break;
                }
                break;

            case STATE_READING:
                switch(this.instances[i].i2c_transaction.status)
                {
                    case I2C_TRANSACTION_STATUS_DONE:
                        this.instances[i].port_state = this.instances[i].read_data;

                        DEBUG_MSG("[instance %d] Port read: 0x%02x", i, this.instances[i].port_state);

                        // Execute callback, if registered
                        if(this.instances[i].read_cb != NULL)
                        {
                            this.instances[i].read_cb(i, this.instances[i].port_state);
                        }

                        this.instances[i].status = PCA9674_STATUS_OK;
                        this.instances[i].state = STATE_IDLE;
                        break;
                    
                    case I2C_TRANSACTION_STATUS_ERROR:
                        DEBUG_MSG("I2C transaction error while reading port");
                        this.instances[i].status = PCA9674_STATUS_ERROR;
                        this.instances[i].state = STATE_IDLE;
                        break;
                    
                    default:
                        break;
                }
                break;

            case STATE_IDLE:
            case STATE_UNCONFIGURED:          
            default:
                break;
        }
    }
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void PCA9674_init()
{ 
    this.instance_idx = 0;
    for(int i = 0; i < MAX_INSTANCES; i++)
    {
        this.instances[i].state = STATE_UNCONFIGURED;
        this.instances[i].port_state = 0xff;
        this.instances[i].read_cb = NULL;
    }
}

void PCA9674_tasks()
{ 
    task();
}

PCA9674_handle_t PCA9674_open(PCA9674_config_t config)
{
    if(this.instance_idx < MAX_INSTANCES)
    {
        DEBUG_MSG("Opening instance %d", this.instance_idx);

        // Copy the configuration to the instance
        this.instances[this.instance_idx].config = config;
        // Set the instance state to configuring
        this.instances[this.instance_idx].state = STATE_CONFIGURING;
        // Set the instance status to idle
        this.instances[this.instance_idx].status = PCA9674_STATUS_PROCESSING;
        
        // Set the read callback function
        if(config.read_cb != NULL)
            this.instances[this.instance_idx].read_cb = config.read_cb;

        // Initialize the I2C transaction for the instance configuration
        this.instances[this.instance_idx].i2c_transaction.address = config.i2c_address;
        this.instances[this.instance_idx].i2c_transaction.type = I2C_TRANSACTION_WRITE;
        this.instances[this.instance_idx].i2c_transaction.tx_length = 1;
        this.instances[this.instance_idx].i2c_transaction.tx_data = &this.instances[this.instance_idx].write_data;
        this.instances[this.instance_idx].i2c_transaction.rx_length = 0;
        this.instances[this.instance_idx].i2c_transaction.rx_data = &this.instances[this.instance_idx].read_data;
        
        // Prepare the data to write. If configured as input or set as output and initial state is 
        // high, the bit is set to 1. Otherwise, it is set to 0.
        this.instances[this.instance_idx].write_data = config.port_cfg | config.port_initial_state;

        DEBUG_MSG("port config: 0x%02x", config.port_cfg);
        DEBUG_MSG("port initial state: 0x%02x", config.port_initial_state);
        DEBUG_MSG("write data: 0x%02x", this.instances[this.instance_idx].write_data);

        // Start the I2C transaction
        if(I2C_add_transaction(&this.instances[this.instance_idx].i2c_transaction))
        {
            this.instances[this.instance_idx].port_state = this.instances[this.instance_idx].write_data;
            this.instance_idx++;
            return this.instance_idx - 1;
        }
        else
        {
            DEBUG_MSG("Couldn't add I2C transaction");
            return -1;
        }
    }
    else
    {
        return -1;
    }
}

bool PCA9674_set_port_bit(PCA9674_handle_t handle, uint8_t bit, uint8_t state)
{
    DEBUG_MSG("[instance %d]Set port bit %d to %d", handle, bit, state);
    if(handle < this.instance_idx)
    {
        // Check if the pin was configured as input
        if(this.instances[handle].config.port_cfg & (1 << bit))
        {
            DEBUG_MSG("Pin %d is configured as input", bit);
            return false;
        }
        else
        {
            if(this.instances[handle].state == STATE_IDLE)
            {                
                // Initialize the I2C transaction for the instance configuration
                this.instances[handle].i2c_transaction.type = I2C_TRANSACTION_WRITE;
                this.instances[handle].i2c_transaction.tx_length = 1;
                
                // Turn the bit high or low based on the current state
                if(state)
                {
                    this.instances[handle].write_data = this.instances[handle].port_state | (1 << bit);
                }
                else
                {
                    this.instances[handle].write_data = this.instances[handle].port_state & ~(1 << bit);
                }

                // Start the I2C transaction
                if(I2C_add_transaction(&this.instances[handle].i2c_transaction))
                {
                    this.instances[handle].port_state = this.instances[handle].write_data;

                    // Set the instance state to writings
                    this.instances[handle].state = STATE_WRITING;
                    // Set the instance status to processing
                    this.instances[handle].status = PCA9674_STATUS_PROCESSING;
                    
                    return true;
                }
                else
                {
                    this.instances[handle].status = PCA9674_STATUS_ERROR;
                    DEBUG_MSG("Couldn't add I2C transaction");
                    return false;
                }
            }
            else
            {
                DEBUG_MSG("Instance is not ready");
                return false;
            }
        }
    }
    else
    {
        DEBUG_MSG("Invalid handle");
        return false;
    }
}

bool PCA9674_set_port_byte(PCA9674_handle_t handle, uint8_t data)
{
    DEBUG_MSG("[instance %d] Set port Byte to 0x%02x", handle, data);
    if(handle < this.instance_idx)
    {
        if(this.instances[handle].state == STATE_IDLE)
        {            
            // Initialize the I2C transaction for the instance configuration
            this.instances[handle].i2c_transaction.type = I2C_TRANSACTION_WRITE;
            this.instances[handle].i2c_transaction.tx_length = 1;
            
            // Set the port state to the new data
            this.instances[handle].write_data = this.instances[handle].config.port_cfg | data;

            // Start the I2C transaction
            if(I2C_add_transaction(&this.instances[handle].i2c_transaction))
            {
                this.instances[handle].port_state = this.instances[handle].write_data;

                // Set the instance state to writings
                this.instances[handle].state = STATE_WRITING;
                // Set the instance status to processing
                this.instances[handle].status = PCA9674_STATUS_PROCESSING;
                return true;
            }
            else
            {
                this.instances[handle].status = PCA9674_STATUS_ERROR;
                DEBUG_MSG("Couldn't add I2C transaction");
                return false;
            }
        }
        else
        {
            DEBUG_MSG("Instance is not ready");
            return false;
        }
    }
    else
    {
        DEBUG_MSG("Invalid handle");
        return false;
    }
}

uint8_t PCA9674_static_read_port_byte(PCA9674_handle_t handle)
{
    if(handle < this.instance_idx)
    {
        return this.instances[handle].port_state;
    }
    else
    {
        DEBUG_MSG("Invalid handle");
        return 0;
    }
}

bool PCA9674_static_read_bit(PCA9674_handle_t handle, uint8_t bit)
{
    if(handle < this.instance_idx)
    {
        return (this.instances[handle].port_state & (1 << bit)) >> bit;
    }
    else
    {
        DEBUG_MSG("Invalid handle");
        return false;
    }
}

bool PCA9674_read(PCA9674_handle_t handle)
{
    if(handle < this.instance_idx)
    {
        if(this.instances[handle].state == STATE_IDLE)
        {            
            DEBUG_MSG("[instance %d] Read port ", handle);
            
            // Initialize the I2C transaction for the instance configuration
            this.instances[handle].i2c_transaction.type = I2C_TRANSACTION_READ;
            this.instances[handle].i2c_transaction.tx_length = 0;
            this.instances[handle].i2c_transaction.rx_length = 1;

            // Start the I2C transaction
            if(I2C_add_transaction(&this.instances[handle].i2c_transaction))
            {
                // Set the instance state to readings
                this.instances[handle].state = STATE_READING;
                // Set the instance status to processing
                this.instances[handle].status = PCA9674_STATUS_PROCESSING;
                return true;
            }
            else
            {
                this.instances[handle].status = PCA9674_STATUS_ERROR;
                DEBUG_MSG("Couldn't add I2C transaction");
                return false;
            }
        }
        else
        {
            DEBUG_MSG("Instance is not ready");
            return false;
        }
    }
    else
    {
        DEBUG_MSG("Invalid handle");
        return false;
    }
}

bool PCA9674_is_ready(PCA9674_handle_t handle)
{
    if(handle < this.instance_idx)
    {
        return this.instances[handle].state == STATE_IDLE;
    }
    else
    {
        DEBUG_MSG("Invalid handle");
        return false;
    }
}