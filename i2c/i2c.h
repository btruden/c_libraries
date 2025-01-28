/***************************************************************************//**
 * @file        i2c.h
 * @brief       Provides the necessary interface elements for a successful use 
 *              of the functionality.
 * @author      Blas Truden
 * @date        20241219
 * @version     v1
 * 
 * @copyright   -
 * 
 * @details     This module is part of the BSI BSP core.
 ******************************************************************************/
#ifndef I2C_H_
#define I2C_H_

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
 * I2C transaction type
 */
typedef enum
{
    I2C_TRANSACTION_WRITE = 0,
    I2C_TRANSACTION_READ,
    I2C_TRANSACTION_WRITE_READ
}I2C_transaction_type_t;

/**
 * I2C transaction status
 */
typedef enum
{
    I2C_TRANSACTION_STATUS_IDLE = 0,
    I2C_TRANSACTION_STATUS_BUSY,
    I2C_TRANSACTION_STATUS_DONE,
    I2C_TRANSACTION_STATUS_ERROR
}I2C_transaction_status_t;

/**
 * I2C transaction structure
 */
typedef struct
{
    uint8_t address;                // I2C address
    I2C_transaction_type_t type;    // Transaction type
    uint8_t *tx_data;               // Pointer to the data to be transmitted
    uint8_t tx_length;              // Length of the data to be transmitted
    uint8_t *rx_data;               // Pointer to the data to be received
    uint8_t rx_length;              // Length of the data to be received
    I2C_transaction_status_t status;// Transaction status
}I2C_transaction_t;


/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * Module initialization. Must be called once during startup.
 */
void I2C_init();

/**
 * Module tasks. Must be called periodically.
 */
void I2C_tasks();

/**
 * Add a transaction to the queue
 * 
 * @param t Pointer to the transaction structure
 * 
 * @return true if the transaction was added to the queue, false otherwise
 */
bool I2C_add_transaction(I2C_transaction_t *t);

#endif /* I2C_H_ */
