/*
 * spi.h
 *
 *  Created on: Aug 27, 2021
 *      Author: btrud
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/******************************************************************************
 * Public Constants
 ******************************************************************************/
// Maximum transaction size
#define SPI_MAX_TRANSACTION_SIZE   UINT16_MAX


/******************************************************************************
 * Public Types
 ******************************************************************************/

/**
 * SPI CS line assert function pointer type. Part of the transaction structure.
 */
typedef void (*spi_assertCS_t)(void);

/**
 * SPI CS line release function pointer type. Part of the transaction structure.
 */
typedef void (*spi_releaseCS_t)(void);

/**
 * transaction status type. Used for tracking the status of a transaction. Part
 * of the transaction structure.
 */
typedef enum
{
	SPI_TRANSACTION_STATUS_IDLE = 0,
	SPI_TRANSACTION_STATUS_WORKING,
	SPI_TRANSACTION_STATUS_FINISHED,
	SPI_TRANSACTION_STATUS_FAILED
}spi_transaction_status_t;

/**
 * SPI transaction structure type
 */
typedef struct spi_transaction
{
    uint8_t *txbuf;							// TX buffer pointer
    uint8_t *rxbuf;							// RX buffer pointer
    uint32_t length;						// Transaction length

    spi_transaction_status_t status;		// Transaction status

    spi_assertCS_t assertCS;				// SPI CS line assert function pointer
    spi_releaseCS_t releaseCS;				// SPI CS line assert function pointer

    void (*callback)(struct spi_transaction *t);	// Callback function
}spi_transaction_t;

/**
 * @brief Callback function type to be called when the transaction was processed.
 * Part of the transaction structure.
 *
 * @param the callback will give as a paremeter the tranasaction that triggered
 * the event. The user can check the <pi_transaction_status_t> in order to know
 * if the transaction was correctly processed of there was an error.
 */
typedef void (*spi_transaction_callback_t)(spi_transaction_t *t);

/**
 * @brief Callback function type to be called when all transactions in queue
 * were sent
 *
 */
typedef void (*spi_general_callback_t)(void);

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * Initializes the SPI module
 */
void SPI_Init();

/**
 * Keeps the SPI transactions working
 */
void SPI_Tasks();

/**
 * Returns if all the queued transactions have been processed
 */
bool SPI_isDone();

/**
 * Adds a transaction to the queue
 * @param  t transaction pointer
 * @return   true if ok, false if there was some issue
 */
bool SPI_AddTransaction(spi_transaction_t *t);

/**
 * @brief Completes the transaction structure with the given information and adds it
 * to the transactions queue. After calling this function the transaction will be 
 * waiting for being sent.
 * 
 * @param t 
 * @param tx 
 * @param rx 
 * @param size 
 * @return true 
 * @return false 
 */


/**
 * @brief Completes the transaction structure with the given information and adds it
 * to the transactions queue. After calling this function the transaction will be 
 * waiting for being sent.
 * 
 * @param t transaction pointer
 * @param tx transmission buffer pointer
 * @param rx reception buffer pointer
 * @param size amount of Byte elements to be sent
 * @param assertCS Chip Select assert function pointer. If not needed set to NULL
 * @param releaseCS Chip Select release function pointer. If not needed set to NULL
 * @param clbck Callback function pointer for being called on any new transaction event
 * refer to the "spi_transaction_status_t". If not used, set to NULL.
 *
 * @note if the callback functionality is not used, the user must keep polling the
 * <status> parameter of the transaction structure to know the status of that single
 * transaction. As an alternative, the user can ask when the entire spi transactions
 * queue has been processed by calling the SPI_isDone() function or by setting the spi
 * general callback with the SPI_SetGeneralCallback() function.
 *
 *
 * @return true 
 * @return false 
 */
bool SPI_CreateTransactionAdd(spi_transaction_t *t, 
                            uint8_t *tx, 
                            uint8_t *rx, 
                            uint32_t size, 
                            spi_assertCS_t assertCS,
                            spi_releaseCS_t releaseCS,
							spi_transaction_callback_t clbck);

/**
 * @brief Registers a callback function to be called on each SPI plib callback
 *
 * @param func function callback pointer that will be called when all the transactions
 * in the spi queue were processed
 */
void SPI_SetGeneralCallback(spi_general_callback_t func);

/**
 * @brief Returns the status of the given transaction
 * 
 * @param t transaction pointer to be analyzed
 * @return spi_transaction_status_t current status of the given transaction
 */
spi_transaction_status_t SPI_GetTransactionStatus(spi_transaction_t *t);
