/*
 * spi.c
 *
 *  Created on: Aug 27, 2021
 *      Author: btrud
 */


/******************************************************************************
 * Includes
 ******************************************************************************/
#include <debug/debug.h>
#include <imu/spi.h>
#include <imu/spi1.h>

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Size of the transaction queue
#define TRANSACTION_QUEUE_SIZE     64

// Size of a tx/rx element
#define SPI_TX_RX_ELEMENT_SIZE_BYTES    1

// Debug options
#define ENABLE_DEBUG        false
#define DEBUG_TAG           "SPI"

#if ENABLE_DEBUG == true
#define DEBUG_PRINTF(format, ...) Debug_Print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#define PRINTF(format, ...) Debug_Print(0, true, format, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(format, ...)
#define PRINTF(format, ...)
#endif

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states
 */
typedef enum
{
    TRSTATE_WAIT,
	TRSTATE_PERFORMING,
}transaction_states_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct
{
    // spi transaction pointers buffer
    spi_transaction_t *tra_queue[TRANSACTION_QUEUE_SIZE];

    // Pointer to the current processing transaction
    spi_transaction_t *t;

    // queue functionality variables
    uint32_t in_idx;
    uint32_t out_idx;
    bool queue_full;

    // Callback
    spi_general_callback_t callback;

    // transaction state variable
    transaction_states_t trState;
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Returns if the transactions queue is empty
 */
bool trQueue_isEmpty()
{
    return (this.in_idx == this.out_idx && !this.queue_full);
}

/**
 * Returns if the transaction queue is full
 */
bool trQueue_isFull()
{
    return this.queue_full;
}

/**
 * Takes the next existing transaction from queue
 */
bool trQueue_Pull(spi_transaction_t **tr)
{
    if(trQueue_isEmpty()) return false;

    // Copy the next queued transaction into given transaction pointer
    *tr = this.tra_queue[this.out_idx];

    this.out_idx = (this.out_idx + 1) % TRANSACTION_QUEUE_SIZE;
    this.queue_full = false;

    return true;
}

/**
 * Takes the next existing transaction from queue
 */
bool trQueue_Push(spi_transaction_t *tr)
{
    if(trQueue_isFull()) return false;

    // Copy the given transaction pointer into the queue
    this.tra_queue[this.in_idx] = tr;

    this.in_idx = (this.in_idx + 1) % TRANSACTION_QUEUE_SIZE;

    if(this.in_idx == this.out_idx) this.queue_full = true;

    return true;
}

/**
 * Starts the SPI transmission and reception process by called the SPI peripheral
 * function.
 *
 * @note The corresponding peripheral transmission/reception function should return
 * the status of the operation in a boolean return. If the peripheral function
 * doens't have that capability the user must manually return a "true" in this funciton.
 *
 * @return true if succeeded
 * @return false if failed
 */
bool trStart(spi_transaction_t *tr)
{
    if(tr->txbuf == NULL && tr->rxbuf == NULL) return false;
    if(tr->length == 0) return false;
    
	/*
	 * TODO: Put here the corresponding SPI transmission/reception function.
	 */
	return (SPI1_TransmitReceive(tr->txbuf,tr->rxbuf,tr->length));

	// Uncomment this line if the previous function doesn't return any value.
	//return true;
}

/**
 * Executes the queued transaction over SPI
 */
void TransactionTasks()
{
    switch(this.trState)
    {
        default:
        case TRSTATE_WAIT:
            // try to pull a transaction
            if(trQueue_Pull(&this.t))
            {
            	DEBUG_PRINTF("Performing all the transactions registered in the queue...");

            	// Assert the CS line
            	if(this.t->assertCS != NULL) this.t->assertCS();

            	this.t->status = SPI_TRANSACTION_STATUS_WORKING;

            	// Try to start the SPI transaction
            	if(trStart(this.t))
            	{
            		this.trState = TRSTATE_PERFORMING;
            	}
            	else
            	{
            		// An error occured release the CS line
            		if(this.t->releaseCS != NULL) this.t->releaseCS();
            		this.t->status = SPI_TRANSACTION_STATUS_FAILED;
            		if(this.t->callback != NULL) this.t->callback(this.t);
            	}

            }
            break;

        case TRSTATE_PERFORMING:
            if(this.t->status != SPI_TRANSACTION_STATUS_WORKING)
            {
            	if(trQueue_Pull(&this.t))
            	{
            		// Assert the CS line
            		if(this.t->assertCS != NULL) this.t->assertCS();
            		this.t->status = SPI_TRANSACTION_STATUS_WORKING;

            		// Try to start the SPI transaction
            		if(!trStart(this.t))
					{
						// An error occurred, release the CS line
						if(this.t->releaseCS != NULL) this.t->releaseCS();
						this.t->status = SPI_TRANSACTION_STATUS_FAILED;
						if(this.t->callback != NULL) this.t->callback(this.t);
					}
            	}
            	else
            	{
            		if(this.callback != NULL) this.callback();
            		DEBUG_PRINTF("Transactions finished");
            		this.trState = TRSTATE_WAIT;
            	}
            }
            break;
    }
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
/**
 * @brief SPI callback.
 *
 * @note User can register a callback function to be called from the SPI plib
 *
 */
static void SPI_transaction_callback()
{
	if(this.t->releaseCS != NULL) this.t->releaseCS();

	this.t->status = SPI_TRANSACTION_STATUS_FINISHED;
	if(this.t->callback != NULL) this.t->callback(this.t);
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void SPI_Init()
{
    int i;

    // Init the queue
    this.in_idx = 0;
    this.out_idx = 0;
    this.queue_full = false;
    this.callback = NULL;
    this.trState = TRSTATE_WAIT;
    SPI1_RegisterCallback_TxRx(SPI_transaction_callback);

    // Initialize the queue elements
    for(i = 0; i < TRANSACTION_QUEUE_SIZE; i++)
    {
        this.tra_queue[i] = NULL;
    }
}

void SPI_Tasks()
{
    TransactionTasks();
}

bool SPI_isDone()
{
    return (trQueue_isEmpty() && this.trState == TRSTATE_WAIT);
}

bool SPI_AddTransaction(spi_transaction_t *t)
{
    return trQueue_Push(t);
}

bool SPI_CreateTransactionAdd(spi_transaction_t *t, 
                            uint8_t *tx, 
                            uint8_t *rx, 
                            uint32_t size, 
                            spi_assertCS_t assertCS,
                            spi_releaseCS_t releaseCS,
							spi_transaction_callback_t clbck)
{
	// Charge all the parameters in the transaction
    t->assertCS = assertCS;
    t->releaseCS = releaseCS;
    t->rxbuf = rx;
    t->txbuf = tx;
    t->length = size;
    t->status = SPI_TRANSACTION_STATUS_IDLE;
    if(clbck != NULL) t->callback = clbck;

    return trQueue_Push(t);
}

void SPI_SetGeneralCallback(spi_general_callback_t func)
{
    this.callback = func;
}

spi_transaction_status_t SPI_GetTransactionStatus(spi_transaction_t *t)
{
    return t->status;
}
