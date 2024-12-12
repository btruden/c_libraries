/*******************************************************************************
 * @file        debug.c
 * @brief       This file uses a UART to open a communication channel intended
 *              to be used for debugging purposes using a terminal on the PC.
 * @author      Blas Truden
 * @date        20241210
 * @version     v1
 * 
 * @copyright   -
 * 
 * @note		This module is part of the BSI BSP core
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/
#include "debug.h"
#include "definitions.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/

#define TX_BUF_SIZE             32
#define MSG_SIZE_MAX            128

#if RX_STREAM_ENABLE == true
    #define RX_BUF_SIZE         16
#endif

/******************************************************************************
 * Local Types
 ******************************************************************************/

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
	// TX variables
	uint8_t tx_buf[TX_BUF_SIZE][MSG_SIZE_MAX];
	uint32_t TXinIdx;
	uint32_t TXoutIdx;
	bool TXfull;

#if RX_STREAM_ENABLE == true
	// RX variables
	uint8_t rx_buf[RX_BUF_SIZE];
	uint32_t RXinIdx;
	uint32_t RXoutIdx;
	bool RXfull;
#endif
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/

#if RX_STREAM_ENABLE == true
/**
 * @brief Returns if the RX buffer is empty
 *
 * @return true
 * @return false
 */
bool RXempty()
{
    return (!this.RXfull && this.RXinIdx == this.RXoutIdx);
}

/**
 * @brief Increment the input index in a circular way
 *
 */
static void RXInIdxUpdate()
{
	this.RXinIdx = (this.RXinIdx+1) % RX_BUF_SIZE;
    if(this.RXinIdx == this.RXoutIdx) this.RXfull = true;
}

/**
 * @brief Increment the output index in a circular way
 *
 */
static void RXOutIdxUpdate()
{
	this.RXoutIdx = (this.RXoutIdx+1) % RX_BUF_SIZE;
	this.RXfull = false;
}

/**
 * @brief Returns the free spaces in the RX ring buffer
 *
 * @return size_t
 */
static size_t RXFreeSpace()
{
    if(this.RXfull) return 0;
    if(this.RXoutIdx == this.RXinIdx) return RX_BUF_SIZE;

    if(this.RXoutIdx > this.RXinIdx)
    {
        return this.RXoutIdx - this.RXinIdx;
    }
    else
    {
        return (RX_BUF_SIZE-this.RXinIdx) + this.RXoutIdx;
    }
}

static size_t RXUsedSpace()
{
    if(RXempty()) return 0;
    if(this.RXfull) return RX_BUF_SIZE;
    if(this.RXinIdx > this.RXoutIdx)
    {
        return this.RXinIdx - this.RXoutIdx;
    }
    else
    {
        return (RX_BUF_SIZE-this.RXoutIdx) + this.RXinIdx;
    }
}

/**
 * @brief Puts a bunch of bytes into the RX ring buffer
 *
 * @param msg bunch of bytes to be pushed
 * @param len qty of bytes to push
 * @return true suceed
 * @return false ring buffer full
 */
static bool RXPushMsg(uint8_t *msg, size_t len)
{
    uint32_t cnt = 0;

    if(len > RXFreeSpace()) return false;

    for(cnt = 0; cnt < len; cnt++)
    {
    	this.rx_buf[this.RXinIdx] = *(msg+cnt);
        RXInIdxUpdate();
    }

    return true;
}

/**
 * @brief Pulls a bunch of bytes from the RX ring buffer
 *
 * @param msg destination buffer
 * @param len qty of bytes to pull
 * @return true suceed
 * @return false ring buffer empty
 */
static bool RXPullMsg(uint8_t *msg, size_t len)
{
    uint32_t cnt = 0;

    if(len > RXUsedSpace()) return false;

    for(cnt = 0; cnt < len; cnt++)
    {
        *(msg+cnt) = this.rx_buf[this.RXoutIdx];
        RXOutIdxUpdate();
    }

    return true;
}
#endif

/**
 * @brief Increment the input index in a circular way
 *
 */
static void TXInIdxUpdate()
{
	this.TXinIdx = (this.TXinIdx+1) % TX_BUF_SIZE;
    if(this.TXinIdx == this.TXoutIdx) this.TXfull = true;
}

/**
 * @brief Increment the output index in a circular way
 *
 */
static void TXOutIdxUpdate()
{
	this.TXoutIdx = (this.TXoutIdx+1) % TX_BUF_SIZE;
	this.TXfull = false;
}

/**
 * @brief puts a message in the ring buffer
 *
 * @param msg string to be inserted in the buffer
 * @return true ok
 * @return false buffer TXfull
 */
static bool TXPushMsg(uint8_t *msg)
{
    if(this.TXfull) return false;

    this.tx_buf[this.TXinIdx][0] = '\0';
    strcpy((char *)this.tx_buf[this.TXinIdx],(char *)msg);
    TXInIdxUpdate();

    return true;
}

/**
 * @brief Gets a string from the buffer
 *
 * @param msg string pointer were to copy the taken string from the buff
 * @return true ok
 * @return false buffer empty
 */
static bool TXPullMsg(uint8_t *msg)
{
    if(!this.TXfull && this.TXinIdx == this.TXoutIdx) return false;

    strcpy((char *)msg,(char *)this.tx_buf[this.TXoutIdx]);
    TXOutIdxUpdate();

    return true;
}

/**
 * @brief UART transmission task
 *
 */
static void TX_task()
{
    static enum
    {
        TXSTATE_IDLE,
		TXSTATE_RETRY,
        TXSTATE_SENDING
    }txState = TXSTATE_IDLE;

    static uint8_t line[MSG_SIZE_MAX];
    static uint32_t line_size;

    switch(txState)
    {
        default:
        case TXSTATE_IDLE:
            if(TXPullMsg(line))
            {
                line_size = strlen((char *)line)+1;
                if(SERCOM1_USART_Write(line,line_size))
                {
                	txState = TXSTATE_SENDING;
                }
                else
                {
                	txState = TXSTATE_RETRY;
                }
            }
            break;

        case TXSTATE_RETRY:
        	if(SERCOM1_USART_Write(line,line_size))
			{
				txState = TXSTATE_SENDING;
			}
        	break;

        case TXSTATE_SENDING:
        	if(SERCOM1_USART_TransmitComplete())
        	{
				txState = TXSTATE_IDLE;
        	}
            break;
    }
}

#if RX_STREAM_ENABLE == true
/**
 * @brief UART reception task
 *
 */
static void RX_task()
{
    static uint32_t bytes_to_read = 0;
    static uint32_t bytes_read = 0;
    static uint8_t buf[128];

    static enum
    {
        RXSTATE_IDLE,
        RXSTATE_READING
    }RXState = RXSTATE_IDLE;

    switch(RXState)
    {
        default:
        case RXSTATE_IDLE:
            bytes_to_read = UART1_ReadCountGet();
            if(bytes_to_read)
            {
                bytes_read = 0;
                RXState = RXSTATE_READING;
            }
            break;

        case RXSTATE_READING:
            bytes_read += UART1_Read(&buf[bytes_read],bytes_to_read - bytes_read);
            if(bytes_read == bytes_to_read)
            {
                RXPushMsg(buf,bytes_to_read);
                bytes_to_read = 0;
                RXState = RXSTATE_IDLE;
            }
            break;
    }
}
#endif

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void DEBUG_init()
{
	this.TXinIdx = 0;
	this.TXoutIdx = 0;
	this.TXfull = false;

#if RX_STREAM_ENABLE == true
	this.RXinIdx = 0;
	this.RXoutIdx = 0;
	this.RXfull = false;
#endif
}

void DEBUG_tasks()
{
    TX_task();
#if RX_STREAM_ENABLE == true
    RX_task();
#endif
}

void DEBUG_print(char *tag, bool carryRet, char *format, ...)
{
    char buff[MSG_SIZE_MAX*2];
    char msg[MSG_SIZE_MAX];

    va_list args;
    va_start (args, format);

    // Print original msg:
    vsnprintf(msg, MSG_SIZE_MAX, format, args);

    // Add debug info:
    if(tag != NULL)
    {
        sprintf(buff, "%s: %s", tag, msg);
    }
    else
    {
        sprintf(buff, "%s", msg);
    }

    // Add carry return
    if(carryRet && ((strlen(buff)-1) < MSG_SIZE_MAX))
        strcat(buff,"\n");

    TXPushMsg((uint8_t *)buff);

    return;
}

uint8_t DEBUG_getchar()
{
    return (uint8_t)SERCOM1_USART_ReadByte();
}

#if RX_STREAM_ENABLE == true
void DEBUG_ClearRX()
{
	this.RXinIdx = 0;
	this.RXoutIdx = 0;
	this.RXfull = false;
}
#endif