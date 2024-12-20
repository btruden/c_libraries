/*******************************************************************************
 * @file        queue.c
 * @brief       This file provides a generic queue functionality.
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
#include "queue.h"
#include "debug.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        false
#define DEBUG_TAG           "QUEUE"

#if ENABLE_DEBUG == true
#define DEBUG_PRINTF(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#define PRINTF(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(format, ...)
#define PRINTF(format, ...)
#define PRINT(format, ...)
#endif


/******************************************************************************
 * Local Types
 ******************************************************************************/

/******************************************************************************
 * Local Variables
 ******************************************************************************/

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Returns if the queue is empty
 * 
 * @param r pointer to the queue to evaluate
 */
bool isEmpty(queue_t *r)
{
    return (r->in_idx == r->out_idx && !r->queue_full);
}

/**
 * Returns if the queue is full
 * 
 * @param r pointer to the queue to evaluate
 */
bool isFull(queue_t *r)
{
    return r->queue_full;
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
bool QUEUE_Create(queue_t *r, void *buf, uint32_t len, size_t elem_size)
{
	if(r == NULL) return false;
	if(buf == NULL) return false;
	if(!len) return false;
	if(!elem_size) return false;

	r->obj_buf = buf;
	r->buf_len = len;
	r->in_idx = 0;
	r->out_idx = 0;
	r->queue_full = false;
	r->element_size = elem_size;

	return true;
}

bool QUEUE_Pull(queue_t *r, void *dst)
{
	if(r == NULL) return false;
	if(dst == NULL) return false;
    if(isEmpty(r)) return false;

	memcpy(dst,r->obj_buf+(r->out_idx*r->element_size),r->element_size);
    r->out_idx = (r->out_idx + 1) % r->buf_len;
    r->queue_full = false;

    return true;
}

bool QUEUE_Push(queue_t *r, void *src)
{
	if(r == NULL) return false;
	if(src == NULL) return false;
    if(isFull(r)) return false;

	memcpy(r->obj_buf+(r->in_idx*r->element_size),src,r->element_size);
    r->in_idx = (r->in_idx + 1) % r->buf_len;
    if(r->in_idx == r->out_idx) r->queue_full = true;

    return true;
}