/*
 * ringbuf.c
 *
 *  Created on: Sep 13, 2021
 *      Author: btrud
 */


/******************************************************************************
 * Includes
 ******************************************************************************/
#include <queue.h>

/******************************************************************************
 * Local Constants
 ******************************************************************************/

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
bool QUEUE_Create(queue_t *r, void *buf, size_t len, size_t elem_size)
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