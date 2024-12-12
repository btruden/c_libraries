/*
 * ringbuf.h
 *
 *  Created on: Sep 13, 2021
 *      Author: btrud
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/******************************************************************************
 * Public Constants
 ******************************************************************************/
// Maximum amount of elements that the queue can store
#define QUEUE_MAX_ELEMENTS        64

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * Ring buffer structure type
 */
typedef struct
{
    // Pointer to the buffer given by the user for applying the ring feature
    void *obj_buf;

    // Buffer elements qty
    size_t buf_len;

    // size of each buffer element in bytes
    size_t element_size;

    // queue functionality variables
    uint32_t in_idx;
    uint32_t out_idx;
    bool queue_full;

}queue_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * @brief Creates a new ring buffer
 * 
 * @param r pointer to a queue buffer structure
 * @param buf pointer to the buffer that this library will use for the queue
 * @param len size in bytes of the given buffer
 * @param elem_size size in Bytes of each buffer element
 * 
 * @return true succeeded
 * @return false failed
 */
bool QUEUE_Create(queue_t *r, void *buf, size_t len, size_t elem_size);

/**
 * @brief Pulls an element from the queue
 * 
 * @param r pointer to the queue to evaluate
 * @param dst pointer to the destination where the element will be copied
 * @return true succeeded
 * @return false failed
 */
bool QUEUE_Pull(queue_t *r, void *dst);

/**
 * @brief Pushes an element into the queue
 * 
 * @param r pointer to the queue to evaluate
 * @param src pointer to the source element from where copy the value to the buffer
 * @return true succeeded
 * @return false failed
 */
bool QUEUE_Push(queue_t *r, void *src);


#endif /* INC_QUEUE_H_ */
