/***************************************************************************//**
 * @file        queue.h
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
#ifndef QUEUE_H_
#define QUEUE_H_

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
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * @brief Creates a new ring buffer
 * 
 * @param r pointer to a queue buffer structure
 * @param buf pointer to the buffer that will be used as a ring buffer
 * @param len length of the buffer in elements
 * @param elem_size size in Bytes of each buffer element
 * 
 * @return true succeeded
 * @return false failed
 */
bool QUEUE_Create(queue_t *r, void *buf, uint32_t len, size_t elem_size);

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

#endif /* QUEUE_H_ */
