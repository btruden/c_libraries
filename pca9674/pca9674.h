/***************************************************************************//**
 * @file        pca9674.h
 * @brief       Provides the necessary interface elements for a successful use 
 *              of the functionality.
 * @author      Blas Truden
 * @date        20241220
 * @version     v1
 * 
 * @copyright   -
 * 
 * @details     This module is part of the BSI BSP core.
 ******************************************************************************/
#ifndef PCA9674_H_
#define PCA9674_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Public Constants & defines
 ******************************************************************************/
// Port direction configuration
#define PCA9674_PORT_OUTPUT     0x00
#define PCA9674_PORT_INPUT      0x01

// Port state configuration
#define PCA9674_PORT_LOW        0x00
#define PCA9674_PORT_HIGH       0x01

/******************************************************************************
 * Public Types
 ******************************************************************************/
/**
 * PCA9674 handle type
 */
typedef int32_t PCA9674_handle_t;

/**
 * PCA9674 status type
 */
typedef enum
{
    PCA9674_STATUS_ERROR = -1,
    PCA9674_STATUS_OK,
    PCA9674_STATUS_PROCESSING,
}PCA9674_status_t;

/**
 * PCA9674 read callback function type. When called, it'll give the port state
 * in the argument.
 * 
 * @param handle    Handle to the device instance that executed the callback
 * @param port      Port state read from the device
 */
typedef void (*PCA9674_read_callback_t)(PCA9674_handle_t handle, uint8_t port);

/**
 * PCA9674 device configuration structure type
 */
typedef struct 
{
    uint8_t i2c_address;            // I2C address of the device
    uint8_t port_cfg;               // 8-bit configuration of the port. 0 = output, 1 = input
    uint8_t port_initial_state;     // 8-bit initial state of the port. 0 = low, 1 = high
    PCA9674_read_callback_t read_cb;// Callback function to call when a port read is performed
}PCA9674_config_t;

/******************************************************************************
 * Callback Functions
 ******************************************************************************/

/******************************************************************************
 * Public Functions
 ******************************************************************************/

/**
 * Module initialization. Must be called once during startup.
 */
void PCA9674_init();

/**
 * Module tasks. Must be called periodically.
 */
void PCA9674_tasks();

/**
 * Opens a PCA9674 device instance with the given configuration.
 * 
 * @param config    Configuration of the device
 * @return          Handle to the device instance
 */
PCA9674_handle_t PCA9674_open(PCA9674_config_t config);

/**
 * Sets the state of a bit in the port of the PCA9674 device.
 * 
 * @param handle    Handle to the device instance
 * @param bit       Bit to set
 * @param state     State to set the bit to
 * @return          True if the operation was successful, false otherwise
 */
bool PCA9674_set_port_bit(PCA9674_handle_t handle, uint8_t bit, uint8_t state);

/**
 * Sets the state of the port of the PCA9674 device.
 * 
 * @param handle    Handle to the device instance
 * @param data      Data to set the port to
 * @return          True if the operation was successful, false otherwise
 */
bool PCA9674_set_port_byte(PCA9674_handle_t handle, uint8_t data);

/**
 * Returns the last Byte state programmed to or read from PCA9674 device.
 * 
 * This read function doesn't perform an I2C transaction, it just returns the
 * last state registered in the driver's RAM, which occurs after a write or a 
 * read transaction.
 * 
 * This function is useful when the port is configured as outputs and there are
 * no inputs that may change from an external source.
 * 
 * @param handle    Handle to the device instance
 * @return          State of the port
 */
uint8_t PCA9674_static_read_port_byte(PCA9674_handle_t handle);

/**
 * Returns the last bit state programmed to or read from PCA9674 device.
 * 
 * This read function doesn't perform an I2C transaction, it just returns the
 * last state registered in the driver's RAM, which occurs after a write or a 
 * read transaction.
 * 
 * This function is useful when the port is configured as outputs and there are
 * no inputs that may change from an external source.
 * 
 * @param handle    Handle to the device instance
 * @param bit       Bit to read
 * @return          State of the bit
 */
bool PCA9674_static_read_bit(PCA9674_handle_t handle, uint8_t bit);

/**
 * Starts an I2C operation for reading the port state of the PCA9674 device.
 * 
 * When te read is complete, the callback function will be called with the port
 * information in its argument. If the callback was not registered in the 
 * configuration, the user can poll the function PCA9674_is_ready().
 */
bool PCA9674_read(PCA9674_handle_t handle);

/**
 * Returns "true" if the device instance is ready to perform an I2C transaction.
 */
bool PCA9674_is_ready(PCA9674_handle_t handle);

#endif /* PCA9674_H_ */
