#ifndef MCU_PROTOCOL_H_
#define MCU_PROTOCOL_H_

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*******************************************************************************
 *                          PROTOCOL DOCUMENTATION
 * 
 * This protocol was designed for the communcation between two MCUs. It is 
 * abstracted from the physical layer, so it can be used with any communication
 * interface (UART, SPI, I2C, etc).
 * 
 * FRAME STRUCTURE:
 * The protocol is based on a frame structure. Each frame is composed of the
 * following fields:
 * 
 * <SOF>msgID,payload,CRC16<EOF>
 * 
 * Where:
 * - <SOF>      is the start of frame character. It is used to identify the 
 *              beginning of a frame.
 * - msgID      is the message ID. It is a string that identifies the message 
 *              type.
 * - payload    is the message payload. It is a list of parameters separated by
 *              a delimiter.
 * - CRC16      is the CRC16 of the frame. It is used to check the frame
 *              integrity.
 * - <EOF>      is the end of frame character. It is used to identify the end of
 *              a frame.
 * 
 * FRAME TYPES:
 * The protocol supports 3 types of frames:
 * - Request:       The frame is a request from the sender to the receiver. The
 *                  receiver must reply to the request.
 * - Request reply: The frame is a reply to a request. It is sent by the
 *                  receiver to the sender.
 * - Notification:  The frame is a notification. It is sent by the sender to the
 *                  receiver. No reply is expected.
 * 
 * It's important to mention that no master and slave are defined in this
 * protocol. Both devices can send requests, request replies and notifications.
 * However, the application can define a master and a slave by defining the
 * message IDs that each device can send and receive.
 * 
 * USAGE:
 * 1. Initialize the module by calling MCU_PROTOCOL_init() once during startup.
 * 2. Create a configuration structure (MCU_PROTOCOL_config_t) and fill it with
 *   the desired configuration.
 * 3. Open a protocol instance by calling MCU_PROTOCOL_open() and passing the
 *  configuration structure.
 * 4. Push bytes into the protocol driver by calling MCU_PROTOCOL_push_byte()
 * whenever a byte is received from the communication interface.
 * 5. Call MCU_PROTOCOL_tasks() periodically to process the received frames.
 * 6. Call MCU_PROTOCOL_tick_1ms() every 1ms.
 * 7. Assemble a request, request reply or notification by calling
 * MCU_PROTOCOL_assemble_request(), MCU_PROTOCOL_assemble_request_reply() or
 * MCU_PROTOCOL_assemble_notification(), respectively.
 * 
 * CALLBACKS:
 * The protocol supports 3 types of callbacks:
 * - Request callback: Called whenever a request is received.
 * - Request reply callback: Called whenever a request reply is received.
 * - Notification callback: Called whenever a notification is received.
 * 
 * ERRORS:
 * The protocol generates errors whenever a frame is received with an invalid
 * format or when an error occurs during the frame processing.
 *
 ******************************************************************************/

/******************************************************************************
 * Public Constants
 ******************************************************************************/
// Instances definitions. If only 1 instance will be used, it's recommended to
// set the maximum instances quantity to 1 to save memory.
#define MCU_PROTOCOL_MAX_INSTANCES_QTY          1       // Up to 255

#if MCU_PROTOCOL_MAX_INSTANCES_QTY > 1
    #warning "If only one instance of MCU_PROTOCOL will be used, it's recommended to set MCU_PROTOCOL_MAX_INSTANCES_QTY to 1 to save memory."
#endif

// Frame character definitions
#define MCU_PROTOCOL_SOF_REQUEST                '$'
#define MCU_PROTOCOL_SOF_REQUEST_REPLY          '%'
#define MCU_PROTOCOL_SOF_NOTIFICATION           '!'
#define MCU_PROTOCOL_DELIMITER                  ','
#define MCU_PROTOCOL_PAYLOAD_DELIMITER          ';'
#define MCU_PROTOCOL_EOF                        '\n'

// Sizes and lengths
#define MCU_PROTOCOL_MAX_MSG_ID_LENGTH          32
#define MCU_PROTOCOL_MAX_PAYLOAD_LENGTH         128
#define MCU_PROTOCOL_CRC_LENGTH                 4 // Length of CRC16 in HEX format
#define MCU_PROTOCOL_MAX_PARAMS_LENGTH          32
#define MCU_PROTOCOL_MAX_PARAMS_QTY             16

#define MCU_PROTOCOL_MAX_FRAME_LENGTH (1 + MCU_PROTOCOL_MAX_MSG_ID_LENGTH + 1 + MCU_PROTOCOL_MAX_PAYLOAD_LENGTH + 1 + MCU_PROTOCOL_CRC_LENGTH + 1) // SOF + msgID + , + payload + , + CRC + EOF


// Maximum number of message ID supported
#define MCU_PROTOCOL_MAX_MSG_ID_QTY             32      // up to 255

// Receive byte queue definitions
#define MCU_PROTOCOL_RX_QUEUE_SIZE              256

/******************************************************************************
 * Public Types
 ******************************************************************************/

/**
 * Handle used to reference a protocol object
 */
typedef int16_t MCU_PROTOCOL_handle_t;

/**
 * Protocol message ID list type used for confugring the supported messages by
 * the instance
 */
typedef struct
{
    uint8_t msg_ids[MCU_PROTOCOL_MAX_MSG_ID_QTY][MCU_PROTOCOL_MAX_MSG_ID_LENGTH];
    uint8_t msg_id_qty;
}MCU_PROTOCOL_msg_id_list_t;

/**
 * An easy to use structure containing all the parameters of the payload in a 
 * single list. The parameters are in string format.
 */
typedef struct
{
    uint8_t params[MCU_PROTOCOL_MAX_PARAMS_QTY][MCU_PROTOCOL_MAX_PARAMS_LENGTH];
    uint8_t params_qty;
}MCU_PROTOCOL_payload_list_t;

/**
 * If registered, this function is called whenever a supported message ID 
 * corresponding to a request is received 
 */
typedef void (*MCU_PROTOCOL_request_callback_t)(uint8_t msg_id_idx, MCU_PROTOCOL_payload_list_t payload);

/**
 * If registered, this function is called whenever a supported message ID 
 * corresponding to a request reply is received 
 */
typedef void (*MCU_PROTOCOL_request_reply_callback_t)(uint8_t msg_id_idx, MCU_PROTOCOL_payload_list_t payload);

/**
 * If registered, this function is called whenever a supported message ID 
 * corresponding to a notification is received 
 */
typedef void (*MCU_PROTOCOL_notification_callback_t)(uint8_t msg_id_idx, MCU_PROTOCOL_payload_list_t payload);

/**
 * Protocol error code
 */
typedef enum
{
    MCU_PROTOCOL_ERROR_OK = 0,

    // Reception errors
    MCU_PROTOCOL_RX_ERROR_TIMOUT,
    MCU_PROTOCOL_RX_ERROR_CRC,
    MCU_PROTOCOL_RX_ERROR_INVALID_MSG_ID,
    MCU_PROTOCOL_RX_ERROR_FRAME_SIZE,
    MCU_PROTOCOL_RX_ERROR_TOO_FEW_FIELDS,

    // QUEUE errors
    MCU_PROTOCOL_RX_QUEUE_FULL,

    // Transmission errors
    MCU_PROTOCOL_TX_ERROR_INVALID_HANDLE,
    MCU_PROTOCOL_TX_ERROR_INVALID_MSG_ID_IDX,
    MCU_PROTOCOL_TX_ERROR_INVALID_FRAME_SIZE,
}MCU_PROTOCOL_error_code_t;

/**
 * Error callback function. This function is called whenever an error is 
 * detected in the protocol
 */
typedef void (*MCU_PROTOCOL_error_callback_t)(MCU_PROTOCOL_error_code_t e);

/**
 * Protocol configuration structure
 */
typedef struct
{
    MCU_PROTOCOL_msg_id_list_t msg_id_list; // List of supported msg IDs
                                            // This list includes requests, 
                                            // request replies and notifications

    uint32_t receive_timeout_ms;                // Timeout between btyes when
                                                // receiving a frame

    MCU_PROTOCOL_request_callback_t rq_cb;          // Request callback
    MCU_PROTOCOL_request_reply_callback_t rq_r_cb;  // Request reply callback
    MCU_PROTOCOL_notification_callback_t n_cb;      // Notification callback
    MCU_PROTOCOL_error_callback_t e_cb;             // Error callback
}MCU_PROTOCOL_config_t;

/******************************************************************************
 * Public Functions
 ******************************************************************************/
/**
 * Module initialization. Must be called once during startup.
 */
void MCU_PROTOCOL_init();

/**
 * Module tasks. Must be called periodically.
 */
void MCU_PROTOCOL_tasks();

/**
 * Module 1ms tick. Must be called every 1ms.
 */
void MCU_PROTOCOL_tick_1ms();

/**
 * Open a protocol instance
 * 
 * @param config Configuration structure
 * 
 * @return Handle to the protocol instance or -1 if an error occurred
 */
MCU_PROTOCOL_handle_t MCU_PROTOCOL_open(MCU_PROTOCOL_config_t *config);

/**
 * Pushes a byte into the protocol driver. This function must be called whenever
 * a byte is received from the communication interface.
 * 
 * @param handle Protocol instance handle
 * @param byte Byte to push
 * 
 * @return Error code
 */
MCU_PROTOCOL_error_code_t MCU_PROTOCOL_push_byte(MCU_PROTOCOL_handle_t handle, 
                                                uint8_t byte);

/**
 * Assembles a request message to the communication interface. After sending the 
 * request, the protocol will wait for a reply. If the reply is not received
 * within the timeout, an error will be generated.
 * 
 * A request reply callback function must be registered in the configuration
 * structure.
 * 
 * @param handle Protocol instance handle
 * @param msg_id_idx Index of the message ID in the list
 * @param payload Payload to send
 * @param dst Destination buffer. Must be >= MCU_PROTOCOL_MAX_FRAME_LENGTH long.
 * 
 * @return Error code
 */
MCU_PROTOCOL_error_code_t MCU_PROTOCOL_assemble_request(
                                        MCU_PROTOCOL_handle_t handle, 
                                        uint8_t msg_id_idx, 
                                        MCU_PROTOCOL_payload_list_t payload,
                                        uint8_t *dst);

 /**
 * Assembles a request reply message to the communication interface. After 
 * sending the request, the protocol will wait for a reply. If the reply is not 
 * received within the timeout, an error will be generated.
 * 
 * A request reply callback function must be registered in the configuration
 * structure.
 * 
 * @param handle Protocol instance handle
 * @param msg_id_idx Index of the message ID in the list
 * @param payload Payload to send
 * @param dst Destination buffer. Must be >= MCU_PROTOCOL_MAX_FRAME_LENGTH long.
 * 
 * @return Error code
 */
MCU_PROTOCOL_error_code_t MCU_PROTOCOL_assemble_request_reply(
    MCU_PROTOCOL_handle_t handle, 
    uint8_t msg_id_idx, 
    MCU_PROTOCOL_payload_list_t payload,
    uint8_t *dst);

/**
 * Assembles a notification message to the communication interface. 
 * Notifications do not require a reply.
 * 
 * A notification callback function must be registered in the configuration
 * structure.
 * 
 * @param handle Protocol instance handle
 * @param msg_id_idx Index of the message ID in the list
 * @param payload Payload to send
 * @param dst Destination buffer. Must be >= MCU_PROTOCOL_MAX_FRAME_LENGTH long.
 * 
 * @return Error code
 */
MCU_PROTOCOL_error_code_t MCU_PROTOCOL_assemble_notification(
                                    MCU_PROTOCOL_handle_t handle, 
                                    uint8_t msg_id_idx, 
                                    MCU_PROTOCOL_payload_list_t payload,
                                    uint8_t *dst);

#endif /* MCU_PROTOCOL_H_ */
