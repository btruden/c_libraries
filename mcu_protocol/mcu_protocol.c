/******************************************************************************
 * Includes
 ******************************************************************************/
#include "mcu_protocol.h"
#include "debug.h"
#include "queue.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "MCU_PROTOCOL"

#if ENABLE_DEBUG == true
#define DEBUG_MSG(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

#define PRINT_LINE(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)

/**
 * CRC16 lookup table for high–order byte
 */
static const unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
} ;
    
/**
 * CRC16 lookup table for low–order byte
 */
static const unsigned char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states type
 */
typedef enum
{
    STATE_IDLE = 0,
    STATE_RECEIVING_MSG,
}state_t;

typedef enum
{
    MSG_TYPE_REQUEST = 0,
    MSG_TYPE_REQUEST_REPLY,
    MSG_TYPE_NOTIFICATION
}msg_type_t;

/**
 * Complete message type structure containing all the information about the
 * message
 */
typedef struct
{
    msg_type_t type;
    int16_t msg_id_idx;
    MCU_PROTOCOL_payload_list_t payload;
    uint16_t crc16;
    uint16_t crc16_calc;
    bool too_few_fields;
}msg_t;

/**
 * Instance data structure
 */
typedef struct
{
    state_t state;
    uint32_t tmr;
    
    // Protocol configuration
    MCU_PROTOCOL_config_t config;

    // Reception queue
    queue_t rx_queue;
    uint8_t rx_buf[MCU_PROTOCOL_RX_QUEUE_SIZE];

    // Receive message variables
    msg_t msg;
    uint8_t msg_buf[MCU_PROTOCOL_MAX_FRAME_LENGTH*2];
    uint16_t msg_buf_len;
}instance_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct
{
    // Array of instances data
    instance_t instances[MCU_PROTOCOL_MAX_INSTANCES_QTY];

    // Amount of instances registered
    uint8_t instance_idx;
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Calculates the CRC16 of the given message
 * 
 * @param msg   The message on which to calculate the CRC
 * @param len   Size of the message
 * 
 * @return The CRC16 calculated value
 */
uint16_t get_crc16(uint8_t *msg, size_t len)
{
    uint8_t ucCRCHi = 0xFF;
    uint8_t ucCRCLo = 0xFF;
    int32_t idx;

    while( len-- )
    {
        idx = (0x00FF) & ( ucCRCLo ^ *( msg++ ));
        ucCRCLo = ( uint8_t  )( (0x00FF) & ( ucCRCHi ^ auchCRCHi[idx] ));
        ucCRCHi = auchCRCLo[idx];
    }

    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}

 /**
  * Get the frame length
  * 
  * Frame format:
  * <SOF>msgID,payload,CRC16<EOF>
  * 
  * @param handle Instance handle
  * @param msg_id_idx Message ID index
  * @param payload Payload list
  * 
  * @return Frame length
  */
static uint32_t get_frame_length(MCU_PROTOCOL_handle_t handle,
                                uint8_t msg_id_idx, 
                                MCU_PROTOCOL_payload_list_t payload)
{
    uint32_t frame_size = 0;

    // Add 1 byte for the start of frame
    frame_size++; 

    // Add the message ID length
    frame_size += strlen((char *)this.instances[handle].config.msg_id_list.msg_ids[msg_id_idx]);

    // Add 1 byte for the delimiter between msgID and payload
    frame_size++;

    // Sum the payload length
    for(uint8_t i = 0; i < payload.params_qty; i++)
    {
        frame_size += strlen((char *)payload.params[i]);
        frame_size ++; // Add 1 byte for the parameter separator
    }
    
    // the last parameter separator is counted as the delimiter between the 
    // payload and the CRC16.

    // Sum the CRC16 length
    frame_size += MCU_PROTOCOL_CRC_LENGTH;

    frame_size++; // Add 1 byte for the end of frame

    return frame_size;
}

/**
 * Assemble the message
 * 
 * @param sof Start of frame
 * @param handle Instance handle
 * @param msg_id_idx Message ID index
 * @param payload Payload list
 * @param dst Destination buffer
 */
static void assemble_msg(uint8_t sof, 
                        MCU_PROTOCOL_handle_t handle, 
                        uint8_t msg_id_idx, 
                        MCU_PROTOCOL_payload_list_t payload,
                        uint8_t *dst)
{
    uint8_t sof_str[2] = {'\0', '\0'};
    uint8_t delim_str[2] = {MCU_PROTOCOL_DELIMITER, '\0'};
    uint8_t payload_delim_str[2] = {MCU_PROTOCOL_PAYLOAD_DELIMITER, '\0'};
    uint8_t crc16_str[8];
    uint16_t crc16;
    uint8_t eof_str[2] = {MCU_PROTOCOL_EOF, '\0'};
    
    // Prepare the start of frame
    sof_str[0] = sof;

    // Add the start of frame
    strcpy((char *)dst, (char *)sof_str);

    // Add the message ID
    strcat((char *)dst, (char *)this.instances[handle].config.msg_id_list.msg_ids[msg_id_idx]);

    // Add the delimiter between the msgID and the payload
    strcat((char *)dst, (char *)delim_str);
    
    // If exists, add the payload
    if(payload.params_qty > 0)
    {
        for(uint8_t i = 0; i < payload.params_qty; i++)
        {
            // Add the parameter
            strcat((char *)dst, (char *)payload.params[i]);

            // If it's not the last parameter, add the parameter delimiter
            if(i < payload.params_qty - 1)
            {
                strcat((char *)dst, (char *)payload_delim_str);
            }
        }

        // Add the delimiter between the payload and the CRC16
        strcat((char *)dst, (char *)delim_str);
    }

    // Add the CRC16
    crc16 = get_crc16(&dst[1], strlen((char *)dst));
    sprintf((char *)crc16_str, "%04X", crc16);
    strcat((char *)dst, (char *)crc16_str);

    // Add the end of frame
    strcat((char *)dst, (char *)eof_str);
}

/**
 * Parses the received message and returns a msg_t structure.
 * 
 * Frame format:
 * <SOF>msgID,payload,CRC16<EOF>
 * 
 * @param msg_buf Buffer containing the message
 * @param msg_buf_len Length of the message buffer
 */
static msg_t parse_msg(uint8_t *msg_buf, uint16_t msg_buf_len)
{
    msg_t m = {0};
    char msg_id_str[MCU_PROTOCOL_MAX_FRAME_LENGTH];
    char payload_str[MCU_PROTOCOL_MAX_FRAME_LENGTH];
    char crc16_str[MCU_PROTOCOL_MAX_FRAME_LENGTH];

    // Get the message type
    if(msg_buf[0] == MCU_PROTOCOL_SOF_REQUEST)
    {
        m.type = MSG_TYPE_REQUEST;
    }
    else if(msg_buf[0] == MCU_PROTOCOL_SOF_REQUEST_REPLY)
    {
        m.type = MSG_TYPE_REQUEST_REPLY;
    }
    else if(msg_buf[0] == MCU_PROTOCOL_SOF_NOTIFICATION)
    {
        m.type = MSG_TYPE_NOTIFICATION;
    }

    // Get the message ID
    char *p = strchr((char *)&msg_buf[1], MCU_PROTOCOL_DELIMITER);
    if(p != NULL)
    {
        strncpy((char *)msg_id_str, (char *)&msg_buf[1], p - (char *)&msg_buf[1]);
        msg_id_str[p - (char *)&msg_buf[1]] = '\0';
        m.msg_id_idx = -1;
        for(uint8_t i = 0; i < this.instances[0].config.msg_id_list.msg_id_qty; i++)
        {
            if(strcmp((char *)msg_id_str, (char *)this.instances[0].config.msg_id_list.msg_ids[i]) == 0)
            {
                m.msg_id_idx = i;
                break;
            }
        }

        if(m.msg_id_idx == -1)
        {            
            return m;
        }

        // Get the payload
        char *p2 = strchr((char *)p + 1, MCU_PROTOCOL_DELIMITER);
        
        // if payload exists
        if(p2 != NULL)
        {
            strncpy((char *)payload_str, (char *)p + 1, p2 - p - 1);
            payload_str[p2 - p - 1] = '\0';
            m.payload.params_qty = 0;
            char *p3 = payload_str;
            char *p4 = strchr((char *)p3, MCU_PROTOCOL_PAYLOAD_DELIMITER);
            while(p4 != NULL)
            {
                strncpy((char *)m.payload.params[m.payload.params_qty], (char *)p3, p4 - p3);
                m.payload.params[m.payload.params_qty][p4 - p3] = '\0';
                m.payload.params_qty++;
                p3 = p4 + 1;
                p4 = strchr((char *)p3, MCU_PROTOCOL_PAYLOAD_DELIMITER);
            }
            strncpy((char *)m.payload.params[m.payload.params_qty], (char *)p3, p2 - p3);
            m.payload.params[m.payload.params_qty][p2 - p3] = '\0';
            m.payload.params_qty++;
        }
        
        // Process the CRC
        strncpy((char *)crc16_str, (char *)p2 + 1, 4);
        crc16_str[4] = '\0';
        m.crc16 = (uint16_t)strtol((char *)crc16_str, NULL, 16);

        // Calculate the CRC
        m.crc16_calc = get_crc16(&msg_buf[1], msg_buf_len - 1 - 1 - 4);
    }
    else
    {
        m.too_few_fields = true;
    }

    return m;
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
void MCU_PROTOCOL_tick_1ms()
{
    for(uint8_t i = 0; i < this.instance_idx; i++)
    {
        if(this.instances[i].tmr > 0)
        {
            this.instances[i].tmr--;
        }
    }
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void MCU_PROTOCOL_init()
{
    this.instance_idx = 0;
    for(uint8_t i = 0; i < MCU_PROTOCOL_MAX_INSTANCES_QTY; i++)
    {
        this.instances[i].state = STATE_IDLE;
        this.instances[i].tmr = 0;
        memset(&this.instances[i].config, 0, sizeof(MCU_PROTOCOL_config_t));
        this.instances[i].config.n_cb = NULL;
        this.instances[i].config.rq_cb = NULL;
        this.instances[i].config.rq_r_cb = NULL;
        this.instances[i].config.e_cb = NULL;
    }
}

void MCU_PROTOCOL_tasks()
{
    uint8_t c;

    for(uint8_t i = 0; i < this.instance_idx; i++)
    {
        switch(this.instances[i].state)
        {
            case STATE_IDLE:
                if(QUEUE_pull(&this.instances[i].rx_queue, &c))
                {
                    if(c == MCU_PROTOCOL_SOF_REQUEST || 
                        c == MCU_PROTOCOL_SOF_REQUEST_REPLY ||
                        c == MCU_PROTOCOL_SOF_NOTIFICATION)
                    {
                            this.instances[i].tmr = this.instances[i].config.receive_timeout_ms;
                            this.instances[i].msg_buf[0] = c;
                            this.instances[i].msg_buf_len = 1;
                            this.instances[i].state = STATE_RECEIVING_MSG;

                            DEBUG_MSG("SOF detected for instance %d. Receiving message", i);
                    }
                }
                break;
            
            case STATE_RECEIVING_MSG:
                if(this.instances[i].tmr == 0)
                {
                    // Timeout
                    DEBUG_MSG("Timeout between characters for instance %d", i);
                    this.instances[i].state = STATE_IDLE;
                    if(this.instances[i].config.e_cb != NULL)
                    {
                        this.instances[i].config.e_cb(MCU_PROTOCOL_RX_ERROR_TIMOUT);
                    }

                }
                else if(QUEUE_pull(&this.instances[i].rx_queue, &c))
                {
                    // Restart the timer
                    this.instances[i].tmr = this.instances[i].config.receive_timeout_ms;

                    if(c == MCU_PROTOCOL_EOF)
                    {
                        this.instances[i].msg_buf[this.instances[i].msg_buf_len++] = c;

                        // Process the message
                        DEBUG_MSG("Message received for instance %d", i);

                        this.instances[i].msg = parse_msg(this.instances[i].msg_buf, this.instances[i].msg_buf_len);

                        if(this.instances[i].msg.too_few_fields)
                        {
                            // Too few fields
                            DEBUG_MSG("Too few fields for instance %d", i);
                            this.instances[i].state = STATE_IDLE;
                            if(this.instances[i].config.e_cb != NULL)
                            {
                                this.instances[i].config.e_cb(MCU_PROTOCOL_RX_ERROR_TOO_FEW_FIELDS);
                            }
                        }
                        else if(this.instances[i].msg.msg_id_idx == -1)
                        {
                            // Invalid message ID
                            DEBUG_MSG("Invalid message ID for instance %d", i);
                            this.instances[i].state = STATE_IDLE;
                            if(this.instances[i].config.e_cb != NULL)
                            {
                                this.instances[i].config.e_cb(MCU_PROTOCOL_RX_ERROR_INVALID_MSG_ID);
                            }
                        }
                        else if(this.instances[i].msg.crc16 != this.instances[i].msg.crc16_calc)
                        {
                            // CRC error
                            DEBUG_MSG("CRC error for instance %d", i);
                            this.instances[i].state = STATE_IDLE;
                            if(this.instances[i].config.e_cb != NULL)
                            {
                                this.instances[i].config.e_cb(MCU_PROTOCOL_RX_ERROR_CRC);
                            }
                        }
                        else
                        {
                            // Process the message
                            switch(this.instances[i].msg.type)
                            {
                                case MSG_TYPE_REQUEST:
                                    if(this.instances[i].config.rq_cb != NULL)
                                    {
                                        this.instances[i].config.rq_cb(this.instances[i].msg.msg_id_idx, this.instances[i].msg.payload);
                                    }
                                    break;
                                
                                case MSG_TYPE_REQUEST_REPLY:
                                    if(this.instances[i].config.rq_r_cb != NULL)
                                    {
                                        this.instances[i].config.rq_r_cb(this.instances[i].msg.msg_id_idx, this.instances[i].msg.payload);
                                    }
                                    break;
                                
                                case MSG_TYPE_NOTIFICATION:
                                    if(this.instances[i].config.n_cb != NULL)
                                    {
                                        this.instances[i].config.n_cb(this.instances[i].msg.msg_id_idx, this.instances[i].msg.payload);
                                    }
                                    break;
                                
                                default:
                                    break;
                            }
                        }

                        this.instances[i].msg_buf[this.instances[i].msg_buf_len] = '\0';
                        DEBUG_MSG("Message for instance %d: \"%s\"", i, this.instances[i].msg_buf);

                        // Reset the state
                        this.instances[i].state = STATE_IDLE;
                    }
                    else
                    {
                        // Add the byte to the buffer
                        this.instances[i].msg_buf[this.instances[i].msg_buf_len++] = c;

                        if(this.instances[i].msg_buf_len >= MCU_PROTOCOL_MAX_FRAME_LENGTH)
                        {
                            // Frame size error
                            this.instances[i].state = STATE_IDLE;
                            if(this.instances[i].config.e_cb != NULL)
                            {
                                this.instances[i].config.e_cb(MCU_PROTOCOL_RX_ERROR_FRAME_SIZE);
                            }
                        }
                    }
                }
                break;
            
            default:
                break;
        }
    }
}

MCU_PROTOCOL_handle_t MCU_PROTOCOL_open(MCU_PROTOCOL_config_t *config)
{
    if(this.instance_idx >= MCU_PROTOCOL_MAX_INSTANCES_QTY)
    {
        DEBUG_MSG("Maximum instances reached");
        return -1;
    }

    // Copy configuration
    memcpy(&this.instances[this.instance_idx].config, 
            config, 
            sizeof(MCU_PROTOCOL_config_t));

    // Initialize instance
    this.instances[this.instance_idx].state = STATE_IDLE;
    this.instances[this.instance_idx].tmr = 0;
    QUEUE_create(&this.instances[this.instance_idx].rx_queue, 
        this.instances[this.instance_idx].rx_buf, 
        MCU_PROTOCOL_RX_QUEUE_SIZE,
        sizeof(uint8_t));

    DEBUG_MSG("Instance %d opened", this.instance_idx);
    DEBUG_MSG("\t- Receive timeout: %d ms", this.instances[this.instance_idx].config.receive_timeout_ms);
    DEBUG_MSG("\t- Message ID qty: %d", this.instances[this.instance_idx].config.msg_id_list.msg_id_qty);
    for(uint8_t i = 0; i < this.instances[this.instance_idx].config.msg_id_list.msg_id_qty; i++)
    {
        DEBUG_MSG("\t\t- Message ID %d: \"%s\"", i, this.instances[this.instance_idx].config.msg_id_list.msg_ids[i]);
    }
    
    // Return handle and then increment the index
    return this.instance_idx++;
}

MCU_PROTOCOL_error_code_t MCU_PROTOCOL_push_byte(MCU_PROTOCOL_handle_t handle, 
                                                uint8_t byte)
{
    if(handle < 0 && handle >= this.instance_idx)
    {
        DEBUG_MSG("Invalid handle");
        return MCU_PROTOCOL_TX_ERROR_INVALID_HANDLE;
    }

    if(QUEUE_push(&this.instances[handle].rx_queue, &byte))
    {
        return MCU_PROTOCOL_ERROR_OK;
    }
    else
    {
        DEBUG_MSG("RX queue full");
        return MCU_PROTOCOL_RX_QUEUE_FULL;
    }
}

MCU_PROTOCOL_error_code_t MCU_PROTOCOL_assemble_request(
                                            MCU_PROTOCOL_handle_t handle, 
                                            uint8_t msg_id_idx, 
                                            MCU_PROTOCOL_payload_list_t payload,
                                            uint8_t *dst)
{
    // Validate the handle
    if(handle < 0 && handle >= this.instance_idx)
    {
        DEBUG_MSG("Invalid handle");
        return MCU_PROTOCOL_TX_ERROR_INVALID_HANDLE;
    }

    // Validate the message ID index
    if(msg_id_idx >= this.instances[handle].config.msg_id_list.msg_id_qty)
    {
        DEBUG_MSG("Invalid message ID index");
        return MCU_PROTOCOL_TX_ERROR_INVALID_MSG_ID_IDX;
    }

    // Verify that the frame size is correct
    if(get_frame_length(handle, msg_id_idx, payload) > MCU_PROTOCOL_MAX_FRAME_LENGTH)
    {
        DEBUG_MSG("Frame size bigger than the limit (%d)", MCU_PROTOCOL_MAX_FRAME_LENGTH);
        return MCU_PROTOCOL_TX_ERROR_INVALID_FRAME_SIZE;
    }

    // Assemble the message
    assemble_msg(MCU_PROTOCOL_SOF_REQUEST, handle, msg_id_idx, payload, dst);

    DEBUG_MSG("Request assembled: \"%s\"", dst);

    return MCU_PROTOCOL_ERROR_OK;
}

MCU_PROTOCOL_error_code_t MCU_PROTOCOL_assemble_request_reply(
                                            MCU_PROTOCOL_handle_t handle, 
                                            uint8_t msg_id_idx, 
                                            MCU_PROTOCOL_payload_list_t payload,
                                            uint8_t *dst)
{
    // Validate the handle
    if(handle < 0 && handle >= this.instance_idx)
    {
        DEBUG_MSG("Invalid handle");
        return MCU_PROTOCOL_TX_ERROR_INVALID_HANDLE;
    }

    // Validate the message ID index
    if(msg_id_idx >= this.instances[handle].config.msg_id_list.msg_id_qty)
    {
        DEBUG_MSG("Invalid message ID index");
        return MCU_PROTOCOL_TX_ERROR_INVALID_MSG_ID_IDX;
    }

    // Verify that the frame size is correct
    if(get_frame_length(handle, msg_id_idx, payload) > MCU_PROTOCOL_MAX_FRAME_LENGTH)
    {
        DEBUG_MSG("Frame size bigger than the limit (%d)", MCU_PROTOCOL_MAX_FRAME_LENGTH);
        return MCU_PROTOCOL_TX_ERROR_INVALID_FRAME_SIZE;
    }

    // Assemble the message
    assemble_msg(MCU_PROTOCOL_SOF_REQUEST_REPLY, handle, msg_id_idx, payload, dst);

    DEBUG_MSG("Request reply assembled: \"%s\"", dst);

    return MCU_PROTOCOL_ERROR_OK;
}

MCU_PROTOCOL_error_code_t MCU_PROTOCOL_assemble_notification(
                                        MCU_PROTOCOL_handle_t handle, 
                                        uint8_t msg_id_idx, 
                                        MCU_PROTOCOL_payload_list_t payload,
                                        uint8_t *dst)
{
    // Validate the handle
    if(handle < 0 && handle >= this.instance_idx)
    {
        DEBUG_MSG("Invalid handle");
        return MCU_PROTOCOL_TX_ERROR_INVALID_HANDLE;
    }

    // Validate the message ID index
    if(msg_id_idx >= this.instances[handle].config.msg_id_list.msg_id_qty)
    {
        DEBUG_MSG("Invalid message ID index");
        return MCU_PROTOCOL_TX_ERROR_INVALID_MSG_ID_IDX;
    }

    // Verify that the frame size is correct
    if(get_frame_length(handle, msg_id_idx, payload) > MCU_PROTOCOL_MAX_FRAME_LENGTH)
    {
        DEBUG_MSG("Frame size bigger than the limit (%d)", MCU_PROTOCOL_MAX_FRAME_LENGTH);
        return MCU_PROTOCOL_TX_ERROR_INVALID_FRAME_SIZE;
    }

    // Assemble the message
    assemble_msg(MCU_PROTOCOL_SOF_NOTIFICATION, handle, msg_id_idx, payload, dst);

    DEBUG_MSG("Notification assembled: \"%s\"", dst);

    return MCU_PROTOCOL_ERROR_OK;
}
