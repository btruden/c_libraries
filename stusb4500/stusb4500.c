/*******************************************************************************
 * @file        stusb4500.c
 * @brief       This file provides the functionality for USB PD by controlling
 *              the STUSB4500
 * @author      Blas Truden
 * @date        20241210
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
#include "stusb4500.h"
#include "stusb4500_defs.h"
#include "debug.h"
#include "tick.h"
#include "board.h"
#include "i2c.h"

/******************************************************************************
 * Local Constants
 ******************************************************************************/
// Debug options
#define ENABLE_DEBUG        true
#define DEBUG_TAG           "STUSB4500"

#if ENABLE_DEBUG == true
#define DEBUG_MSG(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

#define PRINT_LINE(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)

// Time defines
#define WAIT_REG_LOAD_TIMEOUT               50
#define CONNECTION_STABILIZATION_TIMEOUT    1000
#define FULL_STATUS_READ_PERIOD_MS          1000
#define RESET_TIMEOUT                       100

// I2C defines
#define STUSB4500_ADDR                      0b0101000

// Configuration values
#define STUSB4500_ALERTS_CONFIGURATION       0xFB   // All alerts masked

// -------------------------- Power Profiles ----------------------------------
#define PDO_QTY                             3
#define PDO1_VOLTAGE_mV                     5000
#define PDO1_CURRENT_mA                     200
#define PDO2_VOLTAGE_mV                     5000
#define PDO2_CURRENT_mA                     3000
#define PDO3_VOLTAGE_mV                     20000
#define PDO3_CURRENT_mA                     3000

/**
 * Default configuration array containing register address/value pairs
 */
static uint8_t config_default[] = {
    // General configuration
    STUSB4500_ALERT_STATUS_1_MASK, 0xFB,    // All alerts masked [default]
    STUSB4500_VBUS_DISCHARGE_CTRL, 0x00,    // VBUS discharge disabled [default]
    STUSB4500_MONITORING_CTRL_0, 0x00,      // SNK_DISC_HIGH: Select a VBUS threshold at 3.5 V [default]
    STUSB4500_MONITORING_CTRL_2, 0xFF,      // VSHIFT_HIGH and VSHIFT_LOW config [default]
    STUSB4500_RESET_CTRL, 0x01,             // SW_RESET enabled
    STUSB4500_VBUS_DISCHARGE_TIME_CTRL, 0x9C, // DISCHARGE_TIME_TO_0V is  756 ms, DISCHARGE_TIME_TRANSITION is 288 ms [default]
    STUSB4500_VBUS_DISCHARGE_CTRL, 0x00,    // VBUS discharge disabled [default]
    STUSB4500_GPIO_SW_GPIO, 0x00,           // GPIO pin value is Hi-Z [default]

    // PDO configuration
    STUSB4500_DPM_PDO_NUMB, PDO_QTY,        // Number of PDOs
/** 
 * STUSB4500 Fixed Supply PDO Sink Configuration
 * ----------------------------------------------------------------------------
 *
 * The Power Data Object (PDO) is a 32-bit register that defines power 
 * requirements in USB Power Delivery. This explanation details the bit 
 * configuration of the Fixed Supply PDO Sink for the STUSB4500.
 *
 * Bits   | Name                    | Description
 * -------|-------------------------|------------------------------------------
 * 31:30  | PDO Type                | Specifies the type of power source:
 *        |                         |   - 00b: Fixed Supply PDO
 *        |                         |          (fixed voltage and current).
 *        |                         |   - 01b: Battery PDO
 *        |                         |          (specifies energy in Wh).
 *        |                         |   - 10b: Variable Supply PDO (non-battery)
 *        |                         |          (voltage range defined).
 *        |                         |   - 11b: Augmented PDO
 *        |                         |          (e.g., Programmable Power Supply).
 * 29     | Dual-role power         | Set to 1 if the sink device can act as a power 
 *        |                         | source as well; otherwise, set to 0.
 * 28     | Higher capability       | Set to 1 if the device can operate in higher 
 *        |                         | performance states when more power is provided.
 * 27     | Unconstrained power     | Set to 1 if the device is unconstrained by power; 
 *        |                         | otherwise, set to 0.
 * 26     | USB communication       | Set to 1 if the device is capable of USB communication.
 * 25     | Dual-role data          | Set to 1 if the device supports dual-role data 
 *        |                         | (host and device switching); otherwise, set to 0.
 * 24:23  | Fast Role Swap Required | Indicates the USB Type-C fast role swap current:
 *        |                         |   - 00b: Fast role swap not supported (default)
 *        |                         |   - 01b: Default USB power (500mA @ 5V)
 *        |                         |   - 10b: 1.5A @ 5V
 *        |                         |   - 11b: 3.0A @ 5V
 * 22:20  | Reserved                | Must be set to 0 (reserved for future use).
 * 19:10  | Voltage                 | Specifies the desired voltage in 50mV units.
 *        |                         | Example: 100 = 5V, 400 = 20V.
 * 9:0    | Operational current     | Specifies the desired operational current 
 *        |                         | in 10mA units.
 *        |                         | Example: 300 = 3.0A, 100 = 1.0A.
 *
 * **************************************************************************
 */

/**
 * PDO1 CONFIGURATION
 * ----------------------------------------------------------------------------
 * This configuration is used to accept the minimum power required by the device
 * to be able to operate. The device will not be able to charge the battery nor
 * operate the display.
 * ----------------------------------------------------------------------------
 * 
 * Value: 0b 00 0 0 0 1 0 00 000 0001100100 0000010100 [0x04019014]
 * 
 *  - PDO Type: Fixed Supply (00)
 *  - Dual-role power: No (0)
 *  - Higher capability: No (0)
 *  - Unconstrained power: No (0)
 *  - USB communication: Yes (1)
 *  - Dual-role data: No (0)
 *  - Fast Role Swap Required: No (00)
 *  - Reserved: 000
 *  - Voltage: 5V (0001100100 in 50mV units)
 *  - Operational current: 0.2A (0000000010 in 10mA units)
 */
    //STUSB4500_DPM_SNK_PDO1_0, 0x14,
    //STUSB4500_DPM_SNK_PDO1_1, 0x90,
    //STUSB4500_DPM_SNK_PDO1_2, 0x01,
    //STUSB4500_DPM_SNK_PDO1_3, 0x04,

/**
 * PDO2 CONFIGURATION
 * ----------------------------------------------------------------------------
 * This configuration is used to fully power the device from a 5V source.
 * ----------------------------------------------------------------------------
 * 
 * Value: 0b 00 0 0 0 1 0 00 000 0001100100 0100101100 [0x0401912C]
 * 
 *  - PDO Type: Fixed Supply (00)
 *  - Dual-role power: No (0)
 *  - Higher capability: No (0)
 *  - Unconstrained power: No (0)
 *  - USB communication: Yes (1)
 *  - Dual-role data: No (0)
 *  - Fast Role Swap Required: No (00)
 *  - Reserved: 000
 *  - Voltage: 5V (0001100100 in 50mV units)
 *  - Operational current: 3A (0100101100 in 10mA units)
 */
    //STUSB4500_DPM_SNK_PDO2_0, 0x2C,
    //STUSB4500_DPM_SNK_PDO2_1, 0x91,
    //STUSB4500_DPM_SNK_PDO2_2, 0x01,
    //STUSB4500_DPM_SNK_PDO2_3, 0x04,

/**
 * PDO3 CONFIGURATION
 * ----------------------------------------------------------------------------
 * This configuration is the higher power capable for 20V capable sources.
 * ----------------------------------------------------------------------------
 * 
 * Value: 0b 00 0 0 0 1 0 00 000 0110010000 0100101100 [0x0406412C]
 * 
 *  - PDO Type: Fixed Supply (00)
 *  - Dual-role power: No (0)
 *  - Higher capability: No (0)
 *  - Unconstrained power: No (0)
 *  - USB communication: Yes (1)
 *  - Dual-role data: No (0)
 *  - Fast Role Swap Required: No (00)
 *  - Reserved: 000
 *  - Voltage: 20V (0110010000 in 50mV units)
 *  - Operational current: 3A (0100101100 in 10mA units)
 */
    //STUSB4500_DPM_SNK_PDO3_0, 0x2C,
    //STUSB4500_DPM_SNK_PDO3_1, 0x41,
    //STUSB4500_DPM_SNK_PDO3_2, 0x06,
    //STUSB4500_DPM_SNK_PDO3_3, 0x04,
};

/**
 * Reset command address/value pair array
 */
static uint8_t reset_cmd[] = {
    STUSB4500_TX_HEADER_LOW, 0x0D,      // Reset command value
    STUSB4500_PD_COMMAND_CTRL, 0x26,    // Send command
};

/**
 * Default PDO configuration
 */
static STUSB4500_SNK_PDO_t pdo_default = {
    .fix.Fixed_Supply = 0,
    .fix.Dual_Role_Power = 0,
    .fix.Higher_Capability = 0,
    .fix.Unconstrained_Power = 0,
    .fix.USB_Communications_Capable = 1,
    .fix.Dual_Role_Data = 0,
    .fix.Fast_Role_Req_cur = 0,
    .fix.Reserved_22_20 = 0,
    .fix.Voltage = PDO1_VOLTAGE_mV/50,
    .fix.Operationnal_Current = PDO1_CURRENT_mA/10,
};

/******************************************************************************
 * Local Types
 ******************************************************************************/
/**
 * Local states type
 */
typedef enum
{
    STATE_INIT = 0,
    STATE_CONFIGURING,
    STATE_USB_DISCONNECTED,
    STATE_USB_WAIT_CONNECTION_STABLE,
    STATE_GETTING_POWER_PROFILE_RESULT,
    STATE_USB_CONNECTED,
}state_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
    state_t state;
    STUSB4500_SNK_PDO_t pdo[PDO_QTY];           // Stores the PDO configured values
    STUSB4500_RDO_t rdo;                        // Stores the RDO value
    STUSB4500_full_status_t status;             // Stores the status of the STUSB4500
    uint32_t tmr;
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Loads the local timer with ms
 */
static void timer_set(uint32_t ms)
{
    this.tmr = ms;
}

/**
 * Returns true if the timer is zero
 */
static bool timer_out()
{
    return this.tmr == 0;
}

/**
 * Writes multiple registers on the STUSB4500
 * 
 * @param data pointer to the data buffer containing register address/value pairs
 * @param length amount of registers to write
 * @param init true if the function is called for the first time
 * 
 * @return true if the registers were read
 */
static bool write_multiple_registers(uint8_t *data, uint8_t length, bool init)
{
    bool ret = false;
    static uint8_t *tx_data;
    static uint16_t total_regs, cnt;
    static I2C_transaction_t t;

    static enum
    {
        WRSTATE_INIT = 0,
        WRSTATE_WRITING_REGS
    }wrState = WRSTATE_INIT;

    if(init)
    {
        wrState = WRSTATE_INIT;
    }

    switch (wrState)
    {
        case WRSTATE_INIT:
            tx_data = data;
            total_regs = length;
            cnt = 0;
            t.address = STUSB4500_ADDR;
            t.tx_data = tx_data;
            t.tx_length = 2;
            t.type = I2C_TRANSACTION_WRITE;

            if(I2C_add_transaction(&t))
            {
                wrState = WRSTATE_WRITING_REGS;
                DEBUG_MSG("Writing regiter 0x%02x with value 0x%02x", t.tx_data[0], t.tx_data[1]);
            }
            else
            {
                DEBUG_MSG("Error adding transaction");
                ret = true;
                wrState = WRSTATE_INIT;
            }

            break;
        
        case WRSTATE_WRITING_REGS:
            switch(t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    cnt++;
                    if(cnt < total_regs)
                    {
                        tx_data += 2;
                        t.tx_data = tx_data;
                        DEBUG_MSG("Writing regiter 0x%02x with value 0x%02x", t.tx_data[0], t.tx_data[1]);
                        if(!I2C_add_transaction(&t))
                        {
                            DEBUG_MSG("Error adding transaction");
                            ret = true;
                            wrState = WRSTATE_INIT;
                        }
                    }
                    else
                    {
                        DEBUG_MSG("Registers written");
                        ret = true;
                        wrState = WRSTATE_INIT;
                    }
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error writing register");
                    ret = true;
                    wrState = WRSTATE_INIT;
                    break;
                
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return ret;
}

/**
 * Writes multiple consecutive registers on the STUSB4500
 * 
 * @param addr register address to start writing
 * @param data pointer to the data buffer containing register address/value pairs
 * @param length amount of registers to write
 * @param init true if the function is called for the first time
 * 
 * @return true if the registers were read
 */
static bool write_consecutive_registers(uint8_t addr, uint8_t *data, uint8_t length, bool init)
{
    bool ret = false;
    static uint8_t tx_data[264];
    static I2C_transaction_t t;

    static enum
    {
        WRCONS_STATE_INIT = 0,
        WRCONS_STATE_WRITING_REGS
    }wrconState = WRCONS_STATE_INIT;

    if(init)
    {
        wrconState = WRCONS_STATE_INIT;
    }

    switch (wrconState)
    {
        case WRCONS_STATE_INIT:
            t.address = STUSB4500_ADDR;

            memcpy(&tx_data[1], data, length);
            tx_data[0] = addr;

            t.tx_data = tx_data;
            t.tx_length = length+1;
            t.rx_data = NULL;
            t.rx_length = 0;

            t.type = I2C_TRANSACTION_WRITE;

            if(I2C_add_transaction(&t))
            {
                wrconState = WRCONS_STATE_WRITING_REGS;
            }
            else
            {
                DEBUG_MSG("Error adding transaction");
                ret = true;
                wrconState = WRCONS_STATE_INIT;
            }

            break;
        
        case WRCONS_STATE_WRITING_REGS:
            switch(t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    DEBUG_MSG("Registers written");
                    ret = true;
                    wrconState = WRCONS_STATE_INIT;
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error writing register");
                    ret = true;
                    wrconState = WRCONS_STATE_INIT;
                    break;
                
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return ret;
}

/**
 * Reads a register from the STUSB4500.
 * 
 * @param dst pointer to the data buffer where the register will be stored
 * @param reg register address to read
 * 
 * @return true if the register was read
 */
static bool read_register(uint8_t *dst, uint8_t reg, bool init)
{
    bool ret = false;
    static I2C_transaction_t t;
    static uint8_t tx_buff[1];

    static enum
    {
        RDSTATE_INIT = 0,
        RDSTATE_READING,
    }rdState = RDSTATE_INIT;

    if(init)
    {
        rdState = RDSTATE_INIT;
    }

    switch(rdState)
    {
        case RDSTATE_INIT:
            t.address = STUSB4500_ADDR;
            t.rx_data = dst;
            t.rx_length = 1;
            t.tx_data = tx_buff;
            t.tx_length = 1;
            t.type = I2C_TRANSACTION_WRITE_READ;
            tx_buff[0] = reg;
            if(I2C_add_transaction(&t))
            {
                rdState = RDSTATE_READING;
            }
            else
            {
                DEBUG_MSG("Error adding transaction");
                rdState = RDSTATE_INIT;
                ret = true;
            }
            break;
        
        case RDSTATE_READING:
            switch(t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    ret = true;
                    rdState = RDSTATE_INIT;
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error reading register 0x%02x", tx_buff[0]);
                    ret = true;
                    rdState = RDSTATE_INIT;
                    break;
                
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return ret;
}


/**
 * Reads multiple registers from the STUSB4500
 * 
 * @param dst pointer to the data buffer where the registers will be stored
 * @param reg register address to start reading
 * @param length amount of registers to read
 * 
 * @return true if the registers were read
 */
static bool read_consecutive_registers(uint8_t *dst, uint8_t reg, uint8_t length, bool init)
{
    bool ret = false;
    static I2C_transaction_t t;
    static uint8_t tx_buff[1];

    static enum
    {
        RDSTATE_INIT = 0,
        RDSTATE_READING,
    }rdState = RDSTATE_INIT;

    if(init)
    {
        rdState = RDSTATE_INIT;
    }

    switch(rdState)
    {
        case RDSTATE_INIT:
            t.address = STUSB4500_ADDR;
            t.rx_data = dst;
            t.rx_length = length;
            t.tx_data = tx_buff;
            t.tx_length = 1;
            t.type = I2C_TRANSACTION_WRITE_READ;
            tx_buff[0] = reg;
            if(I2C_add_transaction(&t))
            {
                rdState = RDSTATE_READING;
            }
            else
            {
                DEBUG_MSG("Error adding transaction");
                rdState = RDSTATE_INIT;
                ret = true;
            }
            break;
        
        case RDSTATE_READING:
            switch(t.status)
            {
                case I2C_TRANSACTION_STATUS_DONE:
                    ret = true;
                    rdState = RDSTATE_INIT;
                    break;
                
                case I2C_TRANSACTION_STATUS_ERROR:
                    DEBUG_MSG("Error reading register 0x%02x", tx_buff[0]);
                    ret = true;
                    rdState = RDSTATE_INIT;
                    break;
                
                default:
                    break;
            }
            break;
        
        default:
            break;
    }

    return ret;
}

/**
 * Sets a PDO configuration. 
 * 
 * If a source is already attached, it's important to perform a reset to the 
 * STUSB4500 to trigger a new power negotiation.
 * 
 * @param pdo_id PDO ID to set
 * @param mV voltage in mV
 * @param mA current in mA
 * @param init true if the function is called for the first time
 * 
 * @return true if the PDO was set
 */
static bool set_pdo(uint8_t pdo_id, uint16_t mV, uint16_t mA, bool init)
{
    bool ret = false;

    static STUSB4500_SNK_PDO_t pdo;

    static enum
    {
        SET_PDO_INIT = 0,
        SET_PDO_WRITING
    }setPDOstate = SET_PDO_INIT;

    if(pdo_id >= PDO_QTY)
    {
        DEBUG_MSG("Invalid PDO ID");
        return true;
    }

    if(init)
    {
        setPDOstate = SET_PDO_INIT;
    }

    switch(setPDOstate)
    {
        case SET_PDO_INIT:
            pdo.d32 = pdo_default.d32;
            pdo.fix.Voltage = mV/50;
            pdo.fix.Operationnal_Current = mA/10;
            
            write_consecutive_registers(STUSB4500_DPM_SNK_PDO1_0 + (pdo_id*4),pdo.d8,sizeof(pdo),true);

            setPDOstate = SET_PDO_WRITING;
            break;
        
        case SET_PDO_WRITING:
            if(write_consecutive_registers(STUSB4500_DPM_SNK_PDO1_0 + (pdo_id*4),pdo.d8,sizeof(pdo),false))
            {
                DEBUG_MSG("PDO %d set to %08x", pdo_id, pdo.d32);
                DEBUG_MSG("\t-current: %d mA", pdo.fix.Operationnal_Current * 10);
                DEBUG_MSG("\t-voltage: %d mV", pdo.fix.Voltage * 50);
                setPDOstate = SET_PDO_INIT;
                ret = true;
            }
            break;

        default:
            break;
    }

    return ret;
}

/**
 * Configures the STUSB4500
 * 
 * @param init true if the function is called for the first time
 * @return true if the configuration is done
 */
static bool configure(bool init)
{
    bool ret = false;
    static uint8_t rx_buff[32]; 

    static enum
    {
        CONFSTATE_INIT = 0,
        CONFSTATE_WAIT_STARTUP_REGS_LOADING,
        CONFSTATE_READ_ID,
        CONFSTATE_READING_EXISTING_SNK_PDOs,
        CONFSTATE_CLEANING_STATUS_REGS,
        CONFSTATE_WRITING_CONFIG,
        CONFSTATE_SETTING_PDO1,
        CONFSTATE_SETTING_PDO2,
        CONFSTATE_SETTING_PDO3,
        CONFSTATE_RESETTING,
        CONFSTATE_RESET_WAIT,
        CONFSTATE_VERIFY_PDO_CONFIGS,
        CONFSTATE_ERROR
    }confState = CONFSTATE_INIT;

    if(init)
    {
        confState = CONFSTATE_INIT;
    }

    switch (confState)  
    {
        case CONFSTATE_INIT:
            timer_set(WAIT_REG_LOAD_TIMEOUT);
            confState = CONFSTATE_WAIT_STARTUP_REGS_LOADING;
            break;
        
        case CONFSTATE_WAIT_STARTUP_REGS_LOADING:
            if(timer_out())
            {
                read_register(rx_buff, STUSB4500_DEVICE_ID, true);
                confState = CONFSTATE_READ_ID;
            }
            break;

        case CONFSTATE_READ_ID:
            if(read_register(rx_buff, STUSB4500_DEVICE_ID, false))
            {
                DEBUG_MSG("Device ID: 0x%02x", rx_buff[0]);

                if(rx_buff[0] == STUSB4500_DEVICE_ID_VALUE)
                {
                    DEBUG_MSG("Device ID is correct. Reading existing SNK PDOs...");
                    read_consecutive_registers(rx_buff, STUSB4500_DPM_SNK_PDO1_0, 12, true);

                    confState = CONFSTATE_READING_EXISTING_SNK_PDOs;
                }
                else
                {
                    DEBUG_MSG("Device ID is incorrect");
                    confState = CONFSTATE_ERROR;
                }
            }
            break;

        case CONFSTATE_READING_EXISTING_SNK_PDOs:
            if(read_consecutive_registers(rx_buff, STUSB4500_DPM_SNK_PDO1_0, 12, false))
            {
                // Copying the PDOs read form chip into the local data structure
                for ( int i = 0 ; i < 3 ; i++)
                {
                    this.pdo[i].d32 = (uint32_t )( rx_buff[i*4] +(rx_buff[(i*4)+1]<<8)+(rx_buff[(i*4)+2]<<16)+(rx_buff[(i*4)+3]<<24));
                } 

                DEBUG_MSG("Existing SNK PDO configs:");

                for ( int i = 0 ; i < 3 ; i++)
                {
                    DEBUG_MSG("\tPDO%d: 0x%08x", i+1, this.pdo[i].d32);
                    DEBUG_MSG("\t\t-current: %d mA", this.pdo[i].fix.Operationnal_Current * 10);
                    DEBUG_MSG("\t\t-voltage: %d mV", this.pdo[i].fix.Voltage * 50);
                }

                DEBUG_MSG("Clearing all ALERT status...");
                read_consecutive_registers(rx_buff, STUSB4500_ALERT_STATUS_1, 12, true);
                confState = CONFSTATE_CLEANING_STATUS_REGS;
            }
            break;

        case CONFSTATE_CLEANING_STATUS_REGS:
            if(read_consecutive_registers(rx_buff, STUSB4500_ALERT_STATUS_1, 12, false))
            {
                DEBUG_MSG("Status registers cleared. Writing configuration...");
                write_multiple_registers(config_default, sizeof(config_default)/2, true);
                confState = CONFSTATE_WRITING_CONFIG;
            }
            break;

        case CONFSTATE_WRITING_CONFIG:
            if(write_multiple_registers(NULL, 0, false))
            {
                DEBUG_MSG("Configuration done. Setting PDO1...");
                
                set_pdo(0, PDO1_VOLTAGE_mV, PDO1_CURRENT_mA, true);

                confState = CONFSTATE_SETTING_PDO1;
            }
            break;

        case CONFSTATE_SETTING_PDO1:
            if(set_pdo(0, PDO1_VOLTAGE_mV, PDO1_CURRENT_mA, false))
            {
                DEBUG_MSG("PDO1 set. Setting PDO2...");
                set_pdo(1, PDO2_VOLTAGE_mV, PDO2_CURRENT_mA, true);
                confState = CONFSTATE_SETTING_PDO2;
            }
            break;

        case CONFSTATE_SETTING_PDO2:
            if(set_pdo(1, PDO2_VOLTAGE_mV, PDO2_CURRENT_mA, false))
            {
                DEBUG_MSG("PDO2 set. Setting PDO3...");
                set_pdo(2, PDO3_VOLTAGE_mV, PDO3_CURRENT_mA, true);
                confState = CONFSTATE_SETTING_PDO3;
            }
            break;

        case CONFSTATE_SETTING_PDO3:
            if(set_pdo(2, PDO3_VOLTAGE_mV, PDO3_CURRENT_mA, false))
            {
                DEBUG_MSG("PDO3 set. Resetting STUSB4500...");
                write_multiple_registers(reset_cmd, sizeof(reset_cmd)/2, true);
                confState = CONFSTATE_RESETTING;
            }
            break;

        case CONFSTATE_RESETTING:
            if(write_multiple_registers(NULL, 0, false))
            {
                DEBUG_MSG("Reset command send");
                timer_set(RESET_TIMEOUT);
                confState = CONFSTATE_RESET_WAIT;
            }
            break;
        
        case CONFSTATE_RESET_WAIT:
            if(timer_out())
            {
                DEBUG_MSG("Reset done. Verifying PDO configurations...");
                read_consecutive_registers(rx_buff, STUSB4500_DPM_SNK_PDO1_0, 12, true);
                confState = CONFSTATE_VERIFY_PDO_CONFIGS;
            }
            break;

        case CONFSTATE_VERIFY_PDO_CONFIGS:
            if(read_consecutive_registers(rx_buff, STUSB4500_DPM_SNK_PDO1_0, 12, false))
            {
                // Copying the PDOs read form chip into the local data structure
                for ( int i = 0 ; i < 3 ; i++)
                {
                    this.pdo[i].d32 = (uint32_t )( rx_buff[i*4] +(rx_buff[(i*4)+1]<<8)+(rx_buff[(i*4)+2]<<16)+(rx_buff[(i*4)+3]<<24));
                } 

                for ( int i = 0 ; i < 3 ; i++)
                {
                    DEBUG_MSG("\tPDO%d: 0x%08x", i+1, this.pdo[i].d32);
                    DEBUG_MSG("\t\t-current: %d mA", this.pdo[i].fix.Operationnal_Current * 10);
                    DEBUG_MSG("\t\t-voltage: %d mV", this.pdo[i].fix.Voltage * 50);
                    DEBUG_MSG("\t\t-USB communication capable: %d", this.pdo[i].fix.USB_Communications_Capable);
                    DEBUG_MSG("\t\t-USB dual role data capable: %d", this.pdo[i].fix.Dual_Role_Data);
                    DEBUG_MSG("\t\t-USB dual role power capable: %d", this.pdo[i].fix.Dual_Role_Power);
                    DEBUG_MSG("\t\t-USB fast role swap required: %d", this.pdo[i].fix.Fast_Role_Req_cur);
                    DEBUG_MSG("\t\t-USB higher capability: %d", this.pdo[i].fix.Higher_Capability);
                    DEBUG_MSG("\t\t-USB unconstrained power: %d", this.pdo[i].fix.Unconstrained_Power);
                }
                
                confState = CONFSTATE_INIT;                    
                ret = true;
            }
            break;
        
        case CONFSTATE_ERROR:
        default:
            break;
    }
    
    return ret;
}

/**
 * Reads the power negotiation result after USB has been attached. This function
 * must be called at least 500ms after the USB has been attached.
 * 
 * @param init true if the function is called for the first time
 * @param rdo pointer to the RDO structure where the result will be stored
 * 
 * @return true if the power negotiation result was read
 */
static bool read_power_negotation_result(bool init, STUSB4500_RDO_t *rdo)
{
    bool ret = false;
    static uint8_t rx_buff[8];

    static enum
    {
        RDO_STATE_INIT = 0,
        RDO_STATE_READING,
    }rdoState = RDO_STATE_INIT;

    if(init)
    {
        rdoState = RDO_STATE_INIT;
    }

    switch(rdoState)
    {
        case RDO_STATE_INIT:
            read_consecutive_registers(rx_buff, STUSB4500_RDO_REG_STATUS_0, 4, true);
            rdoState = RDO_STATE_READING;
            break;
        
        case RDO_STATE_READING:
            if(read_consecutive_registers(rx_buff, STUSB4500_RDO_REG_STATUS_0, 4, false))
            {
                memcpy(rdo, rx_buff, sizeof(STUSB4500_RDO_t));
                DEBUG_MSG("RDO: 0x%08x", rdo->word);
                DEBUG_MSG("\t-max_operating_current: %d mA", rdo->bits.max_operating_current * 10);
                DEBUG_MSG("\t-operating_current: %d mA", rdo->bits.operating_current * 10);
                DEBUG_MSG("\t-unchecked_extended_msg_supported: %d", rdo->bits.unchunked_ext_msg_supported);
                DEBUG_MSG("\t-no_usb_suspend: %d", rdo->bits.no_usb_suspend);
                DEBUG_MSG("\t-usb_comm_capable: %d", rdo->bits.usb_comm_capable);
                DEBUG_MSG("\t-capability_mismatch: %d", rdo->bits.capability_mismatch);
                DEBUG_MSG("\t-give_back_flag: %d", rdo->bits.give_back_flag);
                DEBUG_MSG("\t-object_position: %d", rdo->bits.object_position);

                rdoState = RDO_STATE_INIT;
                ret = true;
            }
            break;
        
        default:
            break;
    }

    return ret;
}

/**
 * Reads the complete set of status registers form the chip periodically
 * 
 * @param init true if the function is called for the first time
 */
static void status_monitor(bool init)
{
    static enum
    {
        STATUS_STATE_INIT = 0,
        STATUS_STATE_READING, 
        STATUS_STATE_WAIT,
    }statusState = STATUS_STATE_INIT;

    if(init)
    {
        statusState = STATUS_STATE_INIT;
    }

    switch(statusState)
    {
        case STATUS_STATE_INIT:
            read_consecutive_registers(this.status.bytes, STUSB4500_ALERT_STATUS_1, sizeof(this.status), true);
            timer_set(FULL_STATUS_READ_PERIOD_MS);
            statusState = STATUS_STATE_READING;
            break;

        case STATUS_STATE_READING:
            if(read_consecutive_registers(this.status.bytes, STUSB4500_ALERT_STATUS_1, sizeof(this.status), false))
            {
                DEBUG_MSG("\tFULL STATUS:");
                DEBUG_MSG("\t\t-ALERT_STATUS_1: 0x%02x", this.status.status_regs.alert_status_1);
                DEBUG_MSG("\t\t-ALERT_STATUS_1_MASK: 0x%02x", this.status.status_regs.alert_status_1_mask);
                DEBUG_MSG("\t\t-PORT_STATUS_0: 0x%02x", this.status.status_regs.port_status_0);
                DEBUG_MSG("\t\t-PORT_STATUS_1: 0x%02x", this.status.status_regs.port_status_1);
                DEBUG_MSG("\t\t-TYPEC_MONITORING_STATUS_0: 0x%02x", this.status.status_regs.typec_monitoring_status_0);
                DEBUG_MSG("\t\t-TYPEC_MONITORING_STATUS_1: 0x%02x", this.status.status_regs.typec_monitoring_status_1);
                DEBUG_MSG("\t\t-CC_STATUS: 0x%02x", this.status.status_regs.cc_status);
                DEBUG_MSG("\t\t-CC_HW_FAULT_STATUS_0: 0x%02x", this.status.status_regs.cc_hw_fault_status_0);
                DEBUG_MSG("\t\t-CC_HW_FAULT_STATUS_1: 0x%02x", this.status.status_regs.cc_hw_fault_status_1);
                DEBUG_MSG("\t\t-PD_TYPEC_STATUS: 0x%02x", this.status.status_regs.pd_typec_status);
                DEBUG_MSG("\t\t-TYPEC_STATUS: 0x%02x", this.status.status_regs.typec_status);
                DEBUG_MSG("\t\t-PRT_STATUS: 0x%02x", this.status.status_regs.prt_status);

                statusState = STATUS_STATE_WAIT;
            }
            break;

        case STATUS_STATE_WAIT:
            if(timer_out())
            {
                read_consecutive_registers(this.status.bytes, STUSB4500_ALERT_STATUS_1, sizeof(this.status), true);
                timer_set(FULL_STATUS_READ_PERIOD_MS);
                statusState = STATUS_STATE_READING;
            }
            break;

        default:
            break;
    }
}

/******************************************************************************
 * Callback Functions
 ******************************************************************************/
/**
 * Timer tick callback used to handle local timer
 */
static void tick_callback()
{
    if(this.tmr)
        this.tmr--;
}

/******************************************************************************
 * Public Functions
 ******************************************************************************/
void STUSB4500_init()
{ 
    TICK_callback_register((TICK_callback_t)tick_callback); 
    memset(&this.pdo, 0, sizeof(this.pdo));
    memset(&this.rdo, 0, sizeof(this.rdo));
    memset(&this.status, 0, sizeof(this.status));
    this.state = STATE_INIT;
}

void STUSB4500_tasks()
{     
    switch (this.state)
    {
        case STATE_INIT:
            configure(true);
            DEBUG_MSG("Configuring...");
            this.state = STATE_CONFIGURING;
            break;

        case STATE_CONFIGURING:
            if(configure(false))
            {
                DEBUG_MSG("Configured");
                DEBUG_MSG("Disconnected");
                status_monitor(true);
                this.state = STATE_USB_DISCONNECTED;
            }
            break;

        case STATE_USB_DISCONNECTED:
            status_monitor(false);
            if(BOARD_usb_detected() && this.status.status_regs.port_status_1.bits.attach)
            {
                DEBUG_MSG("USB detected. Waiting power negotiation and stabilization...");
                timer_set(CONNECTION_STABILIZATION_TIMEOUT);
                this.state = STATE_USB_WAIT_CONNECTION_STABLE;
            }
            break;

        case STATE_USB_WAIT_CONNECTION_STABLE:
            status_monitor(false);
            if(timer_out())
            {
                DEBUG_MSG("Reading RDO register...");
                read_power_negotation_result(true, &this.rdo);
                this.state = STATE_GETTING_POWER_PROFILE_RESULT;
            }
            break;

        case STATE_GETTING_POWER_PROFILE_RESULT:
            status_monitor(false);
            if(read_power_negotation_result(false, &this.rdo))
            {
                DEBUG_MSG("Power profile read");
                this.state = STATE_USB_CONNECTED;
            }
            break;
        
        case STATE_USB_CONNECTED:
            status_monitor(false);
            if(!BOARD_usb_detected() && !this.status.status_regs.port_status_1.bits.attach)
            {
                DEBUG_MSG("USB disconnected");
                this.state = STATE_USB_DISCONNECTED;
            }
            break;
        
        default:
            break;
    }
}