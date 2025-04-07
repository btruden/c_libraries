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
#define ENABLE_DEBUG        false
#define DEBUG_TAG           "STUSB4500"

#if ENABLE_DEBUG == true
#define DEBUG_MSG(format, ...) DEBUG_print(DEBUG_TAG, true, format, ##__VA_ARGS__)
#else
#define DEBUG_MSG(format, ...)
#endif

#define PRINT_LINE(format, ...) DEBUG_print(0, true, format, ##__VA_ARGS__)
#define PRINT(format, ...) DEBUG_print(0, false, format, ##__VA_ARGS__)

// Time defines
#define WAIT_REG_LOAD_TIMEOUT               100
#define FULL_STATUS_READ_PERIOD_MS          1000
#define FULL_STATUS_READ_ERROR_RESEND_MS    10
#define HARD_RESET_HOLD_TIME_MS             100
#define NEGOTIATION_TIME_MS                 100

// I2C defines
#define STUSB4500_ADDR                      0b0101000

// Configuration values
#define STUSB4500_ALERTS_CONFIGURATION       0xFB   // All alerts masked

// -------------------------- Power Profiles ----------------------------------
#define PDO_QTY                             3
#define PDO1_VOLTAGE_mV                     5000
#define PDO1_CURRENT_mA                     1500
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
    STUSB4500_VBUS_DISCHARGE_TIME_CTRL, 0x9C, // DISCHARGE_TIME_TO_0V is  756 ms, DISCHARGE_TIME_TRANSITION is 288 ms [default]
    STUSB4500_VBUS_DISCHARGE_CTRL, 0x00,    // VBUS discharge disabled [default]
    STUSB4500_GPIO_SW_GPIO, 0x00,           // GPIO pin value is Hi-Z [default]

    // PDO configuration
    STUSB4500_DPM_PDO_NUMB, PDO_QTY,        // Number of PDOs
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

/**
 * This command sends a "soft reset" message to the source. This is used to
 * re negotiate the power contract after a configuration is done.
 */
static uint8_t soft_reset_msg_to_src[] = 
{
    STUSB4500_TX_HEADER_LOW, 0x0D,
    STUSB4500_TX_HEADER_HIGH, 0x00,
    STUSB4500_PD_COMMAND_CTRL, 0x26
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
    STATE_USB_WAITING_NEGOTIATION,
    STATE_USB_CONNECTED,
}state_t;

/**
 * Local timers type
 */
typedef enum
{
    TIMER_MAIN = 0,
    TIMER_STATUS_READ,

    TIMER_QTY
}timer_id_t;

/******************************************************************************
 * Local Variables
 ******************************************************************************/
/**
 * Local data continer
 */
static struct local_data
{
    state_t state;                          // Main state machine state variable
    uint32_t tmr[TIMER_QTY];                // Local timers

    STUSB4500_SNK_PDO_t pdo[PDO_QTY];       // Stores the PDO configured values
    STUSB4500_RDO_t rdo;                    // Stores the RDO value
    STUSB4500_full_status_t status;         // Stores the status of the STUSB4500
}this;

/******************************************************************************
 * Local Functions
 ******************************************************************************/
/**
 * Loads the local timer with ms
 */
static void timer_set(timer_id_t tmr, uint32_t ms)
{
    if(tmr >= TIMER_QTY)
    {
        return;
    }

    this.tmr[tmr] = ms;
}

/**
 * Returns true if the timer is zero
 */
static bool timer_out(timer_id_t tmr)
{
    if(tmr >= TIMER_QTY)
    {
        return false;
    }

    return this.tmr[tmr] == 0;
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
                DEBUG_MSG("PDO %d set to %08x", pdo_id+1, pdo.d32);
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
        CONFSTATE_HARD_RESET,
        CONFSTATE_WAIT_STARTUP_REGS_LOADING,
        CONFSTATE_READ_ID,
        CONFSTATE_READING_EXISTING_SNK_PDOs,
        CONFSTATE_CLEANING_STATUS_REGS,
        CONFSTATE_WRITING_CONFIG,
        CONFSTATE_SETTING_PDO1,
        CONFSTATE_SETTING_PDO2,
        CONFSTATE_SETTING_PDO3,
        CONFSTATE_SOFT_RESET,
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
            BOARD_usb_pd_reset_assert();
            timer_set(TIMER_MAIN, HARD_RESET_HOLD_TIME_MS);
            confState = CONFSTATE_HARD_RESET;
            break;
        
        case CONFSTATE_HARD_RESET:
            if(timer_out(TIMER_MAIN))
            {
                BOARD_usb_pd_reset_release();
                timer_set(TIMER_MAIN, WAIT_REG_LOAD_TIMEOUT);
                confState = CONFSTATE_WAIT_STARTUP_REGS_LOADING;
            }
            break;
        
        case CONFSTATE_WAIT_STARTUP_REGS_LOADING:
            if(timer_out(TIMER_MAIN))
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
                /*
                DEBUG_MSG("Configuration done. Setting PDO1...");
                
                set_pdo(0, PDO1_VOLTAGE_mV, PDO1_CURRENT_mA, true);

                confState = CONFSTATE_SETTING_PDO1;
                */
               DEBUG_MSG("Configuration done. Sending Soft Reset msg to the source");
                write_multiple_registers(soft_reset_msg_to_src, sizeof(soft_reset_msg_to_src)/2, true);
                confState = CONFSTATE_SOFT_RESET;
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
                DEBUG_MSG("PDO3 set. Sending Soft Reset msg to the source");
                write_multiple_registers(soft_reset_msg_to_src, sizeof(soft_reset_msg_to_src)/2, true);
                confState = CONFSTATE_SOFT_RESET;
            }
            break;

        case CONFSTATE_SOFT_RESET:
            if(write_multiple_registers(soft_reset_msg_to_src, sizeof(soft_reset_msg_to_src)/2, false))
            {
                DEBUG_MSG("Soft reset msg sent to the source. Verifying PDO configs...");
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
 * Prints the values of the status registers of the STUSB4500
 * 
 * @param st pointer to the status structure containing the status registers
 * 
 * @param full_details true if the full details of the status registers are 
 *                      required
 */
static void print_status(STUSB4500_full_status_t *st, bool full_details)
{
#if ENABLE_DEBUG == true
    static uint32_t cnt = 0;
    DEBUG_MSG("----------------------------------------- STATUS -----------------------------------------");
    DEBUG_MSG("#%d", cnt++);
    // ALERT_STATUS_1
    DEBUG_MSG("\t* ALERT_STATUS_1: 0x%02x", st->status_regs.alert_status_1.byte);

    if(full_details)
    {
        if(st->status_regs.alert_status_1.bits.prt_status_al)
        {
            DEBUG_MSG("\t\t- PRT_STATUS_AL -> alert triggered");
        }
        if(st->status_regs.alert_status_1.bits.cc_hw_fault_status_al)
        {
            DEBUG_MSG("\t\t- CC_HW_FAULT_STATUS_AL -> alert triggered");
        }
        if(st->status_regs.alert_status_1.bits.typec_monitoring_status_al)
        {
            DEBUG_MSG("\t\t- TYPEC_MONITORING_STATUS_AL -> alert triggered");
        }
        if(st->status_regs.alert_status_1.bits.port_status_al)
        {
            DEBUG_MSG("\t\t- PORT_STATUS_AL -> alert triggered");
        }
    }

    // ALERT_STATUS_1_MASK
    DEBUG_MSG("\t* ALERT_STATUS_1_MASK: 0x%02x", st->status_regs.alert_status_1_mask.byte);
    if(full_details)
    {
        DEBUG_MSG("\t\t- PRT_STATUS_AL_MASK -> %s", (st->status_regs.alert_status_1_mask.bits.prt_status_al_msk) ? "masked" : "unmasked");
        DEBUG_MSG("\t\t- CC_FAULT_STATUS_AL_MASK -> %s", (st->status_regs.alert_status_1_mask.bits.cc_hw_fault_status_al_msk) ? "masked" : "unmasked");
        DEBUG_MSG("\t\t- TYPEC_MONITORING_STATUS_MASK -> %s", (st->status_regs.alert_status_1_mask.bits.typec_monitoring_status_al_msk) ? "masked" : "unmasked");
        DEBUG_MSG("\t\t- PORT_STATUS_AL_MASK -> %s", (st->status_regs.alert_status_1_mask.bits.port_status_al_msk) ? "masked" : "unmasked");        
    }

    // PORT_STATUS_0
    DEBUG_MSG("\t* PORT_STATUS_0: 0x%02x", st->status_regs.port_status_0.byte);
    if(full_details)
    {
        if(st->status_regs.port_status_0.bits.attach_transition)
        {
            DEBUG_MSG("\t\t- ATTACH -> attach transition detected");
        }
    }

    // PORT_STATUS_1
    DEBUG_MSG("\t* PORT_STATUS_1: 0x%02x", st->status_regs.port_status_1.byte);
    if(full_details)
    {
        DEBUG_MSG("\t\t- ATTACH -> %s", st->status_regs.port_status_1.bits.attach ? "attached" : "detached");
        DEBUG_MSG("\t\t- DATA_MODE -> %s", st->status_regs.port_status_1.bits.data_mode ? "UFP" : "[reserved]");
        DEBUG_MSG("\t\t- POWER_MODE -> %s", st->status_regs.port_status_1.bits.pwr_mode ? "[reserved]" : " device is sinking power");
        switch(st->status_regs.port_status_1.bits.attached_device)
        {
            case 0: DEBUG_MSG("\t\t- ATTACHED_DEVICE -> No device connected"); break;
            case 1: DEBUG_MSG("\t\t- ATTACHED_DEVICE -> Sink device connected"); break;
            case 3: DEBUG_MSG("\t\t- ATTACHED_DEVICE ->  Debug accessory device connected"); break; 
            case 2:
            case 4:
            case 5: DEBUG_MSG("\t\t- ATTACHED_DEVICE -> [reserved]"); break;
            default: break;
        }
    }

    // TYPEC_MONITORING_STATUS_0
    DEBUG_MSG("\t* TYPEC_MONITORING_STATUS_0: 0x%02x", st->status_regs.typec_monitoring_status_0.byte);
    if(full_details)
    {
        if(st->status_regs.typec_monitoring_status_0.bits.vbus_valid_snk_trans)
        {
            DEBUG_MSG("\t\t- VBUS_VALID_SNK_TRANS -> Transition detected on VBUS_VALID_SNK bit");
        }
        if(st->status_regs.typec_monitoring_status_0.bits.vbus_vsafe0v_trans)
        {
            DEBUG_MSG("\t\t- VBUS_VSAFE0V_TRANS -> Transition detected on VBUS_VSAFE0V bit");
        }
        if(st->status_regs.typec_monitoring_status_0.bits.vbus_ready_trans)
        {
            DEBUG_MSG("\t\t- VBUS_READY_TRANS -> Transition detected on VBUS_READY bit");
        }
        DEBUG_MSG("\t\t- VBUS_LOW_STATUS -> %s", st->status_regs.typec_monitoring_status_0.bits.vbus_low_status ? "VBUS below low threshold" : "VBUS above low threshold");
        DEBUG_MSG("\t\t- VBUS_HIGH_STATUS -> %s", st->status_regs.typec_monitoring_status_0.bits.vbus_high_status ? "VBUS above high threshold" : "VBUS below high threshold");    
    }

    // TYPEC_MONITORING_STATUS_1
    DEBUG_MSG("\t* TYPEC_MONITORING_STATUS_1: 0x%02x", st->status_regs.typec_monitoring_status_1.byte);
    if(full_details)
    {
        DEBUG_MSG("\t\t- VBUS_VALID_SNK -> %s", st->status_regs.typec_monitoring_status_1.bits.vbus_valid_snk ? "VBUS is higher than 1.9 V or 3.5 V" : "VBUS is lower than 1.9 V or 3.5 V");
        DEBUG_MSG("\t\t- VBUS_VSAFE0V -> %s", st->status_regs.typec_monitoring_status_1.bits.vbus_vsafe0v ? "VBUS is lower than 0.8 V" : "VBUS is higher than 0.8 V");
        DEBUG_MSG("\t\t- VBUS_READY -> %s", st->status_regs.typec_monitoring_status_1.bits.vbus_ready ? "VBUS connected" : " VBUS disconnected");    
    }

    // CC_STATUS
    DEBUG_MSG("\t* CC_STATUS: 0x%02x", st->status_regs.cc_status.byte);
    if(full_details)
    {
        switch(st->status_regs.cc_status.bits.cc1_state)
        {
            case 0: DEBUG_MSG("\t\t- CC1_STATE -> [reserved]"); break;
            case 1: DEBUG_MSG("\t\t- CC1_STATE -> SNK.Default"); break;
            case 2: DEBUG_MSG("\t\t- CC1_STATE -> SNK.Power1.5"); break;
            case 3: DEBUG_MSG("\t\t- CC1_STATE -> SNK.Power3.0"); break;
            default: break;
        }
        switch(st->status_regs.cc_status.bits.cc2_state)
        {
            case 0: DEBUG_MSG("\t\t- CC2_STATE -> [reserved]"); break;
            case 1: DEBUG_MSG("\t\t- CC2_STATE -> SNK.Default"); break;
            case 2: DEBUG_MSG("\t\t- CC2_STATE -> SNK.Power1.5"); break;
            case 3: DEBUG_MSG("\t\t- CC2_STATE -> SNK.Power3.0"); break;
            default: break;
        }
        DEBUG_MSG("\t\t- CONNECT_RESULT -> %s", st->status_regs.cc_status.bits.connect_result ? "The device is presenting Rd" : "[reserved]");
        DEBUG_MSG("\t\t- LOOKING_4_CONNECTION -> %s", st->status_regs.cc_status.bits.looking_for_connection ? " looking 4 connection" : "NOT looking 4 connection");    
    }

    // CC_HW_FAULT_STATUS_0
    DEBUG_MSG("\t* CC_HW_FAULT_STATUS_0: 0x%02x", st->status_regs.cc_hw_fault_status_0.byte);
    if(full_details)
    {
        if(st->status_regs.cc_hw_fault_status_0.bits.vpu_valid_trans)
        {
            DEBUG_MSG("\t\t- VPU_VALID_TRANS -> Transition occurred on VPU_VALID bit");
        }   
        if(st->status_regs.cc_hw_fault_status_0.bits.vpu_ovp_fault_trans)
        {
            DEBUG_MSG("\t\t- VPU_OVP_FAULT_TRANS -> Transition occurred on VPU_OVP_FAULT bit");
        }    
    }

    // CC_HW_FAULT_STATUS_1
    DEBUG_MSG("\t* CC_HW_FAULT_STATUS_1: 0x%02x", st->status_regs.cc_hw_fault_status_1.byte);
    if(full_details)
    {
        DEBUG_MSG("\t\t- VBUS_DISCH_FAULT -> %s", st->status_regs.cc_hw_fault_status_1.bits.vbus_disch_fault ? "VBUS discharge issue has occurred" : "No VBUS discharge issue");
        DEBUG_MSG("\t\t- VPU_VALID -> %s", st->status_regs.cc_hw_fault_status_1.bits.vpu_valid ? 
            " CC pins pull-up voltage is above UVLO threshold of 2.8 V when in pull-up mode" : 
            "CC pins pull-up voltage is below UVLO threshold of 2.8 V when in pull-up mode");
        DEBUG_MSG("\t\t- VPU_OVP_FAULT -> %s", st->status_regs.cc_hw_fault_status_1.bits.vpu_ovp_fault ? 
            "Overvoltage condition has occurred on CC pins when in pull-up mode" : 
            "No overvoltage condition on CC pins when in pull-up mode");    
    }
    
    // PD_TYPEC_STATUS
    DEBUG_MSG("\t* PD_TYPEC_STATUS: 0x%02x", st->status_regs.pd_typec_status.byte);
    if(full_details)
    {
        switch(st->status_regs.pd_typec_status.bits.pd_typec_hand_check)
        {
            case 0: DEBUG_MSG("\t\t- PD_TYPEC_HAND_CHECK -> Cleared"); break;
            case 8: DEBUG_MSG("\t\t- PD_TYPEC_HAND_CHECK -> PD_HARD_RESET_COMPLETE_ACK"); break;
            case 14: DEBUG_MSG("\t\t- PD_TYPEC_HAND_CHECK -> PD_HARD_RESET_RECEIVED_ACK"); break;
            case 15: DEBUG_MSG("\t\t- PD_TYPEC_HAND_CHECK -> PD_HARD_RESET_SEND_ACK"); break;
            default: break;
        }    
    }

    // TYPEC_STATUS
    DEBUG_MSG("\t* TYPEC_STATUS: 0x%02x", st->status_regs.typec_status.byte);
    if(full_details)
    {
        switch(st->status_regs.typec_status.bits.typec_fsm_state)
        {
            case 0: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> UNATTACHED_SNK"); break;
            case 1: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> ATTACHWAIT_SNK"); break;
            case 2: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> ATTACHED_SNK"); break;
            case 3: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> DEBUGACCESSORY_SNK"); break;
            case 12: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> TRY_SRC"); break;
            case 13: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> UNATTACHED_ACCESSORY"); break;
            case 14: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> ATTACHWAIT_ACCESSORY"); break;
            case 19: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> TYPEC_ERRORRECOVERY"); break;
            default: DEBUG_MSG("\t\t- TYPEC_FSM_STATE -> [reserved]"); break;
        }    
    }

    // PRT_STATUS
    DEBUG_MSG("\t* PRT_STATUS: 0x%02x", st->status_regs.prt_status.byte);
    if(full_details)
    {
        DEBUG_MSG("\t\t- PRL_HW_RST_RECEIVED -> %s", st->status_regs.prt_status.bits.prl_hw_rst_received ? 
            " Interrupt for a PD hardware reset request coming from RX" : 
            "Cleared by I2C maste3");
        DEBUG_MSG("\t\t- PRL_MSG_RECEIVED -> %s", st->status_regs.prt_status.bits.prl_msg_received ? 
            "Interrupt for protocol layer message received" :
            "Cleared by I2C master");    
    }
    DEBUG_MSG("------------------------------------------------------------------------------------------");
    
    DEBUG_MSG(""); 
    DEBUG_MSG(""); 
#endif
}

/**
 * Prints the values of the RDO registers of the STUSB4500
 * 
 * @param r pointer to the RDO structure containing the RDO registers
 * @param full_details true if the full details of the RDO registers are
 *                      required
 */
static void print_rdo(STUSB4500_RDO_t *r, bool full_details)
{
#if ENABLE_DEBUG == true

    DEBUG_MSG("-------------------------------------- RDO STATUS ----------------------------------------");
    DEBUG_MSG("\t* RDO_REG_STATUS_0_3: 0x%08x", r->d32);
    if(full_details)
    {
        DEBUG_MSG("\t\t- max_operating_current -> %d mA", r->bits.max_operating_current * 10);
        DEBUG_MSG("\t\t- operating_current -> %d mA", r->bits.operating_current * 10);
        DEBUG_MSG("\t\t- unchunked_ext_msg_supported -> %d", r->bits.unchunked_ext_msg_supported);
        DEBUG_MSG("\t\t- no_usb_suspend -> %d", r->bits.no_usb_suspend);
        DEBUG_MSG("\t\t- usb_comm_capable -> %d", r->bits.usb_comm_capable);
        DEBUG_MSG("\t\t- capability_mismatch -> %d", r->bits.capability_mismatch);
        DEBUG_MSG("\t\t- give_back_flag -> %d", r->bits.give_back_flag);
        DEBUG_MSG("\t\t- object_position -> %d (%s)", r->bits.object_position, 
            (r->bits.object_position > 0)? "USB PD mode" : "USB C mode (NO USB PD mode!!)");
    }
    DEBUG_MSG("------------------------------------------------------------------------------------------");
    
    DEBUG_MSG(""); 
    DEBUG_MSG(""); 

#endif
}

/**
 * Reads the complete set of status registers form the chip periodically
 * 
 * @param init true if the function is called for the first time
 */
static void status_monitor(bool init)
{
    static STUSB4500_RDO_t rdo;
    static STUSB4500_full_status_t status; 

    static enum
    {
        STATUS_STATE_INIT = 0,
        STATUS_STATE_READING_STATUS, 
        STATUS_STATE_READING_RDO
    }statusState = STATUS_STATE_INIT;

    if(init)
    {
        statusState = STATUS_STATE_INIT;
        memset(&this.status, 0, sizeof(this.status)); 
        memset(&status, 0, sizeof(status)); 
        memset(&this.rdo, 0, sizeof(this.rdo)); 
        memset(&rdo, 0, sizeof(rdo)); 
    }

    switch(statusState)
    {
        case STATUS_STATE_INIT:
            if(timer_out(TIMER_STATUS_READ))
            {
                read_consecutive_registers(status.bytes, STUSB4500_ALERT_STATUS_1, sizeof(STUSB4500_full_status_t), true);
                timer_set(TIMER_STATUS_READ, FULL_STATUS_READ_PERIOD_MS);
                statusState = STATUS_STATE_READING_STATUS;
            }
            
            break;

        case STATUS_STATE_READING_STATUS:
            if(read_consecutive_registers(status.bytes, STUSB4500_ALERT_STATUS_1, sizeof(STUSB4500_full_status_t), false))
            {
                // Check for changes
                if(memcmp(this.status.bytes, status.bytes, sizeof(STUSB4500_full_status_t)) != 0)
                {
                    memcpy(this.status.bytes, status.bytes, sizeof(STUSB4500_full_status_t));
                    print_status(&this.status, true);
                }

                read_consecutive_registers(rdo.bytes, STUSB4500_RDO_REG_STATUS_0, sizeof(STUSB4500_RDO_t), true);
                statusState = STATUS_STATE_READING_RDO;
            }

            break;

        case STATUS_STATE_READING_RDO:
            if(read_consecutive_registers(rdo.bytes, STUSB4500_RDO_REG_STATUS_0, sizeof(STUSB4500_RDO_t), false))
            {
                // Check for changes
                if(memcmp(this.rdo.bytes, rdo.bytes, sizeof(STUSB4500_RDO_t)) != 0)
                {
                    memcpy(this.rdo.bytes, rdo.bytes, sizeof(STUSB4500_RDO_t));
                    print_rdo(&this.rdo, true);
                }

                statusState = STATUS_STATE_INIT;
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
    for(int i = 0; i < TIMER_QTY; i++)
    {
        if(this.tmr[i])
            this.tmr[i]--;
    }
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
    memset(this.tmr, 0, sizeof(this.tmr));
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
                timer_set(TIMER_MAIN, NEGOTIATION_TIME_MS);
                this.state = STATE_USB_WAITING_NEGOTIATION;
            }
            break;

        case STATE_USB_WAITING_NEGOTIATION:
            status_monitor(false);
            if(timer_out(TIMER_MAIN))
            {   
                DEBUG_MSG("Connected");
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