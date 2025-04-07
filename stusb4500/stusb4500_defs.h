/***************************************************************************//**
 * @file        stusb4500_defs.h
 * @brief       Provides the necessary register definitions for the STUSB4500
 * @author      Blas Truden
 * @date        20250117
 * @version     v1
 * 
 * @copyright   -
 * 
 * @details     This module is part of the BSI BSP core.
 ******************************************************************************/
#ifndef STUSB4500_DEFS_H
#define STUSB4500_DEFS_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Public Defines
 ******************************************************************************/
/* Register Addresses */
#define STUSB4500_BCD_TYPEC_REV_LOW      0x06
#define STUSB4500_BCD_TYPEC_REV_HIGH     0x07
#define STUSB4500_BCD_USBPD_REV_LOW      0x08
#define STUSB4500_BCD_USBPD_REV_HIGH     0x09
#define STUSB4500_DEVICE_CAPAB_HIGH      0x0A
#define STUSB4500_ALERT_STATUS_1         0x0B
#define STUSB4500_ALERT_STATUS_1_MASK    0x0C
#define STUSB4500_PORT_STATUS_0          0x0D
#define STUSB4500_PORT_STATUS_1          0x0E
#define STUSB4500_TYPEC_MONITORING_STATUS_0 0x0F
#define STUSB4500_TYPEC_MONITORING_STATUS_1 0x10
#define STUSB4500_CC_STATUS              0x11
#define STUSB4500_CC_HW_FAULT_STATUS_0   0x12
#define STUSB4500_CC_HW_FAULT_STATUS_1   0x13
#define STUSB4500_PD_TYPEC_STATUS        0x14
#define STUSB4500_TYPEC_STATUS           0x15
#define STUSB4500_PRT_STATUS             0x16
#define STUSB4500_PD_COMMAND_CTRL        0x1A
#define STUSB4500_MONITORING_CTRL_0      0x20
#define STUSB4500_MONITORING_CTRL_2      0x22
#define STUSB4500_RESET_CTRL             0x23
#define STUSB4500_VBUS_DISCHARGE_TIME_CTRL 0x25
#define STUSB4500_VBUS_DISCHARGE_CTRL    0x26
#define STUSB4500_VBUS_CTRL              0x27
#define STUSB4500_PE_FSM                 0x29
#define STUSB4500_GPIO_SW_GPIO           0x2D
#define STUSB4500_DEVICE_ID              0x2F
#define STUSB4500_RX_HEADER_LOW          0x31
#define STUSB4500_RX_HEADER_HIGH         0x32
#define STUSB4500_TX_HEADER_LOW          0x51
#define STUSB4500_TX_HEADER_HIGH         0x52
#define STUSB4500_DPM_PDO_NUMB           0x70
#define STUSB4500_DPM_SNK_PDO1_0         0x85
#define STUSB4500_DPM_SNK_PDO1_1         0x86
#define STUSB4500_DPM_SNK_PDO1_2         0x87
#define STUSB4500_DPM_SNK_PDO1_3         0x88
#define STUSB4500_DPM_SNK_PDO2_0         0x89
#define STUSB4500_DPM_SNK_PDO2_1         0x8A
#define STUSB4500_DPM_SNK_PDO2_2         0x8B
#define STUSB4500_DPM_SNK_PDO2_3         0x8C
#define STUSB4500_DPM_SNK_PDO3_0         0x8D
#define STUSB4500_DPM_SNK_PDO3_1         0x8E
#define STUSB4500_DPM_SNK_PDO3_2         0x8F
#define STUSB4500_DPM_SNK_PDO3_3         0x90
#define STUSB4500_RDO_REG_STATUS_0       0x91
#define STUSB4500_RDO_REG_STATUS_1       0x92
#define STUSB4500_RDO_REG_STATUS_2       0x93
#define STUSB4500_RDO_REG_STATUS_3       0x94

/* ALERT_STATUS_1 Register Bits */
#define STUSB4500_ALERT_STATUS_PORT      (1 << STUSB4500_ALERT_STATUS_PORT_SHIFT)
#define STUSB4500_ALERT_STATUS_PORT_SHIFT 6
#define STUSB4500_ALERT_STATUS_MONITOR   (1 << STUSB4500_ALERT_STATUS_MONITOR_SHIFT)
#define STUSB4500_ALERT_STATUS_MONITOR_SHIFT 5
#define STUSB4500_ALERT_STATUS_HW_FAULT  (1 << STUSB4500_ALERT_STATUS_HW_FAULT_SHIFT)
#define STUSB4500_ALERT_STATUS_HW_FAULT_SHIFT 4
#define STUSB4500_ALERT_STATUS_PRT       (1 << STUSB4500_ALERT_STATUS_PRT_SHIFT)
#define STUSB4500_ALERT_STATUS_PRT_SHIFT 1
#define STUSB4500_ALERT_STATUS_MASK_ALL  0x7E

/* ALERT_STATUS_1 Register Bit Values */
#define STUSB4500_ALERT_UNMASKED  0
#define STUSB4500_ALERT_MASKED    1

/* PORT_STATUS_1 Register Bits */
#define STUSB4500_PORT_STATUS_1_ATTACHED  (1 << STUSB4500_PORT_STATUS_1_ATTACHED_SHIFT)
#define STUSB4500_PORT_STATUS_1_ATTACHED_SHIFT 0
#define STUSB4500_PORT_STATUS_1_DATA_MODE  (1 << STUSB4500_PORT_STATUS_1_DATA_MODE_SHIFT)
#define STUSB4500_PORT_STATUS_1_DATA_MODE_SHIFT 2
#define STUSB4500_PORT_STATUS_1_POWER_MODE  (1 << STUSB4500_PORT_STATUS_1_POWER_MODE_SHIFT)
#define STUSB4500_PORT_STATUS_1_POWER_MODE_SHIFT 3
#define STUSB4500_PORT_STATUS_1_ATTACHED_DEVICE  (1 << STUSB4500_PORT_STATUS_1_ATTACHED_DEVICE_SHIFT)
#define STUSB4500_PORT_STATUS_1_ATTACHED_DEVICE_SHIFT 5

/* TYPEC_MONITORING_STATUS_0 Register Bits */
#define STUSB4500_VBUS_HIGH_STATUS       (1 << STUSB4500_VBUS_HIGH_STATUS_SHIFT)
#define STUSB4500_VBUS_HIGH_STATUS_SHIFT 5
#define STUSB4500_VBUS_LOW_STATUS        (1 << STUSB4500_VBUS_LOW_STATUS_SHIFT)
#define STUSB4500_VBUS_LOW_STATUS_SHIFT  4
#define STUSB4500_VBUS_READY_TRANS       (1 << STUSB4500_VBUS_READY_TRANS_SHIFT)
#define STUSB4500_VBUS_READY_TRANS_SHIFT 3
#define STUSB4500_VBUS_VSAFE0V_TRANS     (1 << STUSB4500_VBUS_VSAFE0V_TRANS_SHIFT)
#define STUSB4500_VBUS_VSAFE0V_TRANS_SHIFT 2
#define STUSB4500_VBUS_VALID_SNK_TRANS   (1 << STUSB4500_VBUS_VALID_SNK_TRANS_SHIFT)
#define STUSB4500_VBUS_VALID_SNK_TRANS_SHIFT 1
#define STUSB4500_TYPEC_MONITORING_STATUS_0_MASK_ALL  0x3E

/* TYPEC_MONITORING_STATUS_0 Bit Values */
#define STUSB4500_VBUS_HIGH_OK                      0
#define STUSB4500_VBUS_HIGH_OVERVOLTAGE             1
#define STUSB4500_VBUS_LOW_OK                       0
#define STUSB4500_VBUS_LOW_UNDERVOLTAGE             1
#define STUSB4500_VBUS_READY_NO_TRANSITION          0
#define STUSB4500_VBUS_READY_TRANSITION_DETECTED    1
#define STUSB4500_VSAFE0V_NO_TRANSITION             0
#define STUSB4500_VSAFE0V_TRANSITION_DETECTED       1
#define STUSB4500_VALID_SNK_NO_TRANSITION           0
#define STUSB4500_VALID_SNK_TRANSITION_DETECTED     1

/* TYPEC_MONITORING_STATUS_1 Register Bits */
#define STUSB4500_VBUS_READY             (1 << STUSB4500_VBUS_READY_SHIFT)
#define STUSB4500_VBUS_READY_SHIFT       3
#define STUSB4500_VBUS_VSAFE0V           (1 << STUSB4500_VBUS_VSAFE0V_SHIFT)
#define STUSB4500_VBUS_VSAFE0V_SHIFT     2
#define STUSB4500_VBUS_VALID_SNK         (1 << STUSB4500_VBUS_VALID_SNK_SHIFT)
#define STUSB4500_VBUS_VALID_SNK_SHIFT   1
#define STUSB4500_TYPEC_MONITORING_STATUS_1_MASK_ALL  0x0E

/* TYPEC_MONITORING_STATUS_1 Bit Values */
#define STUSB4500_VBUS_NOT_READY         0
#define STUSB4500_VBUS_IS_READY          1
#define STUSB4500_VBUS_NOT_SAFE0V        0
#define STUSB4500_VBUS_IS_SAFE0V         1
#define STUSB4500_VBUS_NOT_VALID_SNK     0
#define STUSB4500_VBUS_IS_VALID_SNK      1

/* CC_STATUS Register Bits */
#define STUSB4500_CC_LOOKING_FOR_CONN    (1 << STUSB4500_CC_LOOKING_FOR_CONN_SHIFT)
#define STUSB4500_CC_LOOKING_FOR_CONN_SHIFT 5
#define STUSB4500_CC_CONNECT_RESULT      (1 << STUSB4500_CC_CONNECT_RESULT_SHIFT)
#define STUSB4500_CC_CONNECT_RESULT_SHIFT 4
#define STUSB4500_CC_STATE_MASK          0x3C

/* CC_STATUS Bit Values */
#define STUSB4500_CC_NOT_LOOKING         0
#define STUSB4500_CC_LOOKING             1
#define STUSB4500_CC_NO_CONNECTION       0
#define STUSB4500_CC_PRESENT_RD          1

/* Reset Control Register Bits */
#define STUSB4500_RESET_SW_ENABLE        (1 << STUSB4500_RESET_SW_ENABLE_SHIFT)
#define STUSB4500_RESET_SW_ENABLE_SHIFT  0
#define STUSB4500_RESET_CTRL_MASK_ALL    0x01

/* Reset Control Bit Values */
#define STUSB4500_SW_RESET_DISABLED      0
#define STUSB4500_SW_RESET_ENABLED       1

/* GPIO Control Register Bits */
#define STUSB4500_GPIO_SW_GPIO_ENABLE    (1 << STUSB4500_GPIO_SW_GPIO_ENABLE_SHIFT)
#define STUSB4500_GPIO_SW_GPIO_ENABLE_SHIFT 0
#define STUSB4500_GPIO_CTRL_MASK_ALL     0x01

/* GPIO Control Bit Values */
#define STUSB4500_GPIO_DISABLED          0
#define STUSB4500_GPIO_ENABLED           1

/* Device ID */
#define STUSB4500_DEVICE_ID_VALUE        0x25

/******************************************************************************
 * Public Types
 ******************************************************************************/
/* Register Structures */

// --------------------------- Status registers ------------------------------
typedef union {
    struct {
        uint8_t reserved : 1;
        uint8_t prt_status_al : 1;
        uint8_t reserved2 : 2;
        uint8_t cc_hw_fault_status_al : 1;
        uint8_t typec_monitoring_status_al : 1;
        uint8_t port_status_al : 1;
        uint8_t reserved3 : 1;
    } bits;
    uint8_t byte;
} STUSB4500_AlertStatus1_t;

typedef union {
    struct {
        uint8_t reserved : 1;
        uint8_t prt_status_al_msk : 1;
        uint8_t reserved2 : 2;
        uint8_t cc_hw_fault_status_al_msk : 1;
        uint8_t typec_monitoring_status_al_msk : 1;
        uint8_t port_status_al_msk : 1;
        uint8_t reserved3 : 1;
    } bits;
    uint8_t byte;
} STUSB4500_AlertStatus1Mask_t;

typedef union {
    struct {
        uint8_t attach_transition :1;
        uint8_t reserved :7;
    } bits;
    uint8_t byte;
} STUSB4500_PortStatus0_t;

typedef union {
    struct {
        uint8_t attach : 1;
        uint8_t reserved1 : 1;
        uint8_t data_mode :1;
        uint8_t pwr_mode :1;
        uint8_t reserved2 :1;
        uint8_t attached_device :3;
    } bits;
    uint8_t byte;
} STUSB4500_PortStatus1_t;

typedef union {
    struct {
        uint8_t reserved : 1;
        uint8_t vbus_valid_snk_trans : 1;
        uint8_t vbus_vsafe0v_trans : 1;
        uint8_t vbus_ready_trans : 1;
        uint8_t vbus_low_status : 1;
        uint8_t vbus_high_status : 1;
        uint8_t reserved2 : 2;
    } bits;
    uint8_t byte;
} STUSB4500_TypeCMonitoringStatus0_t;

typedef union {
    struct {
        uint8_t reserved : 1;
        uint8_t vbus_valid_snk : 1;
        uint8_t vbus_vsafe0v : 1;
        uint8_t vbus_ready : 1;
        uint8_t reserved2 : 4;
    } bits;
    uint8_t byte;
} STUSB4500_TypeCMonitoringStatus1_t;

typedef union {
    struct {
        uint8_t cc1_state : 2;
        uint8_t cc2_state : 2;
        uint8_t connect_result : 1;
        uint8_t looking_for_connection : 1;
        uint8_t reserved : 2;
    } bits;
    uint8_t byte;
} STUSB4500_CCStatus_t;

typedef union {
    struct {
        uint8_t reserved : 4;
        uint8_t vpu_valid_trans : 1;
        uint8_t vpu_ovp_fault_trans : 1;
        uint8_t reserved2 : 2;
    } bits;
    uint8_t byte;
} STUSB4500_CCHwFaultStatus0_t;

typedef union {
    struct {
        uint8_t reserved : 4;
        uint8_t vbus_disch_fault : 1;
        uint8_t reserved2 : 1;
        uint8_t vpu_valid : 1;
        uint8_t vpu_ovp_fault : 1;
    } bits;
    uint8_t byte;
} STUSB4500_CCHwFaultStatus1_t;

typedef union {
    struct {
        uint8_t pd_typec_hand_check : 4;
        uint8_t reserved : 4;
    } bits;
    uint8_t byte;
} STUSB4500_PDTypeCStatus_t;

typedef union {
    struct {
        uint8_t typec_fsm_state : 5;
        uint8_t reserved : 3;
    } bits;
    uint8_t byte;
} STUSB4500_TypeCStatus_t;

typedef union {
    struct {
        uint8_t prl_hw_rst_received: 1;
        uint8_t reserved : 1;
        uint8_t prl_msg_received : 1;
        uint8_t reserve2 : 1;
        uint8_t prt_bist_received : 1;
        uint8_t reserve3 : 3;
    } bits;
    uint8_t byte;
} STUSB4500_PrtStatus_t;

/**
 * Complete set of status registers
 */
typedef union
{
    uint8_t bytes[12];
    struct
    {
        STUSB4500_AlertStatus1_t alert_status_1;
        STUSB4500_AlertStatus1Mask_t alert_status_1_mask;
        STUSB4500_PortStatus0_t port_status_0;
        STUSB4500_PortStatus1_t port_status_1;
        STUSB4500_TypeCMonitoringStatus0_t typec_monitoring_status_0;
        STUSB4500_TypeCMonitoringStatus1_t typec_monitoring_status_1;
        STUSB4500_CCStatus_t cc_status;
        STUSB4500_CCHwFaultStatus0_t cc_hw_fault_status_0;
        STUSB4500_CCHwFaultStatus1_t cc_hw_fault_status_1;
        STUSB4500_PDTypeCStatus_t pd_typec_status;
        STUSB4500_TypeCStatus_t typec_status;
        STUSB4500_PrtStatus_t prt_status;     
    }status_regs;
}STUSB4500_full_status_t;

// --------------------------- Control registers ------------------------------
typedef union {
    struct {
        uint8_t reserved : 1;
        uint8_t reserved2 : 7;
    } bits;
    uint8_t byte;
} STUSB4500_ResetCtrl_t;

typedef union {
    struct {
        uint8_t reserved : 1;
        uint8_t reserved2 : 7;
    } bits;
    uint8_t byte;
} STUSB4500_GPIOCtrl_t;

// ----------------------------- Custom Types -------------------------------
/**
 * SNK PDO register structure
 */
typedef union
{
    uint32_t d32;
    uint8_t d8[4];
    
    struct
    {
    uint32_t Operationnal_Current :10;
    uint32_t Voltage :10;
    uint32_t Reserved_22_20  :3;
    uint32_t Fast_Role_Req_cur : 2;  /* must be set to 0 in 2.0*/
    uint32_t Dual_Role_Data    :1;
    uint32_t USB_Communications_Capable :1;
    uint32_t Unconstrained_Power :1;
    uint32_t Higher_Capability :1;
    uint32_t Dual_Role_Power :1;
    uint32_t Fixed_Supply :2;
    
    }fix;

    struct
    {
    uint32_t Operating_Current :10;
    uint32_t Min_Voltage:10;
    uint32_t Max_Voltage:10;
    uint32_t VariableSupply:2; 
    }var;
    
    struct
    {
    uint32_t Operating_Power :10;
    uint32_t Min_Voltage:10;
    uint32_t Max_Voltage:10;
    uint32_t Battery:2; 
    }bat;     

}STUSB4500_SNK_PDO_t;

/**
 * RDO register structure
 */
typedef union
{
  uint32_t d32;
  uint8_t bytes[4];
  struct
  {     
        uint32_t max_operating_current          :       10; //Bits 9..0
        uint32_t operating_current              :       10;
        uint8_t reserved_22_20                  :       3;
        uint8_t unchunked_ext_msg_supported     :       1;
        uint8_t no_usb_suspend                  :       1;
        uint8_t usb_comm_capable                :       1;
        uint8_t capability_mismatch             :       1;
        uint8_t give_back_flag                  :       1;
        uint8_t object_position                 :       3; //Bits 30..28 (3-bit)
        uint8_t reserved_31		                :	    1; //Bits 31
        
  } bits;
} STUSB4500_RDO_t;

#endif // STUSB4500_DEFS_H