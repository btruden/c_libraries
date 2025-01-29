/***************************************************************************//**
 * @file        tca8418_defs.h
 * @brief       Provides the necessary register definitions for the TCA8418
 * @author      Blas Truden
 * @date        20250128
 * @version     v1
 * 
 * @copyright   -
 * 
 * @details     This module is part of the BSI BSP core.
 ******************************************************************************/
#ifndef TCA8418_DEFS_H
#define TCA8418_DEFS_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * Public Defines
 ******************************************************************************/
/* I2C chip address */
#define TCA8418_I2C_ADDR 0x34  // 7-bit I2C address

/* Register Addresses */
#define TCA8418_REG_CFG                 0x01
#define TCA8418_REG_INT_STAT            0x02
#define TCA8418_REG_KEY_LCK_EC          0x03
#define TCA8418_REG_KEY_EVENT_A         0x04
#define TCA8418_REG_KEY_EVENT_B         0x05
#define TCA8418_REG_KEY_EVENT_C         0x06
#define TCA8418_REG_KEY_EVENT_D         0x07
#define TCA8418_REG_KEY_EVENT_E         0x08
#define TCA8418_REG_KEY_EVENT_F         0x09
#define TCA8418_REG_KEY_EVENT_G         0x0A
#define TCA8418_REG_KEY_EVENT_H         0x0B
#define TCA8418_REG_KEY_EVENT_I         0x0C
#define TCA8418_REG_KEY_EVENT_J         0x0D
#define TCA8418_REG_KP_LCK_TIMER        0x0E
#define TCA8418_REG_UNLOCK1             0x0F
#define TCA8418_REG_UNLOCK2             0x10
#define TCA8418_REG_GPIO_INT_STAT1      0x11
#define TCA8418_REG_GPIO_INT_STAT2      0x12
#define TCA8418_REG_GPIO_INT_STAT3      0x13
#define TCA8418_REG_GPIO_DAT_STAT1      0x14
#define TCA8418_REG_GPIO_DAT_STAT2      0x15
#define TCA8418_REG_GPIO_DAT_STAT3      0x16
#define TCA8418_REG_GPIO_DAT_OUT1       0x17
#define TCA8418_REG_GPIO_DAT_OUT2       0x18
#define TCA8418_REG_GPIO_DAT_OUT3       0x19
#define TCA8418_REG_GPIO_INT_EN1        0x1A
#define TCA8418_REG_GPIO_INT_EN2        0x1B
#define TCA8418_REG_GPIO_INT_EN3        0x1C
#define TCA8418_REG_KP_GPIO1            0x1D
#define TCA8418_REG_KP_GPIO2            0x1E
#define TCA8418_REG_KP_GPIO3            0x1F
#define TCA8418_REG_GPI_EM1             0x20
#define TCA8418_REG_GPI_EM2             0x21
#define TCA8418_REG_GPI_EM3             0x22
#define TCA8418_REG_GPIO_DIR1           0x23
#define TCA8418_REG_GPIO_DIR2           0x24
#define TCA8418_REG_GPIO_DIR3           0x25
#define TCA8418_REG_GPIO_INT_LVL1       0x26
#define TCA8418_REG_GPIO_INT_LVL2       0x27
#define TCA8418_REG_GPIO_INT_LVL3       0x28
#define TCA8418_REG_DEBOUNCE_DIS1       0x29
#define TCA8418_REG_DEBOUNCE_DIS2       0x2A
#define TCA8418_REG_DEBOUNCE_DIS3       0x2B
#define TCA8418_REG_GPIO_PULL1          0x2C
#define TCA8418_REG_GPIO_PULL2          0x2D
#define TCA8418_REG_GPIO_PULL3          0x2E

/* Configuration Register (0x01) Bit Shifts */
#define TCA8418_CFG_AI_SHIFT            7
#define TCA8418_CFG_GPI_E_CFG_SHIFT     6
#define TCA8418_CFG_OVR_FLOW_M_SHIFT    5
#define TCA8418_CFG_INT_CFG_SHIFT       4
#define TCA8418_CFG_OVR_FLOW_IEN_SHIFT  3
#define TCA8418_CFG_K_LCK_IEN_SHIFT     2
#define TCA8418_CFG_GPI_IEN_SHIFT       1
#define TCA8418_CFG_KE_IEN_SHIFT        0

/* Configuration Register (0x01) Bit Masks */
#define TCA8418_CFG_AI_MASK             (1U << TCA8418_CFG_AI_SHIFT)
#define TCA8418_CFG_GPI_E_CFG_MASK      (1U << TCA8418_CFG_GPI_E_CFG_SHIFT)
#define TCA8418_CFG_OVR_FLOW_M_MASK     (1U << TCA8418_CFG_OVR_FLOW_M_SHIFT)
#define TCA8418_CFG_INT_CFG_MASK        (1U << TCA8418_CFG_INT_CFG_SHIFT)
#define TCA8418_CFG_OVR_FLOW_IEN_MASK   (1U << TCA8418_CFG_OVR_FLOW_IEN_SHIFT)
#define TCA8418_CFG_K_LCK_IEN_MASK      (1U << TCA8418_CFG_K_LCK_IEN_SHIFT)
#define TCA8418_CFG_GPI_IEN_MASK        (1U << TCA8418_CFG_GPI_IEN_SHIFT)
#define TCA8418_CFG_KE_IEN_MASK         (1U << TCA8418_CFG_KE_IEN_SHIFT)

/* Interrupt Status Register (0x02) Bit Shifts */
#define TCA8418_INT_STAT_CAD_INT_SHIFT      4
#define TCA8418_INT_STAT_OVR_FLOW_INT_SHIFT 3
#define TCA8418_INT_STAT_K_LCK_INT_SHIFT    2
#define TCA8418_INT_STAT_GPI_INT_SHIFT      1
#define TCA8418_INT_STAT_K_INT_SHIFT        0

/* Interrupt Status Register (0x02) Bit Masks */
#define TCA8418_INT_STAT_CAD_INT_MASK       (1U << TCA8418_INT_STAT_CAD_INT_SHIFT)
#define TCA8418_INT_STAT_OVR_FLOW_INT_MASK  (1U << TCA8418_INT_STAT_OVR_FLOW_INT_SHIFT)
#define TCA8418_INT_STAT_K_LCK_INT_MASK     (1U << TCA8418_INT_STAT_K_LCK_INT_SHIFT)
#define TCA8418_INT_STAT_GPI_INT_MASK       (1U << TCA8418_INT_STAT_GPI_INT_SHIFT)
#define TCA8418_INT_STAT_K_INT_MASK         (1U << TCA8418_INT_STAT_K_INT_SHIFT)

/* Key Lock and Event Counter Register (0x03) Bit Shifts */
#define TCA8418_KEY_LCK_EC_K_LCK_EN_SHIFT   6
#define TCA8418_KEY_LCK_EC_LCK2_SHIFT       5
#define TCA8418_KEY_LCK_EC_LCK1_SHIFT       4
#define TCA8418_KEY_LCK_EC_KEC_SHIFT        0

/* Key Lock and Event Counter Register (0x03) Bit Masks */
#define TCA8418_KEY_LCK_EC_K_LCK_EN_MASK    (1U << TCA8418_KEY_LCK_EC_K_LCK_EN_SHIFT)
#define TCA8418_KEY_LCK_EC_LCK2_MASK        (1U << TCA8418_KEY_LCK_EC_LCK2_SHIFT)
#define TCA8418_KEY_LCK_EC_LCK1_MASK        (1U << TCA8418_KEY_LCK_EC_LCK1_SHIFT)
#define TCA8418_KEY_LCK_EC_KEC_MASK         (0x0FU << TCA8418_KEY_LCK_EC_KEC_SHIFT)

/* Key Event A Register (0x04) Bit Masks */
#define TCA8418_KEY_EVENT_A_KEY_PRESS_MASK  (1U << TCA8418_KEY_EVENT_A_KEY_PRESS_SHIFT)
#define TCA8418_KEY_EVENT_A_KEY_ID_MASK     (0x7FU << TCA8418_KEY_EVENT_A_KEY_ID_SHIFT)

/* Key Event A Register (0x04) Bit Shifts */
#define TCA8418_KEY_EVENT_A_KEY_PRESS_SHIFT 7
#define TCA8418_KEY_EVENT_A_KEY_ID_SHIFT    0

/******************************************************************************
 * Public Types
 ******************************************************************************/
/* Register Structures */
typedef union {
    struct {
        uint8_t ke_ien:1;          /* Key events interrupt enable */
        uint8_t gpi_ien:1;         /* GPI interrupt enable */
        uint8_t k_lck_ien:1;       /* Keypad lock interrupt enable */
        uint8_t ovr_flow_ien:1;    /* Overflow interrupt enable */
        uint8_t int_cfg:1;         /* Interrupt configuration */
        uint8_t ovr_flow_m:1;      /* Overflow mode */
        uint8_t gpi_e_cfg:1;       /* GPI event mode configuration */
        uint8_t ai:1;              /* Auto-increment */
    } bits;
    uint8_t reg;
} tca8418_cfg_reg_t;

typedef union {
    struct {
        uint8_t k_int:1;           /* Key events interrupt status */
        uint8_t gpi_int:1;         /* GPI interrupt status */
        uint8_t k_lck_int:1;       /* Keypad lock interrupt status */
        uint8_t ovr_flow_int:1;    /* Overflow interrupt status */
        uint8_t cad_int:1;         /* CTRL-ALT-DEL status */
        uint8_t reserved:3;        /* Always 0 */
    } bits;
    uint8_t reg;
} tca8418_int_stat_reg_t;

typedef union {
    struct {
        uint8_t kec:4;             /* Key event count */
        uint8_t lck1:1;            /* Keypad lock status 1 */
        uint8_t lck2:1;            /* Keypad lock status 2 */
        uint8_t k_lck_en:1;        /* Key lock enable */
        uint8_t reserved:1;        /* Always 0 */
    } bits;
    uint8_t reg;
} tca8418_key_lck_ec_reg_t;

typedef struct {
    uint8_t key_number:7;          /* Key number (0-80 for matrix, 97-114 for GPI) */
    uint8_t evt_type:1;            /* 0 = release, 1 = press */
} tca8418_key_event_reg_t;

#endif // TCA8418_DEFS_H