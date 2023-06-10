/*******************************************************************************
 * @file    rf24.h
 * @author  lcc
 * @version 
 * @date    2022-12-28
 * @brief   CE  -- PB12
            CSN -- PB10
            IRQ -- PB11
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RF24_H
#define __RF24_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Memory Map */
#define RF24_REG_CONFIG      0x00
#define RF24_REG_EN_AA       0x01
#define RF24_REG_EN_RXADDR   0x02
#define RF24_REG_SETUP_AW    0x03
#define RF24_REG_SETUP_RETR  0x04
#define RF24_REG_RF_CH       0x05
#define RF24_REG_RF_SETUP    0x06
#define RF24_REG_STATUS      0x07
#define RF24_REG_OBSERVE_TX  0x08
#define RF24_REG_RPD         0x09
#define RF24_REG_RX_ADDR_P0  0x0A
#define RF24_REG_RX_ADDR_P1  0x0B
#define RF24_REG_RX_ADDR_P2  0x0C
#define RF24_REG_RX_ADDR_P3  0x0D
#define RF24_REG_RX_ADDR_P4  0x0E
#define RF24_REG_RX_ADDR_P5  0x0F
#define RF24_REG_TX_ADDR     0x10
#define RF24_REG_RX_PW_P0    0x11
#define RF24_REG_RX_PW_P1    0x12
#define RF24_REG_RX_PW_P2    0x13
#define RF24_REG_RX_PW_P3    0x14
#define RF24_REG_RX_PW_P4    0x15
#define RF24_REG_RX_PW_P5    0x16
#define RF24_REG_FIFO_STATUS 0x17
#define RF24_REG_DYNPD       0x1C
#define RF24_REG_FEATRUE     0x1D

/* Bit Mnemonics */
/* 00 - CONFIG */
#define RF24_SHIFT_MASK_RX_DR  6
#define RF24_SHIFT_MASK_TX_DS  5
#define RF24_SHIFT_MASK_MAX_RT 4
#define RF24_SHIFT_EN_CRC      3
#define RF24_SHIFT_CRCO        2
#define RF24_SHIFT_PWR_UP      1
#define RF24_SHIFT_PRIM_RX     0
/* 01 - EN_AA */
#define RF24_SHIFT_ENAA_P5     5
#define RF24_SHIFT_ENAA_P4     4
#define RF24_SHIFT_ENAA_P3     3
#define RF24_SHIFT_ENAA_P2     2
#define RF24_SHIFT_ENAA_P1     1
#define RF24_SHIFT_ENAA_P0     0
/* 02 - EN_RXADDR */
#define RF24_SHIFT_ERX_P5      5
#define RF24_SHIFT_ERX_P4      4
#define RF24_SHIFT_ERX_P3      3
#define RF24_SHIFT_ERX_P2      2
#define RF24_SHIFT_ERX_P1      1
#define RF24_SHIFT_ERX_P0      0
/* 03 - SETUP_AW */
#define RF24_SHIFT_AW          0
/* 04 - SETUP_RETR */
#define RF24_SHIFT_ARD         4
#define RF24_SHIFT_ARC         0
/* 05 - RF_CH */
#define RF24_SHIFT_PLL_LOCK    4
/* 06 - RF_SETUP */
#define RF24_SHIFT_CONT_WAVE   7
#define RF24_SHIFT_RF_DR_LOW   5
#define RF24_SHIFT_RF_DR_HIGH  3
#define RF24_SHIFT_RF_PWR      1
/* 07 - STATUS */
#define RF24_SHIFT_RX_DR       6
#define RF24_SHIFT_TX_DS       5
#define RF24_SHIFT_MAX_RT      4
#define RF24_SHIFT_RX_P_NO     1
#define RF24_SHIFT_TX_FULL     0
/* 08 - OBSERVE_TX */
#define RF24_SHIFT_PLOS_CNT    4
#define RF24_SHIFT_ARC_CNT     0
/* 17 - FIFO_STATUS */
#define RF24_SHIFT_TX_REUSE    6
#define RF24_SHIFT_FIFO_FULL   5
#define RF24_SHIFT_TX_EMPTY    4
#define RF24_SHIFT_RX_FULL     1
#define RF24_SHIFT_RX_EMPTY    0
/* 1C - DYNPD */
#define RF24_SHIFT_DPL_P5      5
#define RF24_SHIFT_DPL_P4      4
#define RF24_SHIFT_DPL_P3      3
#define RF24_SHIFT_DPL_P2      2
#define RF24_SHIFT_DPL_P1      1
#define RF24_SHIFT_DPL_P0      0
/* 1D - FEATURE */
#define RF24_SHIFT_EN_DPL      2
#define RF24_SHIFT_EN_ACK_PAY  1
#define RF24_SHIFT_EN_DYN_ACK  0

/* Instruction Mnemonics */
#define RF24_CMD_R_REGISTER    0x00
#define RF24_CMD_W_REGISTER    0x20
#define RF24_CMD_REGISTER_MASK 0x1F
#define RF24_CMD_R_RX_PAYLOAD  0x61
#define RF24_CMD_W_TX_PAYLOAD  0xA0
#define RF24_CMD_FLUSH_TX      0xE1
#define RF24_CMD_FLUSH_RX      0xE2
#define RF24_CMD_REUSE_TX_PL   0xE3
#define RF24_CMD_R_RX_PL_WID   0x60
#define RF24_CMD_W_ACK_PAYLOAD 0xA0
#define RF24_CMD_W_TX_PAYLOAD_NOACK  0xB0
#define RF24_CMD_NOP           0xFF

/* Status */
#define RF_STATUS_OK       0
#define RF_STATUS_FAIL     -1
#define RF_STATUS_MAX_RT   -2
#define RF_STATUS_TIMEOUT  -3

#define NRF24L01_CONFIG_DEFAULT     \
{                                   \
    .PayloadLen = 0,                \
    .AddrWidth = 5,                 \
    .Channel = 76,                  \
    .DataRate = RF24_1MBPS,         \
    .PowerLevel = RF24_PWR_LVL1,    \
    .PowerExtBit = SET,             \
    .RetryDelay = 1500,             \
    .RepeatCount = 15               \
}

/* Exported types ------------------------------------------------------------*/
/*
 * | level (enum value) | nRF24L01 | Si24R1 ext = 1 | Si24R1 ext = 0 |
 * |:------------------:|:-------:|:--------:|:-------:|
 * |   RF24_PWR_LVL0    | -18 dBm |  -6 dBm  | -12 dBm |
 * |   RF24_PWR_LVL1    | -12 dBm |  -0 dBm  | -4 dBm  |
 * |   RF24_PWR_LVL2    | -6 dBm  |  3 dBm   | 1 dBm   |
 * |   RF24_PWR_LVL3    |  0 dBm  |  7 dBm   | 4 dBm   |
 */
typedef enum
{
    RF24_PWR_LVL0 = 0,
    RF24_PWR_LVL1,
    RF24_PWR_LVL2,
    RF24_PWR_LVL3,
    RF24_PWR_MAX = RF24_PWR_LVL3
} RF24_PowerTypeDef;

typedef enum
{
    RF24_MODE_POWROFF,
    RF24_MODE_UNCONF,
    RF24_MODE_TX,
    RF24_MODE_RX
} RF24_ModeTypeDef;

typedef enum
{
    /** (0) represents 1 Mbps */
    RF24_1MBPS = 0,
    /** (1) represents 2 Mbps */
    RF24_2MBPS,
    /** (2) represents 250 kbps */
    RF24_250KBPS
} RF24_RateTypeDef;

typedef struct
{
    uint8_t PayloadLen;
    uint8_t AddrWidth;
    uint8_t Channel;
    RF24_RateTypeDef DataRate;
    RF24_PowerTypeDef PowerLevel;
    FlagStatus PowerExtBit; /* RF_PWR has 3 bits */
    uint16_t RetryDelay;
    uint8_t RepeatCount;
} RF24_ConfigTypeDef;

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void RF24_GpioInit(void);
FlagStatus RF24_CheckAvailable(void);
void RF24_Config(RF24_ConfigTypeDef *RF24_ConfigStruct);
void RF24_SetPayloadLen(uint8_t Pipe, uint8_t Len);
void RF24_SetAddrWidth(uint8_t Width);
void RF24_SetRetries(uint16_t Delay, uint8_t Count);
void RF24_SetChannel(uint8_t Channel);
void RF24_SetRfParams(RF24_RateTypeDef Rate, RF24_PowerTypeDef Level, FlagStatus Extension);
void RF24_PowerUp(void);
void RF24_PowerDown(void);
void RF24_RxMode(void);
void RF24_TxMode(void);
void RF24_SetTxAddr(uint8_t *Addr, uint8_t Len);
void RF24_SetRxAddr(uint8_t *Addr, uint8_t Len);
uint8_t RF24_GetStatus(void);
FlagStatus RF24_IsDataReady(void);
int RF24_SendData(uint8_t *Buf, uint8_t Len);
FlagStatus RF24_RxFifoEmpty(void);
uint8_t RF24_DataLen(void);
uint8_t RF24_GetData(uint8_t *data);
#ifdef DEBUG
void RF24_DumpReg(void);
#endif
void RF24_DefaultInit(void);

#endif /* __RF24_H */
