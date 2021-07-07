/*******************************************************************************
* @file     --> sht3x.h
* @author   --> Lichangchun
* @version  --> 1.0
* @date     --> 30-Sept-2019
* @brief    --> SHT3x 温湿度传感器头文件
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHT3X_H
#define	__SHT3X_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "i2c_soft.h"

/* Exported define -----------------------------------------------------------*/
/* SHT3X 的 I2C 地址 */
#define SHT3X_ADDRESS   0x44

/* 根据硬件调整 Alert 和 Reset 引脚的宏开关和宏定义 */

//#define SHT3X_ALERT_PIN_ENABLE
//#define SHT3X_RESET_PIN_ENABLE

#ifdef  SHT3X_ALERT_PIN_ENABLE // 报警引脚是否可用
# define  SHT3X_ALERT_PIN           GPIO_Pin_10
# define  SHT3X_ALERT_PORT          GPIOB
# define  SHT3X_ALERT_CLK           RCC_APB2Periph_GPIOB
#endif

#ifdef  SHT3X_RESET_PIN_ENABLE // 硬件复位引脚是否可用
# define  SHT3X_RESET_PIN           GPIO_Pin_9
# define  SHT3X_RESET_PORT          GPIOB
# define  SHT3X_RESET_CLK           RCC_APB2Periph_GPIOB
#endif

/* Exported type -------------------------------------------------------------*/
/* 传感器支持的指令 */
typedef enum {
    CMD_READ_SERIALNBR  = 0x3780, // read serial number
    CMD_READ_STATUS     = 0xF32D, // read status register
    CMD_CLEAR_STATUS    = 0x3041, // clear status register
    CMD_HEATER_ENABLE   = 0x306D, // enabled heater
    CMD_HEATER_DISABLE  = 0x3066, // disable heater
    CMD_SOFT_RESET      = 0x30A2, // sofloat *reset
    CMD_MEAS_CLOCKSTR_H = 0x2C06, // measurement: clock stretching, high repeatability
    CMD_MEAS_CLOCKSTR_M = 0x2C0D, // measurement: clock stretching, medium repeatability
    CMD_MEAS_CLOCKSTR_L = 0x2C10, // measurement: clock stretching, low repeatability
    CMD_MEAS_POLLING_H  = 0x2400, // measurement: polling, high repeatability
    CMD_MEAS_POLLING_M  = 0x240B, // measurement: polling, medium repeatability
    CMD_MEAS_POLLING_L  = 0x2416, // measurement: polling, low repeatability
    CMD_MEAS_PERI_05_H  = 0x2032, // measurement: periodic 0.5 mps, high repeatability
    CMD_MEAS_PERI_05_M  = 0x2024, // measurement: periodic 0.5 mps, medium repeatability
    CMD_MEAS_PERI_05_L  = 0x202F, // measurement: periodic 0.5 mps, low repeatability
    CMD_MEAS_PERI_1_H   = 0x2130, // measurement: periodic 1 mps, high repeatability
    CMD_MEAS_PERI_1_M   = 0x2126, // measurement: periodic 1 mps, medium repeatability
    CMD_MEAS_PERI_1_L   = 0x212D, // measurement: periodic 1 mps, low repeatability
    CMD_MEAS_PERI_2_H   = 0x2236, // measurement: periodic 2 mps, high repeatability
    CMD_MEAS_PERI_2_M   = 0x2220, // measurement: periodic 2 mps, medium repeatability
    CMD_MEAS_PERI_2_L   = 0x222B, // measurement: periodic 2 mps, low repeatability
    CMD_MEAS_PERI_4_H   = 0x2334, // measurement: periodic 4 mps, high repeatability
    CMD_MEAS_PERI_4_M   = 0x2322, // measurement: periodic 4 mps, medium repeatability
    CMD_MEAS_PERI_4_L   = 0x2329, // measurement: periodic 4 mps, low repeatability
    CMD_MEAS_PERI_10_H  = 0x2737, // measurement: periodic 10 mps, high repeatability
    CMD_MEAS_PERI_10_M  = 0x2721, // measurement: periodic 10 mps, medium repeatability
    CMD_MEAS_PERI_10_L  = 0x272A, // measurement: periodic 10 mps, low repeatability
    CMD_FETCH_DATA      = 0xE000, // readout measurements for periodic mode
    CMD_R_AL_LIM_LS     = 0xE102, // read alert limits, low set
    CMD_R_AL_LIM_LC     = 0xE109, // read alert limits, low clear
    CMD_R_AL_LIM_HS     = 0xE11F, // read alert limits, high set
    CMD_R_AL_LIM_HC     = 0xE114, // read alert limits, high clear
    CMD_W_AL_LIM_HS     = 0x611D, // write alert limits, high set
    CMD_W_AL_LIM_HC     = 0x6116, // write alert limits, high clear
    CMD_W_AL_LIM_LC     = 0x610B, // write alert limits, low clear
    CMD_W_AL_LIM_LS     = 0x6100, // write alert limits, low set
    CMD_NO_SLEEP        = 0x303E,
} etCommands;

/* 重复测量精度 */
typedef enum {
    REPEATAB_HIGH,   // high repeatability
    REPEATAB_MEDIUM, // medium repeatability
    REPEATAB_LOW,    // low repeatability
} etRepeatability;

/* 测量模式 */
typedef enum {
    MODE_CLKSTRETCH, // clock stretching
    MODE_POLLING,    // polling
} etMode;

/* 测量频率 */
typedef enum {
    FREQUENCY_HZ5,  //  0.5 measurements per seconds
    FREQUENCY_1HZ,  //  1.0 measurements per seconds
    FREQUENCY_2HZ,  //  2.0 measurements per seconds
    FREQUENCY_4HZ,  //  4.0 measurements per seconds
    FREQUENCY_10HZ, // 10.0 measurements per seconds
} etFrequency;


/* Exported variables---------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void SHT3x_Init(void);

etError SHT3x_ReadSerialNumber(uint32_t *serialNumber);

etError SHT3x_ReadStatus(uint16_t *status);

etError SHT3x_ClearAllAlertFlags(void);

etError SHT3x_GetTempAndHumi(float *temperature, float *humidity,
                             etRepeatability repeatability,
                             etMode mode, uint8_t timeout);

etError SHT3x_GetTempAndHumiClkStretch(float *temperature, float *humidity,
                                       etRepeatability repeatability,
                                       uint8_t timeout);

etError SHT3x_GetTempAndHumiPolling(float *temperature, float *humidity,
                                    etRepeatability repeatability,
                                    uint8_t timeout);

etError SHT3x_StartPeriodicMeasurment(etRepeatability repeatability,
                                      etFrequency frequency);

etError SHT3x_ReadMeasurementBuffer(float *temperature, float *humidity);

etError SHT3x_EnableHeater(void);

etError SHT3x_DisableHeater(void);

etError SHT3x_SoftReset(void);

#ifdef  SHT3X_RESET_PIN_ENABLE

void SHT3x_HardReset(void);

#endif

#ifdef SHT3X_ALERT_PIN_ENABLE

etError SHT3x_SetAlertLimits(float humidityHighSet,   float temperatureHighSet,
                             float humidityHighClear, float temperatureHighClear,
                             float humidityLowClear,  float temperatureLowClear,
                             float humidityLowSet,    float temperatureLowSet);

etError SHT3x_GetAlertLimits(float *humidityHighSet,   float *temperatureHighSet,
                             float *humidityHighClear, float *temperatureHighClear,
                             float *humidityLowClear,  float *temperatureLowClear,
                             float *humidityLowSet,    float *temperatureLowSet);

FlagStatus SHT3x_ReadAlert(void);

#endif /* SHT3X_ALERT_PIN_ENABLE */

#endif /* __SHT3X_H */
