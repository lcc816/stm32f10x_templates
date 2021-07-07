/*******************************************************************************
* @file     --> sht3x.c
* @author   --> Lichangchun
* @version  --> 1.0
* @date     --> 30-Sept-2019
* @brief    --> SHT3x 温湿度传感器头文件
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "sht3x.h"
#include "delay.h"

/* Private define ------------------------------------------------------------*/
#ifdef  SHT3X_ALERT_PIN_ENABLE // 报警引脚是否可用
# define ALERT_READ   (SHT3X_ALERT_PORT->IDR  & SHT3X_ALERT_PIN) // 根据硬件调整
#endif

#ifdef  SHT3X_RESET_PIN_ENABLE // 硬件复位引脚是否可用
# define RESET_LOW()  (SHT3X_RESET_PORT->BRR  = SHT3X_RESET_PIN) // 设置 Reset 为低
# define RESET_HIGH() (SHT3X_RESET_PORT->BSRR = SHT3X_RESET_PIN) // 设置 Reset 为高
#endif

// Generator polynomial for CRC
#define POLYNOMIAL  0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001

/* Private variables ---------------------------------------------------------*/

/* Static functions ----------------------------------------------------------*/
// 主要对 soft I2C 的函数进行封装
// 开始写访问
static etError SHT3x_StartWriteAccess(void);
// 开始读访问
static etError SHT3x_StartReadAccess(void);
// 停止访问
static void SHT3x_StopAccess(void);
// 向从机写 1 条命令
static etError SHT3x_WriteCommand(etCommands cmd);
// 从总线上读取 2 字节并校验
static etError SHT3x_Read2BytesAndCrc(uint16_t *data, etI2cAck finaleAckNack,
                                      uint8_t timeout);
// 校验
static etError SHT3x_CheckCrc(void *data, uint8_t nbrOfBytes, uint8_t checksum);
// 计算校验和
static uint8_t SHT3x_CalcCrc(void *data, uint8_t nbrOfBytes);
// 计算温度
static float SHT3x_CalcTemperature(uint16_t rawValue);
// 计算湿度
static float SHT3x_CalcHumidity(uint16_t rawValue);

#ifdef  SHT3X_ALERT_PIN_ENABLE // 报警引脚是否可用
static etError SHT3x_Write2BytesAndCrc(uint16_t data);
static etError SHT3x_WriteAlertLimitData(float humidity, float temperature);
static etError SHT3x_ReadAlertLimitData(float *humidity, float *temperature);
static uint16_t SHT3x_CalcRawTemperature(float temperature);
static uint16_t SHT3x_CalcRawHumidity(float humidity);
#endif /* SHT3X_ALERT_PIN_ENABLE */

/*******************************************************************************
* @brief    --> 传感器一些初始化操作
* @param    --> None
* @retval   --> None
*******************************************************************************/
void SHT3x_Init(void)
{
#if defined(SHT3X_ALERT_PIN_ENABLE) || defined(SHT3X_RESET_PIN_ENABLE)

    GPIO_InitTypeDef  GPIO_InitStructure;

#ifdef  SHT3X_ALERT_PIN_ENABLE // 报警引脚是否可用
    RCC_APB2PeriphClockCmd(SHT3X_ALERT_CLK, ENABLE);  // 使能 GPIO 端口时钟

    GPIO_InitStructure.GPIO_Pin = SHT3X_ALERT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // 浮空输入
    GPIO_Init(SHT3X_ALERT_PORT, &GPIO_InitStructure);	// 根据设定参数初始化
    GPIO_SetBits(SHT3X_ALERT_PORT, SHT3X_ALERT_PIN);
#endif /* SHT3X_ALERT_PIN_ENABLE */

#ifdef  SHT3X_RESET_PIN_ENABLE // 硬件复位引脚是否可用
    RCC_APB2PeriphClockCmd(SHT3X_RESET_CLK, ENABLE);  // 使能 GPIO 端口时钟

    GPIO_InitStructure.GPIO_Pin = SHT3X_RESET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	// 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	// IO口速度为50MHz
    GPIO_Init(SHT3X_RESET_PORT, &GPIO_InitStructure);	// 根据设定参数初始化
    GPIO_SetBits(SHT3X_RESET_PORT, SHT3X_RESET_PIN);

    RESET_HIGH();
#endif /* SHT3X_RESET_PIN_ENABLE */

#endif
}

/*******************************************************************************
* @brief    --> 从 SHT3X 读取序列号
* @param    --> serialNumber   - 指向 32 位序列号的指针
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_ReadSerialNumber(uint32_t *serialNumber)
{
    etError error;
    uint16_t serialNumWords[2];

    error = SHT3x_StartWriteAccess();
    error |= SHT3x_WriteCommand(CMD_READ_SERIALNBR);

    if(error == NO_ERROR) error = SHT3x_StartReadAccess();
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(&serialNumWords[0], ACK, 100);
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(&serialNumWords[1], NACK, 0);

    SHT3x_StopAccess();

    if (error == NO_ERROR)
        *serialNumber = (serialNumWords[0] << 16) | serialNumWords[1];

    return error;
}

/*******************************************************************************
* @brief    --> 读取 SHT3X 的状态寄存器
* @param    --> status         - 指向 16 位状态的指针
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_ReadStatus(uint16_t *status)
{
    etError error;

    error = SHT3x_StartWriteAccess();
    error |= SHT3x_WriteCommand(CMD_READ_STATUS);

    if(error == NO_ERROR) error = SHT3x_StartReadAccess();
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(status, NACK, 0);

    SHT3x_StopAccess();

    return error;
}

/*******************************************************************************
* @brief    --> 清除 SHT3x 状态寄存器的所有警告标志
* @param    --> None
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_ClearAllAlertFlags(void)
{
    etError error;

    error = SHT3x_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_CLEAR_STATUS);

    SHT3x_StopAccess();

    return error;
}

/*******************************************************************************
* @brief    --> 读取温度值 [°C] 和相对湿度 [%RH]
* @param    --> temperature    - 指向温度值的指针
*               humidity       - 指向适度值的指针
*               repeatability  - 重复测量精度 [low, medium, high]
*               mode           - 指令模式 [时钟延长, 轮询]
*               timeout        - 超时时间
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               PARM_ERROR     = 未知参数
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_GetTempAndHumi(float *temperature, float *humidity,
                             etRepeatability repeatability,
                             etMode mode, uint8_t timeout)
{
    etError error;

    switch (mode)
    {
    case MODE_CLKSTRETCH:
        error = SHT3x_GetTempAndHumiClkStretch(temperature, humidity,
                                               repeatability, timeout);
        break;
    case MODE_POLLING:
        error = SHT3x_GetTempAndHumiPolling(temperature, humidity, repeatability,
                                            timeout);
        break;
    default:
        error = PARM_ERROR;
        break;
    }

    return error;
}

/*******************************************************************************
* @brief    --> 读取温度值 [°C] 和相对湿度 [%RH]
*               此函数使用 I2C 时钟延长来等待直到测量准备就绪
* @param    --> temperature    - 指向温度值的指针
*               humidity       - 指向适度值的指针
*               repeatability  - 重复测量精度 [low, medium, high]
*               timeout        - 超时时间 (ms)
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               PARM_ERROR     = 未知参数
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_GetTempAndHumiClkStretch(float *temperature, float *humidity,
                                       etRepeatability repeatability,
                                       uint8_t timeout)
{
    etError   error;
    uint16_t  rawValueTemp;
    uint16_t  rawValueHumi;

    error = SHT3x_StartWriteAccess();

    if(error == NO_ERROR)
    {
        // start measurement in clock stretching mode
        // use depending on the required repeatability, the corresponding command
        switch(repeatability)
        {
        case REPEATAB_LOW:
            error = SHT3x_WriteCommand(CMD_MEAS_CLOCKSTR_L);
            break;
        case REPEATAB_MEDIUM:
            error = SHT3x_WriteCommand(CMD_MEAS_CLOCKSTR_M);
            break;
        case REPEATAB_HIGH:
            error = SHT3x_WriteCommand(CMD_MEAS_CLOCKSTR_H);
            break;
        default:
            error = PARM_ERROR;
            break;
        }
    }

    if(error == NO_ERROR) error = SHT3x_StartReadAccess();
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(&rawValueTemp, ACK, timeout);
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(&rawValueHumi, NACK, 0);

    SHT3x_StopAccess();

    // 如果无错误, 以 °C 计算温度, 以 %RH 计算湿度
    if(error == NO_ERROR)
    {
        *temperature = SHT3x_CalcTemperature(rawValueTemp);
        *humidity = SHT3x_CalcHumidity(rawValueHumi);
    }

    return error;
}

/*******************************************************************************
* @brief    --> 读取温度值 [°C] 和相对湿度 [%RH]
*               此函数每 1ms 轮询一次, 直到测量准备就绪
* @param    --> temperature    - 指向温度值的指针
*               humidity       - 指向适度值的指针
*               repeatability  - 重复测量精度 [low, medium, high]
*               timeout        - 超时时间 (ms)
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               PARM_ERROR     = 未知参数
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_GetTempAndHumiPolling(float *temperature, float *humidity,
                                    etRepeatability repeatability,
                                    uint8_t timeout)
{
    etError   error;           // error code
    uint16_t  rawValueTemp;    // temperature raw value from sensor
    uint16_t  rawValueHumi;    // humidity raw value from sensor

    error  = SHT3x_StartWriteAccess();

    if(error == NO_ERROR)
    {
        // 以轮询模式开始测量
        switch(repeatability)
        {
        case REPEATAB_LOW:
            error = SHT3x_WriteCommand(CMD_MEAS_POLLING_L);
            break;
        case REPEATAB_MEDIUM:
            error = SHT3x_WriteCommand(CMD_MEAS_POLLING_M);
            break;
        case REPEATAB_HIGH:
            error = SHT3x_WriteCommand(CMD_MEAS_POLLING_H);
            break;
        default:
            error = PARM_ERROR;
            break;
        }
    }

    // 如果无错误, 等待测量就绪
    if(error == NO_ERROR)
    {
        while(timeout--)
        {
            // 检测测量是否完成
            error = SHT3x_StartReadAccess();

            if(error == NO_ERROR) break;

            delay_us(1000);// delay 1ms
        }

        if(timeout == 0) error = TIMEOUT_ERROR;
    }

    if(error == NO_ERROR)
    {
        error |= SHT3x_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
        error |= SHT3x_Read2BytesAndCrc(&rawValueHumi, NACK, 0);
    }

    SHT3x_StopAccess();

    // 如果无错误, 以 °C 计算温度, 以 %RH 计算湿度
    if(error == NO_ERROR)
    {
        *temperature = SHT3x_CalcTemperature(rawValueTemp);
        *humidity = SHT3x_CalcHumidity(rawValueHumi);
    }

    return error;
}

/*******************************************************************************
* @brief    --> 开始周期性测量
* @param    --> repeatability  - 重复测量精度 [low, medium, high]
*               frequency      - 测量频率 [0.5, 1, 2, 4, 10] Hz
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               PARM_ERROR     = 未知参数
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_StartPeriodicMeasurment(etRepeatability repeatability,
                                      etFrequency frequency)
{
    etError error;

    error = SHT3x_StartWriteAccess();

    if(error == NO_ERROR)
    {
        switch(repeatability)
        {
        case REPEATAB_LOW: // low repeatability
            switch(frequency)
            {
            case FREQUENCY_HZ5:  // low repeatability,  0.5 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_05_L);
                break;
            case FREQUENCY_1HZ:  // low repeatability,  1.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_1_L);
                break;
            case FREQUENCY_2HZ:  // low repeatability,  2.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_2_L);
                break;
            case FREQUENCY_4HZ:  // low repeatability,  4.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_4_L);
                break;
            case FREQUENCY_10HZ: // low repeatability, 10.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_10_L);
                break;
            default:
                error |= PARM_ERROR;
                break;
            }
            break;

        case REPEATAB_MEDIUM: // medium repeatability
            switch(frequency)
            {
            case FREQUENCY_HZ5:  // medium repeatability,  0.5 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_05_M);
                break;
            case FREQUENCY_1HZ:  // medium repeatability,  1.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_1_M);
                break;
            case FREQUENCY_2HZ:  // medium repeatability,  2.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_2_M);
                break;
            case FREQUENCY_4HZ:  // medium repeatability,  4.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_4_M);
                break;
            case FREQUENCY_10HZ: // medium repeatability, 10.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_10_M);
                break;
            default:
                error |= PARM_ERROR;
                break;
            }
            break;

        case REPEATAB_HIGH: // high repeatability
            switch(frequency)
            {
            case FREQUENCY_HZ5:  // high repeatability,  0.5 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_05_H);
                break;
            case FREQUENCY_1HZ:  // high repeatability,  1.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_1_H);
                break;
            case FREQUENCY_2HZ:  // high repeatability,  2.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_2_H);
                break;
            case FREQUENCY_4HZ:  // high repeatability,  4.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_4_H);
                break;
            case FREQUENCY_10HZ: // high repeatability, 10.0 Hz
                error |= SHT3x_WriteCommand(CMD_MEAS_PERI_10_H);
                break;
            default:
                error |= PARM_ERROR;
                break;
            }
            break;
        default:
            error |= PARM_ERROR;
            break;
        }
    }

    SHT3x_StopAccess();

    return error;
}

/*******************************************************************************
* @brief    --> 从传感器 buffer 读取最后一次测量值
* @param    --> temperature    - 指向温度值的指针
*               humidity       - 指向湿度值的指针
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_ReadMeasurementBuffer(float *temperature, float *humidity)
{
    etError   error;
    uint16_t  rawValueTemp;
    uint16_t  rawValueHumi;

    error = SHT3x_StartWriteAccess();

    if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_FETCH_DATA);
    if(error == NO_ERROR) error = SHT3x_StartReadAccess();
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(&rawValueTemp, ACK, 0);
    if(error == NO_ERROR) error = SHT3x_Read2BytesAndCrc(&rawValueHumi, NACK, 0);

    if(error == NO_ERROR)
    {
        *temperature = SHT3x_CalcTemperature(rawValueTemp);
        *humidity = SHT3x_CalcHumidity(rawValueHumi);
    }

    SHT3x_StopAccess();

    return error;
}

/*******************************************************************************
* @brief    --> 使能传感器上的加热器
* @param    --> None
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_EnableHeater(void)
{
    etError error;

    error = SHT3x_StartWriteAccess();

    if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_HEATER_ENABLE);

    SHT3x_StopAccess();

    return error;
}

/*******************************************************************************
* @brief    --> 关闭传感器上的加热器
* @param    --> None
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_DisableHeater(void)
{
    etError error;

    error = SHT3x_StartWriteAccess();

    if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_HEATER_DISABLE);

    SHT3x_StopAccess();

    return error;
}

#ifdef SHT3X_ALERT_PIN_ENABLE

/*******************************************************************************
* @brief    -->
* @param    -->
* @retval   -->
*******************************************************************************/
etError SHT3x_SetAlertLimits(float humidityHighSet,   float temperatureHighSet,
                             float humidityHighClear, float temperatureHighClear,
                             float humidityLowClear,  float temperatureLowClear,
                             float humidityLowSet,    float temperatureLowSet)
{
    etError  error;

    // write humidity & temperature alter limits, high set
    error = SHT3x_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_W_AL_LIM_HS);
    if(error == NO_ERROR) error = SHT3x_WriteAlertLimitData(humidityHighSet,
                                      temperatureHighSet);
    SHT3x_StopAccess();

    if(error == NO_ERROR)
    {
        // write humidity & temperature alter limits, high clear
        error = SHT3x_StartWriteAccess();
        if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_W_AL_LIM_HC);
        if(error == NO_ERROR) error = SHT3x_WriteAlertLimitData(humidityHighClear,
                                          temperatureHighClear);
        SHT3x_StopAccess();
    }

    if(error == NO_ERROR)
    {
        // write humidity & temperature alter limits, low clear
        error = SHT3x_StartWriteAccess();
        if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_W_AL_LIM_LC);
        if(error == NO_ERROR) error = SHT3x_WriteAlertLimitData(humidityLowClear,
                                          temperatureLowClear);
        SHT3x_StopAccess();
    }

    if(error == NO_ERROR)
    {
        // write humidity & temperature alter limits, low set
        error = SHT3x_StartWriteAccess();
        if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_W_AL_LIM_LS);
        if(error == NO_ERROR) error = SHT3x_WriteAlertLimitData(humidityLowSet,
                                          temperatureLowSet);
        SHT3x_StopAccess();
    }

    return error;
}

/*******************************************************************************
* @brief    -->
* @param    -->
* @retval   -->
*******************************************************************************/
etError SHT3x_GetAlertLimits(float *humidityHighSet,   float *temperatureHighSet,
                             float *humidityHighClear, float *temperatureHighClear,
                             float *humidityLowClear,  float *temperatureLowClear,
                             float *humidityLowSet,    float *temperatureLowSet)
{
    etError  error;  // error code

    // read humidity & temperature alter limits, high set
    error = SHT3x_StartWriteAccess();
    if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_R_AL_LIM_HS);
    if(error == NO_ERROR) error = SHT3x_StartReadAccess();
    if(error == NO_ERROR) error = SHT3x_ReadAlertLimitData(humidityHighSet,
                                      temperatureHighSet);
    SHT3x_StopAccess();

    if(error == NO_ERROR)
    {
        // read humidity & temperature alter limits, high clear
        error = SHT3x_StartWriteAccess();
        if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_R_AL_LIM_HC);
        if(error == NO_ERROR) error = SHT3x_StartReadAccess();
        if(error == NO_ERROR) error = SHT3x_ReadAlertLimitData(humidityHighClear,
                                          temperatureHighClear);
        SHT3x_StopAccess();
    }

    if(error == NO_ERROR)
    {
        // read humidity & temperature alter limits, low clear
        error = SHT3x_StartWriteAccess();
        if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_R_AL_LIM_LC);
        if(error == NO_ERROR) error = SHT3x_StartReadAccess();
        if(error == NO_ERROR) error = SHT3x_ReadAlertLimitData(humidityLowClear,
                                          temperatureLowClear);
        SHT3x_StopAccess();
    }

    if(error == NO_ERROR)
    {
        // read humidity & temperature alter limits, low set
        error = SHT3x_StartWriteAccess();
        if(error == NO_ERROR) error = SHT3x_WriteCommand(CMD_R_AL_LIM_LS);
        if(error == NO_ERROR) error = SHT3x_StartReadAccess();
        if(error == NO_ERROR) error = SHT3x_ReadAlertLimitData(humidityLowSet,
                                          temperatureLowSet);
        SHT3x_StopAccess();
    }

    return error;
}

/*******************************************************************************
* @brief    --> 读取报警引脚的状态
* @param    --> None
* @retval   --> RESET = 低电平
*               SET   = 高电平
*******************************************************************************/
FlagStatus SHT3x_ReadAlert(void)
{
    return (ALERT_READ != 0) ? SET : RESET;
}

#endif /* SHT3X_ALERT_PIN_ENABLE */

/*******************************************************************************
* @brief    --> 调用软复位机制强制传感器进入确定状态, 不用移除电源
* @param    --> None
* @retval   --> ACK_ERROR      = 传感器无应答
*               CHECKSUM_ERROR = 校验和不匹配
*               TIMEOUT_ERROR  = 超时
*               NO_ERROR       = 无错误
*******************************************************************************/
etError SHT3x_SoftReset(void)
{
    etError error;

    error = SHT3x_StartWriteAccess();

    error |= SHT3x_WriteCommand(CMD_SOFT_RESET);

    SHT3x_StopAccess();

    if(error == NO_ERROR) delay_ms(50);

    return error;
}

#ifdef  SHT3X_RESET_PIN_ENABLE

/*******************************************************************************
* @brief    --> 硬件复位
* @param    --> None
* @retval   --> None
*******************************************************************************/
void SHT3x_HardReset(void)
{
    RESET_LOW();

    delay_ms(100);

    RESET_HIGH();

    delay_ms(50);
}

#endif /* SHT3X_RESET_PIN_ENABLE */

/*******************************************************************************
*                              内部函数定义                                    *
*******************************************************************************/
//-----------------------------------------------------------------------------
static etError SHT3x_StartWriteAccess(void)
{
    etError error;

    I2c_StartCondition();
    error = I2c_WriteByte(SHT3X_ADDRESS << 1);

    return error;
}

//-----------------------------------------------------------------------------
static etError SHT3x_StartReadAccess(void)
{
    etError error;

    I2c_StartCondition();
    error = I2c_WriteByte(SHT3X_ADDRESS << 1 | 0x01);

    return error;
}

//-----------------------------------------------------------------------------
static void SHT3x_StopAccess(void)
{
    I2c_StopCondition();
}

//-----------------------------------------------------------------------------
static etError SHT3x_WriteCommand(etCommands command)
{
    etError error;

    error  = I2c_WriteByte(command >> 8);
    error |= I2c_WriteByte(command & 0xFF);

    return error;
}

#ifdef  SHT3X_ALERT_PIN_ENABLE

//-----------------------------------------------------------------------------
static etError SHT3x_Write2BytesAndCrc(uint16_t data)
{
    etError error;
    uint8_t     bytes[2];
    uint8_t     checksum;

    bytes[0] = data >> 8;
    bytes[1] = data & 0xFF;
    checksum = SHT3x_CalcCrc(bytes, 2);

    // write two data bytes and one checksum byte
    error = I2c_WriteByte(bytes[0]);
    if(error == NO_ERROR) error = I2c_WriteByte(bytes[1]);
    if(error == NO_ERROR) error = I2c_WriteByte(checksum);

    return error;
}

#endif /* SHT3X_ALERT_PIN_ENABLE */

//-----------------------------------------------------------------------------
static etError SHT3x_Read2BytesAndCrc(uint16_t *data, etI2cAck finaleAckNack,
                                      uint8_t timeout)
{
    etError error;
    uint8_t bytes[2];
    uint8_t checksum;

    // read two data bytes and one checksum byte
    error = I2c_ReadByte(&bytes[0], ACK, timeout);
    if(error == NO_ERROR) error = I2c_ReadByte(&bytes[1], ACK, 0);
    if(error == NO_ERROR) error = I2c_ReadByte(&checksum, finaleAckNack, 0);

    if(error == NO_ERROR) error = SHT3x_CheckCrc(bytes, 2, checksum);

    *data = (bytes[0] << 8) | bytes[1];

    return error;
}

//-----------------------------------------------------------------------------
static etError SHT3x_CheckCrc(void *data, uint8_t nbrOfBytes, uint8_t checksum)
{
    uint8_t crc;

    crc = SHT3x_CalcCrc(data, nbrOfBytes);

    if (crc != checksum)  return CHECKSUM_ERROR;
    else                  return NO_ERROR;
}

//-----------------------------------------------------------------------------
static uint8_t SHT3x_CalcCrc(void *data, uint8_t nbrOfBytes)
{
    uint8_t bit;        // bit mask
    uint8_t crc = 0xFF; // calculated checksum
    uint8_t byteCtr;    // byte counter
    uint8_t *pt = (uint8_t *)data;

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
    {
        crc ^= pt[byteCtr];
        for(bit = 8; bit > 0; --bit)
        {
            if(crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
            else           crc = (crc << 1);
        }
    }

    return crc;
}

//-----------------------------------------------------------------------------
static float SHT3x_CalcTemperature(uint16_t rawValue)
{
    // calculate temperature [у]
    // T = -45 + 175 * rawValue / (2^16-1)
    return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

//-----------------------------------------------------------------------------
static float SHT3x_CalcHumidity(uint16_t rawValue)
{
    // calculate relative humidity [%RH]
    // RH = rawValue / (2^16-1) * 100
    return 100.0f * (float)rawValue / 65535.0f;
}

#ifdef  SHT3X_ALERT_PIN_ENABLE

//-----------------------------------------------------------------------------
static uint16_t SHT3x_CalcRawTemperature(float temperature)
{
    // calculate raw temperature [ticks]
    // rawT = (temperature + 45) / 175 * (2^16-1)
    return (temperature + 45.0f) / 175.0f * 65535.0f;
}

//-----------------------------------------------------------------------------
static uint16_t SHT3x_CalcRawHumidity(float humidity)
{
    // calculate raw relative humidity [ticks]
    // rawRH = humidity / 100 * (2^16-1)
    return humidity / 100.0f * 65535.0f;
}

//-----------------------------------------------------------------------------
static etError SHT3x_WriteAlertLimitData(float humidity, float temperature)
{
    etError  error;

    int16_t rawHumidity;
    int16_t rawTemperature;

    if((humidity < 0.0f) || (humidity > 100.0f)
            || (temperature < -45.0f) || (temperature > 130.0f))
    {
        error = PARM_ERROR;
    }
    else
    {
        rawHumidity    = SHT3x_CalcRawHumidity(humidity);
        rawTemperature = SHT3x_CalcRawTemperature(temperature);

        error = SHT3x_Write2BytesAndCrc((rawHumidity & 0xFE00) | ((rawTemperature >> 7) & 0x001FF));
    }

    return error;
}

//-----------------------------------------------------------------------------
static etError SHT3x_ReadAlertLimitData(float *humidity, float *temperature)
{
    etError  error;
    uint16_t     data;

    error = SHT3x_Read2BytesAndCrc(&data, NACK, 0);

    if(error == NO_ERROR)
    {
        *humidity = SHT3x_CalcHumidity(data & 0xFE00);
        *temperature = SHT3x_CalcTemperature(data << 7);
    }

    return error;
}

#endif /* SHT3X_ALERT_PIN_ENABLE */
