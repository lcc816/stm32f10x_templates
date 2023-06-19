/*******************************************************************************
 * @file     rf24.c
 * @author   lcc
 * @version  
 * @date     28-Dec-2022
 * @brief    
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "rf24.h"
#include "delay.h"

/* Private define ------------------------------------------------------------*/
#define RF24_MAX(a, b)  ((a)>(b)?(a):(b))
#define RF24_MIN(a, b)  ((a)<(b)?(a):(b))
#define BIT(n)          (1<<(n))
#define BITS(m, n)      (~(BIT(m)-1) & ((BIT(n) - 1) | BIT(n)))

/******************************************************************************/
/*                     definitions about SPI and GPIO                         */
/******************************************************************************/
#ifdef MINI_STM32 // Development Board
/* Pin definition */
#define CE_PIN          GPIO_Pin_4
#define CE_GPIO_PORT    GPIOA
#define CE_GPIO_CLK     RCC_APB2Periph_GPIOA
#define CSN_PIN         GPIO_Pin_4
#define CSN_GPIO_PORT   GPIOC
#define CSN_GPIO_CLK    RCC_APB2Periph_GPIOC
#define IRQ_PIN         GPIO_Pin_5
#define IRQ_GPIO_PORT   GPIOC
#define IRQ_GPIO_CLK    RCC_APB2Periph_GPIOC
#define IRQ_EXTI_LINE   EXTI_Line5
#define IRQ_EXTI_NUM    EXTI9_5_IRQn
#define IRQ_EXTI_GPIO   GPIO_PortSourceGPIOC
#define IRQ_EXTI_PIN    GPIO_PinSource5
/* slect SPI */
#include "spi1.h"
#define _spi_transfer   SPI1_ReadWriteByte
#define _spi_init       SPI1_Init

#else // NewBee
#define CE_PIN          GPIO_Pin_12
#define CE_GPIO_PORT    GPIOB 
#define CE_GPIO_CLK     RCC_APB2Periph_GPIOB
#define CSN_PIN         GPIO_Pin_10
#define CSN_GPIO_PORT   GPIOB
#define CSN_GPIO_CLK    RCC_APB2Periph_GPIOB
#define IRQ_PIN         GPIO_Pin_11
#define IRQ_GPIO_PORT   GPIOB
#define IRQ_GPIO_CLK    RCC_APB2Periph_GPIOB
#define IRQ_EXTI_LINE   EXTI_Line11
#define IRQ_EXTI_NUM    EXTI15_10_IRQn
#define IRQ_EXTI_GPIO   GPIO_PortSourceGPIOB
#define IRQ_EXTI_PIN    GPIO_PinSource11
/* slect SPI */
#include "spi2.h"
#define _spi_transfer   SPI2_ReadWriteByte
#define _spi_init       SPI2_Init
#endif
/* Pin operation */
#define RF24_CE_HIGH()  GPIO_SetBits(CE_GPIO_PORT, CE_PIN)
#define RF24_CE_LOW()   GPIO_ResetBits(CE_GPIO_PORT, CE_PIN)
#define RF24_CSN_HIGH() GPIO_SetBits(CSN_GPIO_PORT, CSN_PIN)
#define RF24_CSN_LOW()  GPIO_ResetBits(CSN_GPIO_PORT, CSN_PIN)

/* Private variables ---------------------------------------------------------*/
uint8_t PayloadLen;
RF24_ModeTypeDef RF24_Mode = RF24_MODE_POWROFF;

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * @brief   Write one byte to a register
 * @param   addr - register address
 * @param   value - the value to write
 * @retval  The value of STATUS register
 ******************************************************************************/
static uint8_t _spi_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t status;

    RF24_CSN_LOW();
    status = _spi_transfer(addr | RF24_CMD_W_REGISTER);
    _spi_transfer(value);
    RF24_CSN_HIGH();

    return status;
}

/*******************************************************************************
 * @brief   Read one byte from a register
 * @param   addr - register address
 * @retval  Value of the register
 ******************************************************************************/
static uint8_t _spi_read_reg(uint8_t addr)
{
    uint8_t value;

    RF24_CSN_LOW();
    _spi_transfer(addr | RF24_CMD_R_REGISTER);
    value = _spi_transfer(0xFF);
    RF24_CSN_HIGH();

    return value;
}

/*******************************************************************************
 * @brief   Write bytes to a register
 * @param   addr - register address
 * @param   buffer - pointer to the buffer to write
 * @param   bytes - number of bytes
 * @retval  The value of STATUS register
 ******************************************************************************/
static uint8_t _spi_write_bytes(uint8_t addr, const uint8_t *buffer, uint8_t bytes)
{
    uint8_t status;

    RF24_CSN_LOW();
    status = _spi_transfer(addr | RF24_CMD_W_REGISTER);
    for (int i = 0; i < bytes; i++)
    {
        _spi_transfer(buffer[i]);
    }
    RF24_CSN_HIGH();

    return status;
}

/*******************************************************************************
 * @brief   Read bytes from a register
 * @param   addr - register address
 * @param   buffer - pointer to the read buffer
 * @param   bytes - number of bytes to read
 * @retval  The value of STATUS register
 ******************************************************************************/
static uint8_t _spi_read_bytes(uint8_t addr, uint8_t *buffer, uint8_t bytes)
{
    uint8_t status;

    RF24_CSN_LOW();
    status = _spi_transfer(addr | RF24_CMD_R_REGISTER);
    for (int i = 0; i < bytes; i++)
    {
        buffer[i] = _spi_transfer(0xFF);
    }
    RF24_CSN_HIGH();

    return status;
}

static uint8_t _flush_tx(void)
{
    uint8_t st = 0;
    RF24_CSN_LOW();
    st = _spi_transfer(RF24_CMD_FLUSH_TX);
    RF24_CSN_HIGH();
    return st;
}

static uint8_t _flush_rx(void)
{
    uint8_t st = 0;
    RF24_CSN_LOW();
    st = _spi_transfer(RF24_CMD_FLUSH_RX);
    RF24_CSN_HIGH();
    return st;
}

static void _clear_status_flags(void)
{
    /* Clear the flags in STATUS register */
    _spi_write_reg(RF24_REG_STATUS, BIT(RF24_SHIFT_RX_DR) |
                   BIT(RF24_SHIFT_TX_DS) | BIT(RF24_SHIFT_MAX_RT));
}

/*******************************************************************************
 * @brief   nRF24L01 GPIO and SPI initialize
 * @param   None
 * @retval  None
 ******************************************************************************/
void RF24_GpioInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(CE_GPIO_CLK|CSN_GPIO_CLK|IRQ_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = CE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CE_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CSN_PIN;
    GPIO_Init(CSN_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = IRQ_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IRQ_GPIO_PORT, &GPIO_InitStructure);

    GPIO_EXTILineConfig(IRQ_EXTI_GPIO, IRQ_EXTI_PIN); // 绑定中断源, 重要
    EXTI_InitStructure.EXTI_Line = IRQ_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = IRQ_EXTI_NUM; //使能按键外部中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //抢占优先级 2，
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; //子优先级 2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);

    _spi_init();        // 初始化SPI     

    RF24_CE_LOW();      // 使能SI24R1

    RF24_CSN_HIGH();    // SPI片选取消  
}

FlagStatus RF24_CheckAvailable(void)
{
    uint8_t i;
    uint8_t tx_addr[5] = {'A', 'B', 'C', 'D', 'E'};
    uint8_t read_buf[5] = {0};
    uint8_t backup_addr[5] = {0};
    _spi_read_bytes(RF24_REG_TX_ADDR, backup_addr, 5);

    _spi_write_bytes(RF24_REG_TX_ADDR, tx_addr, 5);
    _spi_read_bytes(RF24_REG_TX_ADDR, read_buf, 5);
    for (i = 0; i < 5; i++)
    {
        if (tx_addr[i] != read_buf[i])
        {
            return RESET;
        }
    }

    _spi_write_bytes(RF24_REG_TX_ADDR, backup_addr, 5);
    return SET;
}

/*******************************************************************************
 * @brief   Sets the payload length of pipe x
 * @param   Pipe - index of pipe
 * @param   Len - length value
 * @retval  None
 ******************************************************************************/
void RF24_SetPayloadLen(uint8_t Pipe, uint8_t Len)
{
    Len = RF24_MAX(1, RF24_MIN(32, Len));
    Pipe = RF24_MIN(Pipe, 5);

    _spi_write_reg(RF24_REG_RX_PW_P0 + Pipe, Len);
}

/*******************************************************************************
 * @brief   Sets the ADDR width
 * @param   Width - Address Widths
 * @retval  None
 ******************************************************************************/
void RF24_SetAddrWidth(uint8_t Width)
{
    /*
     * '00' - Illegal
     * '01' - 3 bytes
     * '10' - 4 bytes
     * '11' – 5 bytes
     **/
    Width = RF24_MAX(3, RF24_MIN(5, Width));

    _spi_write_reg(RF24_REG_SETUP_AW, Width - 2);
}

/*******************************************************************************
 * @brief   Setup of Automatic Retransmission
 * @param   Delay - Auto Retransmit Delay
 * @param   Count - Auto Retransmit Count
 * @retval  None
 ******************************************************************************/
void RF24_SetRetries(uint16_t Delay, uint8_t Count)
{
    /*
     * '0000' - 250uS
     * '0001' - 500uS
     * '0010' - 750uS
     * ...
     * '1111' – 40000uS
     **/
    uint8_t ard = RF24_MIN(15, (Delay / 250));
    Count = RF24_MIN(15, Count);
    _spi_write_reg(RF24_REG_SETUP_RETR, (ard << RF24_SHIFT_ARD) | Count);
}

/*******************************************************************************
 * @brief   Sets the RF channel
 * @param   Channel - RF channel
 * @retval  None
 ******************************************************************************/
void RF24_SetChannel(uint8_t Channel)
{
    Channel = RF24_MIN(125, Channel);
    _spi_write_reg(RF24_REG_RF_CH, Channel);
}

static uint8_t _parse_power_reg(RF24_PowerTypeDef Level, FlagStatus Extension)
{
    /*
     * | level (enum value) | nRF24L01 | Si24R1 with extension bit = 1 | Si24R1 with extension bit = 0 |
     * |:------------------:|:-------:|:--------:|:-------:|
     * |   RF24_PWR_LVL0    | -18 dBm |  -6 dBm  | -12 dBm |
     * |   RF24_PWR_LVL1    | -12 dBm |  -0 dBm  | -4 dBm  |
     * |   RF24_PWR_LVL2    | -6 dBm  |  3 dBm   | 1 dBm   |
     * |   RF24_PWR_LVL3    |  0 dBm  |  7 dBm   | 4 dBm   |
     */
    if (Level > RF24_PWR_MAX)
        Level = RF24_PWR_MAX;
    return Extension ? (Level << 1) + 1 : (Level << 1);
}

static uint8_t _parse_data_rate_reg(RF24_RateTypeDef Rate)
{
    /*
     * Encoding:
     * [RF_DR_LOW, RF_DR_HIGH]:
     * ‘00’ – 1Mbps
     * ‘01’ – 2Mbps
     * ‘10’ – 250kbps
     * ‘11’ – Reserved
     */
    if (Rate == RF24_250KBPS)
        return BIT(RF24_SHIFT_RF_DR_LOW);
    else if (Rate == RF24_2MBPS)
        return BIT(RF24_SHIFT_RF_DR_HIGH);
    else
        return 0;
}

/*******************************************************************************
 * @brief   Sets RF parameters
 * @param   Rate - data rate
 * @param   Level - power level
 * @param   Extension - SET means Si24R1 with extension bit
 * @retval  None
 ******************************************************************************/
void RF24_SetRfParams(RF24_RateTypeDef Rate, RF24_PowerTypeDef Level, FlagStatus Extension)
{
    uint8_t setup = _parse_data_rate_reg(Rate);
    setup |= _parse_power_reg(Level, Extension);
    _spi_write_reg(RF24_REG_RF_SETUP, setup);
}

/*******************************************************************************
 * @brief   Power up
 * @param   None
 * @retval  None
 ******************************************************************************/
void RF24_PowerUp(void)
{
    uint8_t value = _spi_read_reg(RF24_REG_CONFIG);
    if(!(value & BIT(RF24_SHIFT_PWR_UP)))
    {
        value |= BIT(RF24_SHIFT_PWR_UP);
        _spi_write_reg(RF24_REG_CONFIG, value);
        //rt_thread_mdelay(3);
    }
}

/*******************************************************************************
 * @brief   Power down
 * @param   None
 * @retval  None
 ******************************************************************************/
void RF24_PowerDown(void)
{
    RF24_CE_LOW();
    _spi_write_reg(RF24_REG_CONFIG, _spi_read_reg(RF24_REG_CONFIG) & ~BIT(RF24_SHIFT_PWR_UP));
}

/*******************************************************************************
 * @brief   Hardware configuration initialization
 * @param   RF24_ConfigStruct - pointer to a RF24_ConfigTypeDef structure which 
 *          will be initialized.
 * @retval  None
 ******************************************************************************/
void RF24_Config(RF24_ConfigTypeDef *Cfg)
{
    uint8_t DYNPD = 0, FEATURE = 0;

    PayloadLen = Cfg->PayloadLen;
    if (PayloadLen)
    {
        /* Sets the payload length of pipe 0 & 1 */
        RF24_SetPayloadLen(0, Cfg->PayloadLen);
        RF24_SetPayloadLen(1, Cfg->PayloadLen);
    }
    else
    {
        /* Enable dynamic payload length data pipe 0 & 1 */
        /* A PTX that transmits to a PRX with DPL enabled must have the DPL_P0
           bit in DYNPD set. */
        DYNPD |= BIT(RF24_SHIFT_DPL_P0)|BIT(RF24_SHIFT_DPL_P1);
    }
    _spi_write_reg(RF24_REG_DYNPD, DYNPD);
    if (DYNPD)
    {
        /* Enables Dynamic Payload Length */
        FEATURE |= BIT(RF24_SHIFT_EN_DPL);
    }
    _spi_write_reg(RF24_REG_FEATRUE, FEATURE);
    /* Enable auto-ack on all pipes */
    _spi_write_reg(RF24_REG_EN_AA, 0x3F);
    /* Enable Rx data pipe 0 & 1 */
    _spi_write_reg(RF24_REG_EN_RXADDR, BIT(RF24_SHIFT_ERX_P0) | BIT(RF24_SHIFT_ERX_P1));
    RF24_SetAddrWidth(Cfg->AddrWidth);
    RF24_SetRetries(Cfg->RetryDelay, Cfg->RepeatCount);
    RF24_SetChannel(Cfg->Channel);
    RF24_SetRfParams(Cfg->DataRate, Cfg->PowerLevel, Cfg->PowerExtBit);
    /* Clear the flags in STATUS register */
    _spi_write_reg(RF24_REG_STATUS, BIT(RF24_SHIFT_RX_DR) |
                   BIT(RF24_SHIFT_TX_DS) | BIT(RF24_SHIFT_MAX_RT));
    _flush_rx();
    _flush_tx();

    _spi_write_reg(RF24_REG_CONFIG, BIT(RF24_SHIFT_EN_CRC) | BIT(RF24_SHIFT_CRCO) |
                   BIT(RF24_SHIFT_MASK_TX_DS) | BIT(RF24_SHIFT_MASK_MAX_RT));
    RF24_PowerUp();
    RF24_Mode = RF24_MODE_RX;
}

/*******************************************************************************
 * @brief   Set nRF24L01 to RX mode
 * @param   None
 * @retval  None
 ******************************************************************************/
void RF24_RxMode(void)
{
    uint8_t config_reg = _spi_read_reg(RF24_REG_CONFIG);

    config_reg |= BIT(RF24_SHIFT_PRIM_RX);
    _spi_write_reg(RF24_REG_CONFIG, config_reg);
    _clear_status_flags();
    RF24_CE_HIGH();
    RF24_Mode = RF24_MODE_RX;
}

/*******************************************************************************
 * @brief   Set nRF24L01 to TX mode
 * @param   None
 * @retval  None
 ******************************************************************************/
void RF24_TxMode(void)
{
    uint8_t config_reg = _spi_read_reg(RF24_REG_CONFIG);

    RF24_CE_LOW();
    //rt_hw_us_delay(100);
    config_reg &= ~BIT(RF24_SHIFT_PRIM_RX);
    _spi_write_reg(RF24_REG_CONFIG, config_reg);
    // enable rx pip0 for auto ack
    _spi_write_reg(RF24_REG_EN_RXADDR, _spi_read_reg(RF24_REG_EN_RXADDR) | BIT(RF24_SHIFT_ERX_P0));
    RF24_Mode = RF24_MODE_TX;
}

/*******************************************************************************
 * @brief   Set the transmit address
 * @param   Addr - The destination address where the data is sent
 * @param   Len - Address length to set
 * @retval  None
 ******************************************************************************/
void RF24_SetTxAddr(uint8_t *Addr, uint8_t Len)
{
    Len = (Len > 5) ? 5 : Len;
    _spi_write_bytes(RF24_REG_TX_ADDR, Addr, Len);
    /*
     * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
     */
    _spi_write_bytes(RF24_REG_RX_ADDR_P0, Addr, Len);
}

/*******************************************************************************
 * @brief   Set the receive address data pipe. Only pipe1 is used.
 * @param   Addr - The PIPE address used to receive data
 * @param   Len - Address length to set
 * @retval  None
 ******************************************************************************/
void RF24_SetRxAddr(uint8_t *Addr, uint8_t Len)
{
    _spi_write_bytes(RF24_REG_RX_ADDR_P1, Addr, Len);
}

/*******************************************************************************
 * @brief   Send a data packet
 * @param   Buf - pointer to the data buffer to send
 * @param   Len - buffer length
 * @retval  Result status of sending
 ******************************************************************************/
int RF24_SendData(uint8_t *Buf, uint8_t Len)
{
    uint8_t status;
    uint8_t timeout;
    uint8_t *ptr = Buf;

    if (RF24_Mode != RF24_MODE_TX)
        return RF_STATUS_FAIL;

    if (Len > 32)
        Len = 32;

    _flush_tx();
    _clear_status_flags();

    /* go to standby-I mode */
    RF24_CE_LOW();
    /* Command: W_TX_PAYLOAD */
    RF24_CSN_LOW();
    _spi_transfer(RF24_CMD_W_TX_PAYLOAD);
    while (Len--)
    {
        _spi_transfer(*ptr++);
    }
    RF24_CSN_HIGH();

    // Start transmission
    RF24_CE_HIGH();

    timeout = 200;
    do
    {
        delay_us(100);
        status = _spi_read_reg(RF24_REG_STATUS);
        /* Maximum number of TX retransmits reached */
        if (status & BIT(RF24_SHIFT_MAX_RT))
        {
            _flush_tx();
            /* Clear interrupt to enable further communication */
            _spi_write_reg(RF24_REG_STATUS, BIT(RF24_SHIFT_MAX_RT));
            return RF_STATUS_MAX_RT;
        }
        if (status & BIT(RF24_SHIFT_TX_DS))
        {
            return RF_STATUS_OK;
        }
    } while (timeout--);

    return RF_STATUS_TIMEOUT;
}

FlagStatus RF24_RxFifoEmpty(void)
{
    return (FlagStatus)(_spi_read_reg(RF24_REG_FIFO_STATUS) & BIT(RF24_SHIFT_RX_EMPTY));
}

uint8_t RF24_GetStatus(void)
{
    return _spi_read_reg(RF24_REG_STATUS);
}

FlagStatus RF24_IsDataReady(void)
{
    if (RF24_GetStatus() & BIT(RF24_SHIFT_RX_DR))
        return SET;
    /* or RX */
    return (FlagStatus)!RF24_RxFifoEmpty();
}

/*******************************************************************************
 * @brief   Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
 * @param   None.
 * @retval  Payload length
 ******************************************************************************/
uint8_t RF24_DataLen(void)
{
    uint8_t len;

    /* static length */
    if (PayloadLen)
        return PayloadLen;
    /* or dynamic length */
    RF24_CSN_LOW();
    _spi_transfer(RF24_CMD_R_RX_PL_WID);
    len = _spi_transfer(0xFF);
    RF24_CSN_HIGH();
    return len;
}

/*******************************************************************************
 * @brief   Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO
 * @param   Data - pointer to the rx data buffer
 * @retval  Payload length
 ******************************************************************************/
uint8_t RF24_GetData(uint8_t *Data)
{
    uint8_t len;

    len = RF24_DataLen();
    // Flush RX FIFO if the read value is larger than 32 bytes
    if (len > 32)
    {
        len = 0;
        _flush_rx();
        goto ret;
    }

    /* read rx payload */
    RF24_CSN_LOW();
    _spi_transfer(RF24_CMD_R_RX_PAYLOAD);
    for (int i = 0; i < len; i++)
    {
        Data[i] = _spi_transfer(0xFF);
    }
    RF24_CSN_HIGH();

ret:
    /* Clear RX_DR flag */
    _spi_write_reg(RF24_REG_STATUS, BIT(RF24_SHIFT_RX_DR));
    return len;
}

#ifdef DEBUG
#include <stdio.h>
void RF24_DumpReg(void)
{
    uint8_t buf[5] = {0};
    for (uint8_t i = 0; i <= 0x1D; i++)
    {
        printf("%X: ", i);
        if (i == 0xA || i == 0xB || i == 0x10)
        {
            _spi_read_bytes(i, buf, 5);
            for (uint8_t j = 0; j < 5; j++)
            {
                 printf("%X ", buf[j]);
            }
            printf("\r\n");
        }
        else
        {
            printf("%X\r\n", _spi_read_reg(i));
        }
    }
}
#endif

// Initialize with default parameters
void RF24_DefaultInit(void)
{
    RF24_ConfigTypeDef cfg = NRF24L01_CONFIG_DEFAULT;
    cfg.Channel = 52;
    cfg.DataRate = RF24_2MBPS;
    cfg.PowerLevel = RF24_PWR_LVL3;

    RF24_GpioInit();
    RF24_Config(&cfg);
}
