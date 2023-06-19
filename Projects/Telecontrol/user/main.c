/**
 ******************************************************************************
 * @file    main.c
 * @author  lcc
 * @version
 * @date    21-Dec-2022
 * @brief
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "delay.h"
#include "rich_led.h"
#include "key.h"
#include "timer.h"
#include "joystick.h"
#include "usart.h"
#include "rf24.h"
#include "string.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define COMM_FAILED_LIMIT 10

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
u8 One_Ms_Timing = 0; // 1ms全局时基
u8 RF_Send_Tick = 0;
u8 Key_Scan_Tick = 0;

const u8 peer_addr[5] = {'n', 'e', 'w', 'b', 'e'};
const u8 my_addr[5]   = {'c', 'n', 't', 'r', 'l'};
u16 sn;
u8 airplane_locked = 1;

/* External interrupt flag */
u8 Ext_Int_Flag = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

// 计算累加和校验
uint8_t calc_sum(uint8_t *buf, uint8_t len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += buf[i];
    }
    return sum;
}

/*******************************************************************************
 * @brief    --> 主函数
 * @param    --> None
 * @retval   --> None
 *******************************************************************************/
int main(void)
{
    // 变量定义
    u16 key_map;
    u16 key_map_last = 0;
    s16 throttle, yaw, roll, pitch;
    u16 adc_thro, adc_yaw, adc_roll, adc_pitch;
    s16 dlt_yaw, dlt_roll, dlt_pitch;
    u8 buf[32];
    int i;
    u16 fail_cnt = 0;
    u8 len;
    int status;

    // 注：程序中使用中断时，NVIC分组设置应尽量位于程序起始处，并且在设置后尽量不要再更改NVIC分组
    //------------------------------------------------------------------------------------------
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);      //NVIC分组2：2位抢占优先级，2位响应优先级

    delay_init();           // 延时初始化（注：调用延时函数之前，必须先调用delay_Init()将SysTick初始化）

    USART1_Configuration(115200);

    led_instance_init();    // 初始化rich_led实例
    led_action_set(ACTION_ID_NORMAL_RUNNING);
    led_action_set(ACTION_ID_AIRPLANE_LOCK);

    TIM3_Int_Init(999, 71); // 定时器3初始化, 每1ms中断

    KEY_Init();             // 初始化按键硬件接口

    joystick_init();        // 手柄初始化

    RF24_DefaultInit();     // 2.4G无线射频初始化
    RF24_SetRfParams(RF24_1MBPS, RF24_PWR_LVL3, RESET);

    while (!RF24_CheckAvailable())
    {
        delay_ms(100);
#ifdef DEBUG
        printf("RF24 not available...\r\n");
#endif
    }

    // 初始化OLED
    //-----------------------------------------
    //OLED_Init();                // 初始化OLED
    //OLED_Clear();                 // 清屏
    //OLED_ShowString(0, 0, "2.4G Telecontrol");
    //-----------------------------------------

    RF24_SetRxAddr((uint8_t *)my_addr, 5);
    RF24_SetTxAddr((uint8_t *)peer_addr, 5);
    RF24_RxMode();
#ifdef DEBUG
    //RF24_DumpReg();
#endif
    sn = 0;

#ifdef DEBUG
    printf("initialized...\r\n");
#endif

    joystick_get_filtered_data(&adc_thro, &adc_yaw, &adc_roll, &adc_pitch);
    // 解锁之前油门置于最小
    while (adc_thro >= 10)
    {
        delay_ms(100);
#ifdef DEBUG
        printf("Throttle %u too big\r\n", adc_thro);
#endif
        joystick_get_filtered_data(&adc_thro, &adc_yaw, &adc_roll, &adc_pitch);
    }

    delay_ms(100); // 待读数稳定后校准
    joystick_get_filtered_data(&adc_thro, &adc_yaw, &adc_roll, &adc_pitch);
    dlt_yaw = adc_yaw - 2048;
    dlt_roll = adc_roll - 2048;
    dlt_pitch = adc_pitch - 2048;

    while (1)
    {
        if (Key_Scan_Tick == 1)
        {
            Key_Scan_Tick = 0;
            key_map = KEY_Scan();
            if (0 != key_map && key_map != key_map_last)
            {
                // 有按键被按下
#ifdef DEBUG
                printf("key_map: %02x\r\n", key_map);
#endif
                memset(buf, 0, 32);
                sn++;
                buf[0] = sn & 0xFF;
                buf[1] = (sn >> 8) & 0xFF;
                buf[2] = 7;
                buf[3] = 0x02; // indicates keymap
                buf[4] = key_map & 0xFF;
                buf[5] = (key_map >> 8) & 0xFF;
                buf[6] = calc_sum(buf, 6);
                RF24_TxMode();
                status = RF24_SendData(buf, 7);
#ifdef DEBUG
                printf("keymap tx status %d\r\n", status);
#endif
                if (status == RF_STATUS_OK)
                {
                    fail_cnt = 0;
                    if (key_map & KEY_BIT(KEY_1))
                    {
                        if (airplane_locked)
                            airplane_locked = 0;
                        else
                            airplane_locked = 1;
                    }
                }
                else
                {
                    if (fail_cnt < COMM_FAILED_LIMIT)
                        fail_cnt++;
#ifdef DEBUG
                    printf("fail cnt %d\r\n", fail_cnt);
#endif
                }
                RF24_RxMode();
            }
            key_map_last = key_map;
        }

        if (airplane_locked)
            goto check_status;

        if (RF_Send_Tick == 1)
        {
            RF_Send_Tick = 0;
            joystick_get_filtered_data(&adc_thro, &adc_yaw, &adc_roll, &adc_pitch);
            throttle = adc_thro; // 0 ~ 4095
            yaw = adc_yaw - 2048 - dlt_yaw; // -2048 ~ 2048
            roll = adc_roll - 2048 - dlt_roll;
            pitch = adc_pitch - 2048 - dlt_pitch;
            memset(buf, 0, 32);
            sn++;
            buf[0] = sn & 0xFF;
            buf[1] = (sn >> 8) & 0xFF;
            buf[2] = 13;
            buf[3] = 0x01;
            buf[4] = throttle & 0xFF;
            buf[5] = (throttle >> 8) & 0xFF;
            buf[6] = yaw & 0xFF;
            buf[7] = (yaw >> 8) & 0xFF;
            buf[8] = roll & 0xFF;
            buf[9] = (roll >> 8) & 0xFF;
            buf[10] = pitch & 0xFF;
            buf[11] = (pitch >> 8) & 0xFF;
            buf[12] = calc_sum(buf, 12);
#ifdef DEBUG
            printf("ADC Avg: %06d %06d %06d %06d\r\n", (int)throttle, (int)yaw,
                   (int)roll, (int)pitch);
#endif
            RF24_TxMode();
            status = RF24_SendData(buf, 13);
#ifdef DEBUG
            printf("joystic tx status %d\r\n", status);
#endif
            if (status == RF_STATUS_OK)
                fail_cnt = 0;
            else if (fail_cnt < COMM_FAILED_LIMIT)
                fail_cnt++;
            RF24_RxMode();
        }

        if (Ext_Int_Flag == 1)
        {
            Ext_Int_Flag = 0;
            do
            {
                memset(buf, 0, 32);
                len = RF24_GetData(buf);
#ifdef DEBUG
                printf("recived %d bytes\r\n", len);
                for (i = 0; i < len; i++)
                    printf("%02x ", buf[i]);
                printf("\r\n");
#endif
            } while (RF24_IsDataReady());
        }

check_status:
        if (fail_cnt >= COMM_FAILED_LIMIT)
        {
            led_action_set(ACTION_ID_AIRPLANE_DISCONNECTED);
        }
        else if (airplane_locked)
        {
            led_action_set(ACTION_ID_AIRPLANE_LOCK);
        }
        else
        {
            led_action_set(ACTION_ID_AIRPLANE_UNLOCK);
        }
    }
}
