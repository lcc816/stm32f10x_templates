/**
 ******************************************************************************
 * @file    main.c
 * @author  Lichangchun
 * @version
 * @date    13-April-2017
 * @brief
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "delay.h"
#include "rich_led.h"
#include "usart.h"
#include "timer.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

extern volatile char val;

/*******************************************************************************
 * @brief    --> 主函数
 * @param    --> None
 * @retval   --> None
 *******************************************************************************/
int main(void)
{
    delay_init();
    /* 设置NVIC中断分组2: 2位抢占优先级，2位响应优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Configuration(115200);
    led_instance_init();
    led_action_set(ACTION_ID_AIRPLANE_LOCK);
    TIM3_Int_Init(999, 72); // 定时器3初始化, 每1ms中断
    printf("Hello World!\r\n");

    char rcd = val;

    while (1)
    {
        delay_ms(100);
        if (rcd == val)
            continue;
        rcd = val;
        printf("received %c\r\n", rcd);
        switch (val)
        {
        case '0': led_action_set(0); break;
        case '1': led_action_set(1); break;
        case '2': led_action_set(2); break;
        case '3': led_action_set(3); break;
        case '4': led_action_set(4); break;
        default: break;
        }
    }
}
