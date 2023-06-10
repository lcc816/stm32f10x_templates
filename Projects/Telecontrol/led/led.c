/**
 ******************************************************************************
 * @file    led.c
 * @author  Lichangchun
 * @version
 * @date    6-Sept-2017
 * @brief
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"

/*******************************************************************************
 * @brief  LED所用的GPIO端口初始化
 * @param  None
 * @retval None
 *******************************************************************************/
void LED_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(LED_0_GPIO_CLK, ENABLE); //使能GPIO端口时钟

    GPIO_InitStructure.GPIO_Pin = LED_0_PIN; //LED 端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //IO口速度为50MHz
    GPIO_Init(LED_0_GPIO_PORT, &GPIO_InitStructure); //根据设定参数初始化
    GPIO_SetBits(LED_0_GPIO_PORT, LED_0_PIN); //LED 引脚输出高
}
