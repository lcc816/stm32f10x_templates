/*******************************************************************************
 * @file     conf_led.c
 * @author   lcc
 * @version
 * @date     2023-05-10
 * @brief
 ******************************************************************************/

#include "rich_led.h"
#include "led.h"

void led_green_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(LED_0_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_0_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_0_GPIO_PORT, &GPIO_InitStructure);
    GPIO_SetBits(LED_0_GPIO_PORT, LED_0_PIN);
    // JTAG失能,用PB3，PB4，PA15做普通IO，PA13&14用于SWD调试
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
}

void led_green_on(void)
{
    LED_ON(0);
}

void led_green_off(void)
{
    LED_OFF(0);
}

void led_blue_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(LED_1_GPIO_CLK, ENABLE);

    GPIO_InitStructure.GPIO_Pin = LED_1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_1_GPIO_PORT, &GPIO_InitStructure);
    GPIO_SetBits(LED_1_GPIO_PORT, LED_1_PIN);
}

void led_blue_on(void)
{
    LED_ON(1);
}

void led_blue_off(void)
{
    LED_OFF(1);
}

struct led_config led_instance[] =
{
    [LED_ID_GREEN] = {led_green_gpio_init, led_green_on, led_green_off},
    [LED_ID_BLUE] = {led_blue_gpio_init, led_blue_on, led_blue_off}
};

struct led_action action_instance[] =
{
    [ACTION_ID_AIRPLANE_UNLOCK] = {LED_ID_BLUE, -1, 0, 1, 1},
    [ACTION_ID_AIRPLANE_LOCK] = {LED_ID_BLUE, 1000, 500, 500, 1},
    [ACTION_ID_BATTERY_LOW] = {LED_ID_GREEN, 3000, 500, 500, 2},
    [ACTION_ID_FLIGHT_ERROR] = {LED_ID_GREEN, 1000, 100, 100, 3}
};

struct led_action_record action_record[NUM_LED_ID];
