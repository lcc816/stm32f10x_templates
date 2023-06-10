/**
 ******************************************************************************
 * @file    led.h
 * @author  lcc
 * @version
 * @date    2023-01-20
 * @brief
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_H
#define __LED_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
#ifdef MINI_STM32 // Development Board
#define LED_0_PIN                   GPIO_Pin_8
#define LED_0_GPIO_PORT             GPIOA
#define LED_0_GPIO_CLK              RCC_APB2Periph_GPIOA

#define LED_1_PIN                   GPIO_Pin_2
#define LED_1_GPIO_PORT             GPIOD
#define LED_1_GPIO_CLK              RCC_APB2Periph_GPIOD

#else // NewBee
#define LED_0_PIN                   GPIO_Pin_4 // 需关闭JTAG功能
#define LED_0_GPIO_PORT             GPIOB
#define LED_0_GPIO_CLK              RCC_APB2Periph_GPIOB

#define LED_1_PIN                   GPIO_Pin_5
#define LED_1_GPIO_PORT             GPIOB
#define LED_1_GPIO_CLK              RCC_APB2Periph_GPIOB
#endif

#define LED_OFF(x)  GPIO_SetBits(LED_##x##_GPIO_PORT, LED_##x##_PIN)
#define LED_ON(x)   GPIO_ResetBits(LED_##x##_GPIO_PORT, LED_##x##_PIN)
#define LED_FLIP(x) \
    { \
        if (GPIO_ReadOutputDataBit(LED_##x##_GPIO_PORT, LED_##x##_PIN)) \
            GPIO_ResetBits(LED_##x##_GPIO_PORT, LED_##x##_PIN); \
        else \
            GPIO_SetBits(LED_##x##_GPIO_PORT, LED_##x##_PIN); \
    }

/* Exported functions ------------------------------------------------------- */
void LED_Init(void);

#endif /* __LED_H */
