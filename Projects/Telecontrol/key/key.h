/*******************************************************************************
* @file     --> key.h
* @author   --> lcc
* @version  --> v1.1
* @date     --> 24-Dec-2022
* @brief    -->
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KEY_H
#define __KEY_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private define ------------------------------------------------------------*/
#define KEY_LEVEL(idx)  GPIO_ReadInputDataBit(KEY_Array[idx].GPIO_Port, \
                                              KEY_Array[idx].GPIO_Pin)
#define KEY_BIT(idx)    (1 << idx)

/* Exported types ------------------------------------------------------------*/
enum key_index
{
    KEY_WKUP,
    KEY_1,
    KEY_UP,
    KEY_DOWN,
    KEY_LEFT,
    KEY_RIGHT,
    KEY_IDX_MAX
};

typedef struct
{
    GPIO_TypeDef* GPIO_Port;
    uint16_t GPIO_Pin;
    uint32_t RCC_APB2Periph;
    uint8_t Press_Level;
    uint8_t Press_Cnt;
} KEY_TypeDef;

/* Exported define -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void KEY_Init(void);
uint16_t KEY_Scan(void);

#endif /* __KEY_H */
