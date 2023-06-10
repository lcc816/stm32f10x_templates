/*******************************************************************************
 * @file     joystick.h
 * @author   lcc
 * @version  
 * @date     2022-12-21
 * @brief    
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __JOYSTICK_H
#define __JOYSTICK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "delay.h"

/* Constant define -----------------------------------------------------------*/
#define ADC_CHAN_NUM       4        // ADC通道数
#define ADC_CHAN_THRO      1        // ADC通道1 油门
#define ADC_CHAN_YAW       2        // ADC通道2 偏航
#define ADC_CHAN_ROLL      3        // ADC通道3 滚转
#define ADC_CHAN_PITCH     6        // ADC通道6 俯仰
#define ADC_SAMPLE_NUM     5        // ADC采样次数

/* Exported variables --------------------------------------------------------*/
//extern u16 adc_buffer[ADC_SAMPLE_NUM][ADC_CHAN_NUM];    // DMA搬运数据存放处


/* Exported functions ------------------------------------------------------- */
void joystick_init(void);
void joystick_get_filtered_data(u16 *thro, u16 *yaw, u16 *roll, u16 *pitch);

#endif /* __JOYSTICK_H */
