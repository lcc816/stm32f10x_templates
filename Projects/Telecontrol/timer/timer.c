/*******************************************************************************
* @file     --> timer.c
* @author   --> lcc
* @version  --> v1.0
* @date     --> 22-Dec-2022
* @brief    --> 定时器配置. Tout = ((arr+1)*(psc+1))/Fclk
*               启用 Timer3 通用定时器
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "timer.h"

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* @brief    --> 定时器中断配置
* @param    --> arr: 自动重装值
*               psc: 时钟预分频数
* @retval   --> None
*******************************************************************************/
void TIM3_Int_Init(uint16_t arr, uint16_t psc)
{
    /* 时基初始化 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;    // 时基配置
    NVIC_InitTypeDef NVIC_InitStructure;              // 中断优先级配置

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 使能定时器时钟
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 设置时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);  // 使能更新中断

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; // TIMx 中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //先占优先级0级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //从优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM3, ENABLE);                      //使能 TIMx 外设
}
