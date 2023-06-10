 /*******************************************************************************
 * @file     joystick.c
 * @author   lcc
 * @version  
 * @date     2022-12-21
 * @brief    封装手柄的操作函数
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "joystick.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
u16 adc_buffer[ADC_SAMPLE_NUM][ADC_CHAN_NUM] = { 0 }; // DMA搬运数据存放处

/*******************************************************************************
 * @brief   摇杆的ADC初始化, 启用4个通道, 启用timer4触发
 *          ADC1_CH1---PA1
 *          ADC1_CH2---PA2
 *          ADC1_CH3---PA3
 *          ADC1_CH6---PA6
 * @param   None
 * @retval  None
 ******************************************************************************/
void ADC1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    // NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1, ENABLE); // 使能PA端口和ADC1时钟

    // 模拟输入：PA1、PA2、PA3、PA6
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;   // 模拟输入引脚
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  // 输入模式无需设置端口速度
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置ADC1_CH1、ADC1_CH2、ADC1_CH3、ADC1_CH6
    // ADC工作条件：ADC的时钟频率<=14MHz && ADC采样频率<=1MHz
    // ADC的总转换周期(必须>=1us) = （采样周期(1.5～239.5) + 12.5(固定转换周期)） / ADC时钟频率
    // 当：ADC的时钟频率==12MHz、采样周期==1.5，ADC转换周期≈1.17us
    //---------------------------------------------------------------------------------------------------------------------
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);      // ADC时钟6分频：72M/6=12M

    ADC_DeInit(ADC1);          // 复位ADC1

    // 设置ADC1的工作模式
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // ADC工作模式:ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;   // "扫描转换模式"使能。即：ADC工作在多通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // "连续转换模式"失能。即：ADC工作在单次转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T4_CC4; // 设置为定时器4外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // ADC数据对齐方式：右对齐
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CHAN_NUM;  // 设置顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);      // 初始化ADC1

    // 为了能够正确地配置每一个 ADC 通道，用户在调用 ADC_Init()之后，
    // 必须调用ADC_xxxChannelConfig()来配置每个所使用通道的转换次序和采样时间。
    //--------------------------------------------------------------------
    // 此函数：配置的是"ADC1"的通道"1"，它在规则组中的采样顺序是"1"，采样周期是"ADC_SampleTime_239Cycles5"
    ADC_RegularChannelConfig(ADC1, ADC_CHAN_THRO, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_CHAN_YAW, 2, ADC_SampleTime_239Cycles5); // 参数：ADC1,要设置的ADC通道序号,规则组采样顺序,采样周期
    ADC_RegularChannelConfig(ADC1, ADC_CHAN_ROLL, 3, ADC_SampleTime_239Cycles5);
    // 此函数：配置的是"ADC1"的通道"3"，它在规则组中的采样顺序是"4"，采样周期是"ADC_SampleTime_239Cycles5"
    ADC_RegularChannelConfig(ADC1, ADC_CHAN_PITCH, 4, ADC_SampleTime_239Cycles5);
    //---------------------------------------------------------------------------------------------------------------------
    /*
    //ADC1的中断NVIC设置
    //---------------------------------------------------------------------------------
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;     // ADC1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   // 抢占优先级3级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;    // 子优先级3级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    // 使能ADC1_2_IRQn中断
    NVIC_Init(&NVIC_InitStructure);         // 初始化NVIC寄存器

    ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE );       // 允许ADC1的EOC中断
    //---------------------------------------------------------------------------------
    */

    // 开启ADC的DMA支持
    ADC_DMACmd(ADC1, ENABLE);     // 要实现DMA功能，还需独立配置DMA通道等参数

    ADC_Cmd(ADC1, ENABLE);      // 使能ADC1

    ADC_ExternalTrigConvCmd(ADC1, ENABLE);  // 使能 外部触发转化 ADC_ExternalTrigConv_T4_CC4

    ADC_ResetCalibration(ADC1);     // 使能ADC1复位校准

    while (ADC_GetResetCalibrationStatus(ADC1)); // 等待ADC1复位校准结束

    ADC_StartCalibration(ADC1);      // 开启ADC1校准

    while (ADC_GetCalibrationStatus(ADC1));   // 等待ADC1校准结束
}

/*******************************************************************************
 * @brief   DMA初始化, 用于自动搬运ADC数据到指定的缓存区
 * @param   None
 * @retval  None
 ******************************************************************************/
void DMA1_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);    // 使能DMA时钟

    // DMA_CH1(ADC1)参数设置
    //--------------------------------------------------------------------------------------------------------------------------
    DMA_DeInit(DMA1_Channel1);              // 将DMA的通道1寄存器重设为缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;      // DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)adc_buffer;      // DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;        // 内存作为数据传输的目的地
    DMA_InitStructure.DMA_BufferSize = ADC_CHAN_NUM * ADC_SAMPLE_NUM;    // DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;    // 外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;      // 内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;   // 数据宽度为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;        // 工作在循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;       // DMA通道 x拥有高优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;         // DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);         // 根据DMA_InitStruct中指定的参数初始化DMA的通道
    //--------------------------------------------------------------------------------------------------------------------------

    // DMA_CH1(ADC1)的中断NVIC设置
    //----------------------------------------------------------------------------------------------
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn; // DMA_CH1(ADC1)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   // 子优先级2级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    // 使能DMA_CH1(ADC1)
    NVIC_Init(&NVIC_InitStructure);        // 初始化NVIC寄存器

    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC,ENABLE );    // 允许DMA_CH1(ADC1)的DMA_IT_TC中断
    //----------------------------------------------------------------------------------------------

    DMA_Cmd(DMA1_Channel1, ENABLE);  // 启动DMA通道
}

/*******************************************************************************
 * @brief   启用Timer4来触发ADC转换
 * @param   arr: 自动重装值
 * @param   psc: 时钟预分频数
 * @retval  None
 ******************************************************************************/
void TIM4_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    // NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //时钟使能

    // 定时器TIM4初始化
    TIM_TimeBaseStructure.TIM_Period = arr;    //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;    //设置用来作为TIMx时钟频率除数的预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;   //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);   //根据指定的参数初始化TIMx的时间基数单位

    // TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);  // 使能更新中断
    // NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; // TIMx 中断
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //先占优先级0级
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; //从优先级3级
    // NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    // NVIC_Init(&NVIC_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //选择定时器模式:TIM脉冲宽度调制模式1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_Pulse = 100;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;  //输出极性:TIM输出比较极性低
    /*
     输出比较极性的指的是你在比较匹配之后输出口输出的极性。
     这个函数就是让你设置有效极性，也就是设置比较输出的有效电平。你可以设置为高电平有效或者低电平有效。
     如果设置为高电平有效，那么当定时器比较匹配之后，输出口输出高电平，否则就反一下。
    */
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); //初始化外设TIM4_CH4
    TIM_CtrlPWMOutputs(TIM4, ENABLE);  //使能 pwm 输出通道 产生上升沿更新信号 
    TIM_Cmd(TIM4, ENABLE);     //使能TIMx 定时器
}

/*******************************************************************************
 * @brief   DMA 中断处理函数
 * @param   None
 * @retval  None
 ******************************************************************************/
void  DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET)
    {
#if 0
        int i;
        // 中断处理代码， 打印 各个方向 读取到的 ADC模拟电压值
        for (i = 0; i < ADC_SAMPLE_NUM; i++)
            printf("%d, %d, %d, %d, %d\r\n", i, adc_buffer[i][0], adc_buffer[i][1],
                   adc_buffer[i][2], adc_buffer[i][3]);
        printf("\n");
#endif
        DMA_ClearITPendingBit(DMA1_IT_TC1);  // 清除中断 标志
    }
}

/*******************************************************************************
 * @brief   摇杆所需外设初始化
 * @param   None
 * @retval  None
 ******************************************************************************/
void joystick_init(void)
{
    DMA1_Init();
    ADC1_Init();
    TIM4_Init(2999, 239); // 10ms
}

/*******************************************************************************
 * @brief   摇杆所需外设初始化
 * @param   thro 指向油门值的指针
 * @param   yaw 指向偏航角值的指针
 * @param   roll 指向滚转角值的指针
 * @param   pitch 指向俯仰角值的指针
 * @retval  None
 ******************************************************************************/
void joystick_get_filtered_data(u16 *thro, u16 *yaw, u16 *roll, u16 *pitch)
{
    int i;
    u32 sum_thro = 0, sum_yaw = 0, sum_roll = 0, sum_pitch = 0;
    for (i = 0; i < ADC_SAMPLE_NUM; i++)
    {
        sum_thro += adc_buffer[i][0];
        sum_yaw += adc_buffer[i][1];
        sum_roll += adc_buffer[i][2];
        sum_pitch += adc_buffer[i][3];
    }
    *thro = sum_thro / ADC_SAMPLE_NUM;
    *yaw = sum_yaw / ADC_SAMPLE_NUM;
    *roll = sum_roll / ADC_SAMPLE_NUM;
    *pitch = sum_pitch / ADC_SAMPLE_NUM;
}
