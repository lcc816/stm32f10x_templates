/*******************************************************************************
 * @file     key.c
 * @author   lcc
 * @version  
 * @date     24-Dec-2022
 * @brief    
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "key.h"

/* Private typedef -----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* 按键引脚定义, 根据实际电路更改 */
static KEY_TypeDef KEY_Array[] =
{
    [KEY_WKUP] = {GPIOA, GPIO_Pin_0, RCC_APB2Periph_GPIOA, 1, 0},
    [KEY_1] = {GPIOC, GPIO_Pin_14, RCC_APB2Periph_GPIOC, 0, 0},
    [KEY_UP] = {GPIOA, GPIO_Pin_11, RCC_APB2Periph_GPIOA, 0, 0},
    [KEY_DOWN] = {GPIOA, GPIO_Pin_12, RCC_APB2Periph_GPIOA, 0, 0},
    [KEY_LEFT] = {GPIOA, GPIO_Pin_15, RCC_APB2Periph_GPIOA, 0, 0},
    [KEY_RIGHT] = {GPIOB, GPIO_Pin_3, RCC_APB2Periph_GPIOB, 0, 0},
};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * @brief   按键初始化
 * @param   None
 * @retval  None
 ******************************************************************************/
void KEY_Init(void)
{
    int i;
    uint32_t RCC_APB2Periph = 0;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // JTAG失能

    for (i = 0; i < KEY_IDX_MAX; i++)
    {
        RCC_APB2Periph |= KEY_Array[i].RCC_APB2Periph;
    }
    RCC_APB2PeriphClockCmd(RCC_APB2Periph, ENABLE);

    for (i = 0; i < KEY_IDX_MAX; i++)
    {
        GPIO_InitStructure.GPIO_Pin = KEY_Array[i].GPIO_Pin;
        /* 根据按下的电平初始化 */
        if (KEY_Array[i].Press_Level == 0)
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
        else
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
        GPIO_Init(KEY_Array[i].GPIO_Port, &GPIO_InitStructure);
    }
}

/*******************************************************************************
 * @brief   按键扫描, 扫描间隔 10ms, 计数大于 5 认为被按下
 * @param   None
 * @retval  the bitmap of all treggered keys
 ******************************************************************************/
uint16_t KEY_Scan(void)
{
    int i = 0;
    uint16_t bitmap = 0;

    for (i = 0; i < KEY_IDX_MAX; i++)
    {
        if (KEY_Array[i].Press_Level != KEY_LEVEL(i))
        {
            KEY_Array[i].Press_Cnt = 0;
            bitmap &= ~KEY_BIT(i);
        }
        else
        {
            KEY_Array[i].Press_Cnt++;
            if (KEY_Array[i].Press_Cnt > 5)
                bitmap |= KEY_BIT(i);
        }
    }

    return bitmap;
}
