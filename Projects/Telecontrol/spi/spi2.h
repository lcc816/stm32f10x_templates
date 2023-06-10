/*******************************************************************************
 * @file     spi2.h
 * @author   lcc
 * @version  
 * @date     2022-12-28
 * @brief    
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI2_H
#define __SPI2_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported functions ------------------------------------------------------- */
void SPI2_Init(void);
void SPI2_SetSpeed(u8 SPI_DivideFrequency);
u8 SPI2_ReadWriteByte(u8 TxData);

#endif /* __SPI2_H */
