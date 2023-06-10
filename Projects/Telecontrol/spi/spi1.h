/*******************************************************************************
 * @file     spi1.h
 * @author   lcc
 * @version  
 * @date     2022-12-28
 * @brief    
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI1_H
#define __SPI1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported functions ------------------------------------------------------- */
void SPI1_Init(void);
void SPI1_SetSpeed(u8 SPI_DivideFrequency);
u8 SPI1_ReadWriteByte(u8 TxData);

#endif /* __SPI1_H */
