#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x.h"

#define I2C1_DR_Address	((uint32_t)0x40005410)

#define I2C_TIME        ((uint32_t)65535)
#define I2C1_SPEED      ((uint32_t)400000)
#define	DMA_BUFFERSIZE	4
/*=====================================================================================================*/
/*=====================================================================================================*/

void I2C_Config(void);
uint32_t I2C_DMA_Read( u8*, u8, u8, u8* );
uint32_t I2C_DMA_ReadReg( u8, u8, u8*, u8 );
uint32_t I2C_DMA_Write( u8*, u8, u8, u8* );
uint32_t I2C_DMA_WriteReg( u8, u8, u8*, u8 );
void I2C1_Send_DMA_IRQ( void );
void I2C1_Recv_DMA_IRQ( void );
uint32_t I2C_TimeOut( void );
/*=====================================================================================================*/
/*=====================================================================================================*/

#endif //__I2C_H__
