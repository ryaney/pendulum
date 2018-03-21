#include "stm32f10x_i2c.h"
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "i2c.h"
/*=====================================================================================================*/
/*=====================================================================================================*/
vu8* I2C_ReadPtr;
vu8* I2C_WritePtr;
unsigned int I2C_TimeCnt = I2C_TIME;
/*=====================================================================================================*/
/*=====================================================================================================*/
DMA_InitTypeDef DMA_InitStruct;
/*=====================================================================================================*/
/*=====================================================================================================*
**��� : I2C_Config
**�\�� : I2C �]�w & �t�m
**��J : None
**��X : None
**�ϥ� : I2C_Config();
**=====================================================================================================*/
/*=====================================================================================================*/
void I2C_Config( void )
{
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	/* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);              //ʹ��peripheral clocks
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);		

	//config afio �ض���I2C1 io�ڵ�PB8��PB9
	GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);
	
	//config gpio PB
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;						//ʹ��PB9��SDA//ʹ��PB8��SCL
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_SetBits(GPIOB,GPIO_Pin_8 | GPIO_Pin_9);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;						//ʹ��PB9��SDA//ʹ��PB8��SCL
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;									//i2c�����Ҫ����
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = DMA1_Channel7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	DMA_ClearFlag(DMA1_FLAG_GL7);
	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStruct.DMA_PeripheralBaseAddr = (u32)I2C1_DR_Address;
	DMA_InitStruct.DMA_MemoryBaseAddr = (u32)0;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStruct.DMA_BufferSize = 20;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStruct.DMA_Priority = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel7, &DMA_InitStruct);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);		//����DMA�ж�

	I2C_Cmd(I2C1, DISABLE);
	I2C_DeInit(I2C1);	
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = I2C1_SPEED;
	I2C_Init(I2C1, &I2C_InitStruct);
	I2C_Cmd(I2C1, ENABLE);
	I2C_DMACmd(I2C1, ENABLE);
}
/*=====================================================================================================*/
/*=====================================================================================================*
������I2C_DMA_Read(ReadBuf, SlaveAddr, ReadAddr, (u8*)(&NumByte));
��������I2C ReadAddr��ַ������NumByte�����ݣ�������ReadBuf��
������*ReadBuf, SlaveAddr, ReadAddr, *NumByte
���أ��ɹ���SUCCESS	ʧ�ܣ���ʱʱ��
**=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_Read( u8* ReadBuf, u8 SlaveAddr, u8 ReadAddr, u8* NumByte )
{
	I2C_ReadPtr = NumByte;

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))		//���EV5״̬�����
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();
	
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))		//���EV6״̬�����
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_SendData(I2C1, ReadAddr);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))				//���EV8״̬�����
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);
	
	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))					//���EV5״̬�����
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Receiver);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))			//���EV6״̬�����
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();
	
	if((u16)(*NumByte) == 1)
	{
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		(void)I2C1->SR2;
		
		I2C_GenerateSTOP(I2C1, ENABLE);

		I2C_TimeCnt = I2C_TIME;
		while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		*ReadBuf = I2C_ReceiveData(I2C1);
		(u16)(*NumByte)--;

		I2C_TimeCnt = I2C_TIME;
		while(I2C1->CR1 & I2C_CR1_STOP)
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

		I2C_AcknowledgeConfig(I2C1, ENABLE);
	}
	else {
		DMA_ClearFlag(DMA1_FLAG_GL7);
		DMA_InitStruct.DMA_MemoryBaseAddr = (u32)ReadBuf;
		DMA_InitStruct.DMA_BufferSize = (u32)(*NumByte);
		DMA_Init(DMA1_Channel7, &DMA_InitStruct);
		I2C_DMALastTransferCmd(I2C1, ENABLE);
		DMA_Cmd(DMA1_Channel7, ENABLE);
	}
	//I2C������DMA�жϷ�����
	I2C_TimeCnt = I2C_TIME;
	while(*NumByte > 0)
		if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();

	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*
�����I2C_DMA_Read(ReadBuf, SlaveAddr, ReadAddr, NumByte);
��������I2C ReadAddr��ַ������NumByte�����ݣ�������ReadBuf��
������*ReadBuf, SlaveAddr, ReadAddr, *NumByte
���أ��ɹ���SUCCESS	ʧ�ܣ���ʱʱ��
**=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_ReadReg( u8 SlaveAddr, u8 ReadAddr, u8* ReadBuf, u8 NumByte )
{
	I2C_DMA_Read(ReadBuf, SlaveAddr, ReadAddr, (u8*)(&NumByte));
	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*
������I2C_DMA_Write( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8* NumByte )
��������I2C ReadAddr��ַ��д��1�����ݣ�������WriteBuf��
������*WriteBuf, SlaveAddr, ReadAddr, *NumByte
���أ��ɹ���SUCCESS	ʧ�ܣ���ʱʱ��
**=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_Write( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8* NumByte )
{
	I2C_WritePtr = NumByte;

	I2C_TimeCnt = I2C_TIME;
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
		if((I2C_TimeCnt--) == 0) 
			return I2C_TimeOut();

	I2C_GenerateSTART(I2C1, ENABLE);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
		if((I2C_TimeCnt--) == 0) 
			return I2C_TimeOut();

	I2C_TimeCnt = I2C_TIME;
	I2C_Send7bitAddress(I2C1, SlaveAddr, I2C_Direction_Transmitter);

	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		if((I2C_TimeCnt--) == 0)
			return I2C_TimeOut();

	I2C_SendData(I2C1, WriteAddr);
	I2C_TimeCnt = I2C_TIME;
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))					//���EV8״̬�����
		if((I2C_TimeCnt--) == 0) 
			return I2C_TimeOut();

	u8* ptr = WriteBuf;
	while((u16)(*I2C_WritePtr)--)
	{
		I2C_SendData(I2C1, *ptr);
		ptr++;
		I2C_TimeCnt = I2C_TIME;
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))					//���EV8״̬�����
			if((I2C_TimeCnt--) == 0)	return I2C_TimeOut();
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	return SUCCESS;
}
/*=====================================================================================================*/
/*=====================================================================================================*
������I2C_DMA_Write( u8* WriteBuf, u8 SlaveAddr, u8 WriteAddr, u8* NumByte )
��������I2C ReadAddr��ַ��д��1�����ݣ�������WriteBuf��
������*WriteBuf, SlaveAddr, ReadAddr, *NumByte
���أ��ɹ���SUCCESS	ʧ�ܣ���ʱʱ��
**=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_DMA_WriteReg( u8 SlaveAddr, u8 WriteAddr, u8* WriteBuf, u8 NumByte )
{
	I2C_DMA_Write(WriteBuf, SlaveAddr, WriteAddr, (u8*)(&NumByte));

	return SUCCESS;
}

/*=====================================================================================================*/
/*=====================================================================================================*/
void DMA1_Channel7_IRQHandler( void )
{
	
	I2C1_Recv_DMA_IRQ();
}

/*=====================================================================================================*/
/*=====================================================================================================*
**��� : I2C1_Recv_DMA_IRQ
**�\�� : I2C1 Recv DMA IRQ
**��J : None
**��X : None
**�ϥ� : I2C1_Recv_DMA_IRQ();
**=====================================================================================================*/
/*=====================================================================================================*/
void I2C1_Recv_DMA_IRQ( void )
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7) != RESET) {
		I2C_GenerateSTOP(I2C1, ENABLE);
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA_ClearFlag(DMA1_FLAG_TC7);
		*I2C_ReadPtr = 0;
	}
}

/*=====================================================================================================*/
/*=====================================================================================================*
**��� : I2C_TimeOut
**�\�� : I2C TimeOut
**��J : None
**��X : None
**�ϥ� : I2C_TimeOut();
**=====================================================================================================*/
/*=====================================================================================================*/
u32 I2C_TimeOut( void )
{
	return ERROR;
}
