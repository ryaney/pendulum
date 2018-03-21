/*********************************************************************************
*                                ɽè�ɿأ�Lynx��
*                                 for LynxFly
*
* Version   	: V1.0
* By        	: Lynx@sia 84693469@qq.com
*
* For       	: Stm32f405RGT6
* Mode      	: Thumb2
* Description   : mpu6050���� ��Ϯ����
*
*				
* Date          : 2013.XX.XX
*******************************************************************************/
/*==============================================================================================*/
/*==============================================================================================*/
#include "i2c.h"
#include "module_mpu6050.h"
/*==============================================================================================*/
/*==============================================================================================*
**���� : MPU6050_Init
**���� : ��ʼ��MPU6050
**ݔ�� : None
**ݔ�� : None
**ʹ�� : MPU6050_Init()
**==============================================================================================*/
/*==============================================================================================*/
void MPU6050_Init( void )
{
  uint8_t MPU6050_Init_Data[6] = {
      0x01, /* MPU6050_PWR_MGMT_1 */
      0x03, /* MPU6050_CONFIG */
      0x18, /* MPU6050_GYRO_CONFIG +-2000dps */
      0x08, /* MPU6050_ACCEL_CONFIG +-4G */
      0x32, /* MPU6050_INT_PIN_CFG */
      0x00	/* MPU6050_USER_CTRL */
		};
  I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1,   MPU6050_Init_Data,   1); 
	mDelay(10);
  I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_CONFIG,       MPU6050_Init_Data+1, 1); 
	mDelay(10);
  I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG,  MPU6050_Init_Data+2, 1); 
	mDelay(10);
  I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, MPU6050_Init_Data+3, 1); 
	mDelay(10);
  I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_INT_PIN_CFG,  MPU6050_Init_Data+4, 1); 
	mDelay(10);
  I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_USER_CTRL,    MPU6050_Init_Data+5, 1); 
	mDelay(10);
}
/*==============================================================================================*/
/*==============================================================================================*/