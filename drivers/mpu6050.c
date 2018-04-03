#include <math.h>
#include "stm32f10x.h"
#include "mpu6050.h"
#include "i2c.h"

void MPU6050_Init(void);
int read_imu(void);
void Angle_Calcu(void);

SYS_IMU_SENSOR_T imu_sensor = {
    .initialize = MPU6050_Init,
    .read_imu = read_imu,
    .angle_cal = Angle_Calcu
};

void MPU6050_Init( void )
{
    uint8_t MPU6050_Init_Data[6] = {
      0x01, /* MPU6050_PWR_MGMT_1 */
      0x03, /* MPU6050_CONFIG */
      0x18, /* MPU6050_GYRO_CONFIG +-2000dps */
      0x08, /* MPU6050_ACCEL_CONFIG +-4G */
      0x32, /* MPU6050_INT_PIN_CFG */
      0x00  /* MPU6050_USER_CTRL */
    };

    I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1,   MPU6050_Init_Data,   1); 
    delay_ms(10);
    I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_CONFIG,       MPU6050_Init_Data+1, 1); 
    delay_ms(10);
    I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_GYRO_CONFIG,  MPU6050_Init_Data+2, 1); 
    delay_ms(10);
    I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_ACCEL_CONFIG, MPU6050_Init_Data+3, 1); 
    delay_ms(10);
    I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_INT_PIN_CFG,  MPU6050_Init_Data+4, 1); 
    delay_ms(10);
    I2C_DMA_WriteReg(MPU6050_I2C_ADDR, MPU6050_USER_CTRL,    MPU6050_Init_Data+5, 1); 
    delay_ms(10);

    imu_sensor.Gyr.RawToTrue = 16.3835f;   //2000dps--16.4LSB
    imu_sensor.Acc.RawToTrue = 8192.0f;   //4g--8192LSB  
}


int read_imu(void)
{
    u8 IMU_Buf[20] = {0};
    
    I2C_DMA_ReadReg(MPU6050_I2C_ADDR, MPU6050_ACCEL_XOUT_H,IMU_Buf,14);
    
    imu_sensor.Acc.X = (s16)((IMU_Buf[0]  << 8) | IMU_Buf[1]);
    imu_sensor.Acc.Y = (s16)((IMU_Buf[2]  << 8) | IMU_Buf[3]);
    imu_sensor.Acc.Z = (s16)((IMU_Buf[4]  << 8) | IMU_Buf[5]);
    imu_sensor.Gyr.X = (s16)((IMU_Buf[8]  << 8) | IMU_Buf[9]);
    imu_sensor.Gyr.Y = (s16)((IMU_Buf[10] << 8) | IMU_Buf[11]);
    imu_sensor.Gyr.Z = (s16)((IMU_Buf[12] << 8) | IMU_Buf[13]);
    
    //校正
    imu_sensor.Acc.TrueX = imu_sensor.Acc.Z/imu_sensor.Acc.RawToTrue + 0.02;
    imu_sensor.Acc.TrueY = imu_sensor.Acc.Y/imu_sensor.Acc.RawToTrue - 0.056;
    imu_sensor.Acc.TrueZ = imu_sensor.Acc.X/imu_sensor.Acc.RawToTrue - 0.003;
    //校正
    imu_sensor.Gyr.TrueX = imu_sensor.Gyr.Z/imu_sensor.Gyr.RawToTrue + 1.5;
    imu_sensor.Gyr.TrueY = imu_sensor.Gyr.Y/imu_sensor.Gyr.RawToTrue - 1.94;
    imu_sensor.Gyr.TrueZ = imu_sensor.Gyr.X/imu_sensor.Gyr.RawToTrue - 0.29;
    
    return 0;
}

/*
 * 角度计算
 */
// void Angle_Calcu(void)
// {
//     float temp1, temp2, JSDx, JSDy, JSDz;
//     float TLYx, TLYy, Angle_B, Angle_A;
//     /****************************Y·½Ïò**************************/
//         //¼ÓËÙ¶È(½Ç¶È)
//     JSDx = imu_sensor.Acc.TrueX;    //¶ÁÈ¡XÖá¼ÓËÙ¶È
//     JSDy = imu_sensor.Acc.TrueY;    //¶ÁÈ¡YÖá¼ÓËÙ¶È
//     JSDz = imu_sensor.Acc.TrueZ;    //¶ÁÈ¡ZÖá¼ÓËÙ¶È
//     temp1 = sqrt((JSDx * JSDx + JSDz * JSDz))/JSDy;
//     JSDy = atan(temp1)/3.1415926 * 180;
//     if(JSDy > 0) Angle_B = JSDy - 89.9;
//     if(JSDy < 0) Angle_B = JSDy + 90.1;
//     //ÍÓÂÝÒÇ(½ÇËÙ¶È)
//     TLYx = imu_sensor.Gyr.TrueX;      //¾²Ö¹Ê±½ÇËÙ¶ÈYÖáÊä³öÎª-30×óÓÒ
//     TLYx = (TLYx)/16.384;         //È¥³ýÁãµãÆ«ÒÆ£¬¼ÆËã½ÇËÙ¶ÈÖµ,¸ººÅÎª·½Ïò´¦Àí 
//     //  Angle_gy = Angle_gy + TLYy*0.0056;  //½ÇËÙ¶È»ý·ÖµÃµ½ÇãÐ±½Ç¶È.	

//     /****************************X·½Ïò**************************/
//     temp2 = sqrt((JSDy * JSDy + JSDz * JSDz))/JSDx;
//     JSDx = atan(temp2) / 3.1415926 * 180;
//     if(JSDx > 0) Angle_A = JSDx - 87.2;
//     if(JSDx < 0) Angle_A = JSDx + 92.8;
//     //ÍÓÂÝÒÇ(½ÇËÙ¶È)
//     TLYy = imu_sensor.Gyr.TrueY;      //¾²Ö¹Ê±½ÇËÙ¶ÈYÖáÊä³öÎª-30×óÓÒ
//     TLYy = -(TLYy)/16.384;         //È¥³ýÁãµãÆ«ÒÆ£¬¼ÆËã½ÇËÙ¶ÈÖµ,¸ººÅÎª·½Ïò´¦Àí 
//     //	Angle_gy = Angle_gy + TLYy*0.0056;  //½ÇËÙ¶È»ý·ÖµÃµ½ÇãÐ±½Ç¶È.	

//     //-------»¥²¹ÂË²¨-----------------------

//     //²¹³¥Ô­ÀíÊÇÈ¡µ±Ç°Çã½ÇºÍ¼ÓËÙ¶È»ñµÃÇã½Ç²îÖµ½øÐÐ·Å´ó£¬È»ºóÓë
//     //ÍÓÂÝÒÇ½ÇËÙ¶Èµþ¼ÓºóÔÙ»ý·Ö£¬´Ó¶øÊ¹Çã½Ç×î¸ú×ÙÎª¼ÓËÙ¶È»ñµÃµÄ½Ç¶È
//     //0.5Îª·Å´ó±¶Êý£¬¿Éµ÷½Ú²¹³¥¶È£»0.005ÎªÏµÍ³ÖÜÆÚ5ms	

//     imu_sensor.AngleX = imu_sensor.AngleX + (((Angle_A - imu_sensor.AngleX) * 0.5 + TLYy) * 0.0054);    //X·½Ïò
//     imu_sensor.AngleY = imu_sensor.AngleY + (((Angle_B - imu_sensor.AngleY) * 0.5 + TLYx) * 0.0054);    //Y·½Ïò

// }

void Angle_Calcu(void) {
    /* γ角度 */
    /* alpha角度 */
    float ax, ay, az, t1, t2;
    float gama, alpha;
    ax = imu_sensor.Acc.TrueX;    //¶ÁÈ¡XÖá¼ÓËÙ¶È
    ay = imu_sensor.Acc.TrueY;    //¶ÁÈ¡YÖá¼ÓËÙ¶È
    az = imu_sensor.Acc.TrueZ;    //¶ÁÈ¡ZÖá¼ÓËÙ¶È
    t1 = sqrt(ax * ax + ay * ay + az * az);
    t2 = sqrt(ax * ax + ay * ay);
    gama = acos( az / t1);
    alpha = acos( ax / t2 );

    imu_sensor.AngleX = gama * 180.0 / 3.1415;
    imu_sensor.AngleY = alpha * 180.0 / 3.1415;
}
