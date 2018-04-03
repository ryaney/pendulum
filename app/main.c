#include <stdio.h>
#include "key.h"
#include "mpu6050.h"
#include "pwmRotor4.h"
#include "i2c.h"
#include "pid.h"

/***************È«¾Ö±äÁ¿Çø****************/ 
extern float anx,any;
extern float Ki, Kp, Kd;    //PID²ÎÊý
extern float Angle, angle_dot,  output_date;    //mp6050Êý¾Ý
static float target = 10;
/*****************************************/
int main(void)
{
//    short x,y,z;
    
    /* 时钟初始化 */
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* GPIO初始化 */
    Key_GPIO_Config();
    
    /* I2C、MPU6050初始化 */
    I2C_Config();
    imu_sensor.initialize();

    /* PWM初始化 */
    pwmRotor4.initialize();
    
    while(1)
    {
        /* 
         * 每2ms采样一次
         * 读MPU6050，得到三轴
         * PID反馈控制
         */
        imu_sensor.read_imu();
        imu_sensor.angle_cal();
        pid(target);
//        delay_ms(100);
    }
}
