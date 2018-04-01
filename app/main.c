#include <stdio.h>
#include "key.h"
#include "mpu6050.h"
#include "pwmRotor4.h"
#include "i2c.h"

/***************È«¾Ö±äÁ¿Çø****************/ 
extern float anx,any;
extern float Ki, Kp, Kd;    //PID²ÎÊý
extern float Angle, angle_dot,  output_date;    //mp6050Êý¾Ý
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
    IIC_GPIO_Configuration( IIC_GOIO_SDA , IIC_SDA , IIC_GPIO_SCL , IIC_SCL );
    MPU6050_Inital();

    /* PWM初始化 */
    pwmRotor4.initialize();
    
    while(1)
    {
        /* 
         * 每2ms采样一次
         * 读MPU6050，得到
         */
        // delay_ms(2);
        getAccX();
        getAccY();
        getAccZ();
        //printf("x:%d,y:%d,z:%d\n", x, y, z);
    }
}
