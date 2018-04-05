#include <stdio.h>
#include "key.h"
#include "mpu6050.h"
#include "pwmRotor4.h"
#include "i2c.h"
#include "pid.h"
#include "timer.h"

/*------------------------------------------
                全局变量                
------------------------------------------*/ 
extern uint8_t CurMode; 

/***************È«¾Ö±äÁ¿Çø****************/ 
extern float anx,any;
extern float Ki, Kp, Kd;    //PID²ÎÊý
extern float Angle, angle_dot,  output_date;    //mp6050Êý¾Ý
/*****************************************/
int main(void)
{
//    short x,y,z;
    int key = 0, count = 0;
    
    /* 时钟初始化 */
    SystemInit();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    /* GPIO初始化 */
    Key_GPIO_Config();
    
    /* I2C、MPU6050初始化 */
    I2C_Config();
    imu_sensor.initialize();

    /* Timer初始化 */
    TIM3_Config(5000-1,71);   /* TIM5 5ms Inturrupt 200Hz*/

    /* PWM初始化 */
    PWM_Init();

    Key_GPIO_Config();
    
    while(1)
    {
        // key = Key_Scan();
        key = 1;
        count ++;
        if (count < 1000 && key == 1)
        {
            CurMode = 1;
        } else if (count >= 1000 && key == 1) {
            CurMode = 1;
        }
    }
}
