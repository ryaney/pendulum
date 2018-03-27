#include <stdio.h>
#include <pendulum.h>


/***************È«¾Ö±äÁ¿Çø****************/ 
extern float anx,any;
extern float Ki, Kp, Kd;																																	//PID²ÎÊý
extern float Angle, angle_dot,  output_date;   																													//mp6050Êý¾Ý
/*****************************************/
void main(void)
{
	/* 时钟初始化 */
	/* GPIO初始化 */
	/* I2C、MPU6050初始化 */
	/* PWM初始化 */
	delay_init();
	delay_ms(100);	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 
   	Key_GPIO_Config();
	TIM5_PWM_Init(999,24);
	TIM3_Int_Init(4999, 71);  
	IIC_GPIO_Configuration( IIC_GOIO_SDA , IIC_SDA , IIC_GPIO_SCL , IIC_SCL );
	delay_ms(10);
	MPU6050_Inital();
    delay_ms(10);	
	while(1)
	{
	    delay_ms(2);
	}
}
