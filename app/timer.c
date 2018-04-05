#include "stm32f10x.h"
#include "mpu6050.h"
#include "pwmRotor4.h"
#include "pid.h"
#include "kalman.h"
#include "timer.h"

/*------------------------------------------
                全局变量                
------------------------------------------*/ 
uint8_t CurMode = 0;

extern M1TypeDef  M1;
extern M2TypeDef  M2;

extern PIDTypdDef M1PID;
extern PIDTypdDef M2PID;

extern AHRS_EulerAngleTypeDef EulerAngle;

extern uint8_t Item;

/*-----------------------------------------------
 函数功能: TIM5定时器为PID采样计算提供稳定中断
 函数参数: ARR寄存器值0-65535,预分频值0-65535
 参 考 值: TIM5_Config(999,71)
           计数频率1MHz,中断频率1000Hz
           计数器每1us加1,中断每1ms产生一次                   
-----------------------------------------------*/
void TIM3_Config(unsigned short int Period,unsigned short int Prescaler)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    
    
    TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;            //时钟预分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数
    TIM_TimeBaseStructure.TIM_Period = Period;                  //自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频1
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM3,TIM_FLAG_Update);
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);                    
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   //抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          //响应优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3, ENABLE);  //⑤使能 TIM3  
    
}

/*-----------------------------------------------
 函数功能:TIM5中断服务程序
 函数说明:每5ms进入一次中断,采样率200Hz
 实测运行时间: 3.93ms
-----------------------------------------------*/
#define H (0.88f)  //万向节距地面的高度(米)
void TIM3_IRQHandler(void)
{
    /* 
     * 每2ms采样一次
     * 读MPU6050，得到三轴
     * PID反馈控制
     */
    float pitch_temp = 0.0;
    float roll_temp = 0.0;
        
    //GPIOE->BSRR = GPIO_Pin_3;
    if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
    {
        imu_sensor.read_imu();

        pitch_temp = atan(imu_sensor.Acc.TrueX / imu_sensor.Acc.TrueZ) * 57.3 - 0.4;   //计算Pitch角度 0.4为静态偏差角
        roll_temp  = atan(imu_sensor.Acc.TrueY / imu_sensor.Acc.TrueZ) * 57.3 - 0.3;   //计算Roll角度  0.3为静态偏差角
        
        // EulerAngle.Pitch = Kalman_Filter1(pitch_temp, imu_sensor.Gyr.TrueY);       //卡尔曼滤波器
        // EulerAngle.Roll  = Kalman_Filter2(roll_temp, -imu_sensor.Gyr.TrueX);       //卡尔曼滤波器
        
        M1.CurPos = pitch_temp; 
        M2.CurPos = roll_temp;                           
        
        //计算速度
        M1.CurSpeed = M1.CurPos - M1.PrevPos;
        M1.PrevPos = M1.CurPos;             
        
        M2.CurSpeed = M2.CurPos - M2.PrevPos;
        M2.PrevPos = M2.CurPos; 

        switch(CurMode) //根据题目选择函数
        {   
            case 1: Mode_1(); break;
            case 2: Mode_2(); break;
            case 3: Mode_3(); break;
            case 4: Mode_4(); break;
            case 5: Mode_5(); break;
            case 6: Mode_6(); break;
            default:break;
        }
                
        TIM_ClearITPendingBit(TIM3,TIM_IT_Update);      
    }
    //GPIOE->BRR = GPIO_Pin_3;                        
}
