#ifndef __PID__H__
#define __PID__H__

#include "mpu6050.h"

extern void pid(float target);

/*------------------------------------------
                电机结构体               
------------------------------------------*/
typedef struct
{
    float Offset;     //允许偏差量
    float CurPos;
    float PrevPos;
    float CurAcc;
    float PrevSpeed;

    volatile float SetXPos;   //设定位置
    volatile float SetYPos;   //设定位置
    volatile float SetSpeed;  //设定速度
    
    volatile float CurXPos;   //当前位置
    volatile float CurYPos;   //当前位置
    volatile float CurSpeed;  //当前速度矢量

    volatile int32_t  PWM;        //PWM
    volatile uint8_t  ShootFlag;
    volatile uint8_t  AdjustFlag;
    volatile uint8_t  ErrFlag;

    volatile uint32_t SetMaxPos;      //软件设定最大位置
    volatile uint32_t SetMaxPower;    //软件设定最大力量
    volatile int32_t  SetMaxSpeed;    //软件设定最大速度
        
}M1TypeDef,M2TypeDef;

/*------------------------------------------
                PID结构体              
------------------------------------------*/
typedef struct
{
    float  SetPoint;    //  设定目标 Desired Value 
    double  SumError;       //  误差累计 
        
    float  Proportion;      //  比例常数 Proportional Const 
    float  Integral;        //  积分常数 Integral Const
    float  Derivative;      //  微分常数 Derivative Const

    float LastError;     //  Error[-1]
    float PrevError;     //  Error[-2]

}PIDTypdDef;

void MCU_Reset(void);
void M1TypeDef_Init(void);
void M2TypeDef_Init(void);
void Mode_0(void);
void Mode_1(void);
void Mode_2(void);
void Mode_3(void);
void Mode_4(void);
void Mode_5(void);
void Mode_6(void);
void MotorMove(int32_t pwm1,int32_t pwm2);


#endif

