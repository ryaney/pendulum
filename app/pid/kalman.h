#ifndef __KaErMan_H_
#define __KaErMan_H_

#include "stm32f10x.h"

extern float angle_dot; 		//外部需要引用的变量

extern float Angle_A,Angle_B;	//X.Y方向加速度暂存
extern float Angle_X,Angle_Y;	//最终倾角X.Yextern float Gyro_y;
extern float Angle_gy;         //前一时刻的积分角度

extern float JSDx;		
extern float JSDy;
extern float JSDz;
extern float TLYx;
extern float TLYy;
extern float TLYz;

extern void Angle_Calcu(void);
 
#endif

