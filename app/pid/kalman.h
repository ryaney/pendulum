#ifndef __KaErMan_H_
#define __KaErMan_H_

#include "stm32f10x.h"

extern float angle_dot; 		//�ⲿ��Ҫ���õı���

extern float Angle_A,Angle_B;	//X.Y������ٶ��ݴ�
extern float Angle_X,Angle_Y;	//�������X.Yextern float Gyro_y;
extern float Angle_gy;         //ǰһʱ�̵Ļ��ֽǶ�

extern float JSDx;		
extern float JSDy;
extern float JSDz;
extern float TLYx;
extern float TLYy;
extern float TLYz;

extern void Angle_Calcu(void);
 
#endif

