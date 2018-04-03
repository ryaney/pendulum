#include "stm32f10x.h"
#include <math.h>
#include "kalman.h"
#include "mpu6050.h"

float Angle_gy;         //前一时刻的积分角度
float Angle_A,Angle_B;  //X.Y方向加速度暂存
float Angle_X,Angle_Y;  //最终倾角X.Y

/* 加速度计的返回值暂存变量 */
float JSDx = 0;
float JSDy = 0;
float JSDz = 0;
/* 陀螺仪的返回值暂存变量 */
float TLYx = 0;
float TLYy = 0;
float TLYz = 0;


//互补滤波
//-------------------------------------------------------
//-------------------------------------------------------
float bias_cfx;
float angle_dotx;   //外部需要引用的变量
const float dtx=0.0048;
//-------------------------------------------------------
void complement_filter(float angle_m_cfx,float gyro_m_cfx)
{
    bias_cfx*=0.0001;          //陀螺仪零飘低通滤波；500次均值；0.998 
    bias_cfx+=gyro_m_cfx*0.009;		   //0.002
    angle_dotx=gyro_m_cfx-bias_cfx;		   
    Angle_X=angle_m_cfx*0.02 + (Angle_X+angle_dotx*dtx)*0.98;	
    //加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；0.90 0.05
}
//-------------------------------------------------------
//-------------------------------------------------------
float bias_cfy;
float angle_doty;   //外部需要引用的变量
const float dty=0.0048;
//-------------------------------------------------------
void complement_filterY(float angle_m_cfy,float gyro_m_cfy)
{
	bias_cfy*=0.0001;			       //陀螺仪零飘低通滤波；500次均值；0.998 
	bias_cfy+=gyro_m_cfy*0.009;		   //0.002
	angle_doty=gyro_m_cfy-bias_cfy;		   
	Angle_Y=angle_m_cfy*0.02 + (Angle_Y+angle_doty*dty)*0.98;	
	//加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；0.90 0.05
}
