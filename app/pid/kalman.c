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
void complement_filterX(float angle_m_cfx,float gyro_m_cfx)
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

void Angle_Calcu(void)	
{
	float temp1,temp2;
	/****************************Y方向**************************/
		//加速度(角度)			
	JSDx  = getAccX();	  //读取X轴加速度
	JSDy  = getAccY();	  //读取Y轴加速度
	JSDz  = getAccZ();	  //读取Z轴加速度
	temp1=sqrt((JSDx*JSDx+JSDz*JSDz))/JSDy;
	JSDy=atan(temp1)/3.1415926*180;
	if(JSDy>0) Angle_B=JSDy-89.9;
	if(JSDy<0) Angle_B=JSDy+90.1;
  //陀螺仪(角速度)
	TLYx = getGyroX();	      //静止时角速度Y轴输出为-30左右
	TLYx = (TLYx)/16.384;         //去除零点偏移，计算角速度值,负号为方向处理 
////	Angle_gy = Angle_gy + TLYy*0.0056;  //角速度积分得到倾斜角度.	
	
	/****************************X方向**************************/
		//加速度(角度)
	JSDx  = getAccX();	  //读取X轴加速度
	JSDy  = getAccY();	  //读取Y轴加速度
	JSDz  = getAccZ();	  //读取Z轴加速度
	temp2=sqrt((JSDy*JSDy+JSDz*JSDz))/JSDx;
	JSDx=atan(temp2)/3.1415926*180;
	if(JSDx>0) Angle_A=JSDx-87.2;
	if(JSDx<0) Angle_A=JSDx+92.8;
  //陀螺仪(角速度)
	TLYy = getGyroY();	      //静止时角速度Y轴输出为-30左右
	TLYy = -(TLYy)/16.384;         //去除零点偏移，计算角速度值,负号为方向处理 
////	Angle_gy = Angle_gy + TLYy*0.0056;  //角速度积分得到倾斜角度.	
	
		//-------互补滤波-----------------------

	//补偿原理是取当前倾角和加速度获得倾角差值进行放大，然后与
    //陀螺仪角速度叠加后再积分，从而使倾角最跟踪为加速度获得的角度
	//0.5为放大倍数，可调节补偿度；0.005为系统周期5ms	
		
	Angle_X = Angle_X + (((Angle_A-Angle_X)*0.5 + TLYy)*0.0054);		//X方向
	Angle_Y =Angle_Y + (((Angle_B-Angle_Y)*0.5 + TLYx)*0.0054);	//Y方向

}

/******************************************************************
备注：通过陀螺仪积分闭环调节的值与加速度计的值一样
	角度值有陀螺仪来测定，原因加速度计测得的角度有许噪声（程序中需要改进），
	用陀螺仪测角度，需要用到波形显示来调节
*********************************************************************/





