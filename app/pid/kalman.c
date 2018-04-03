#include "stm32f10x.h"
#include <math.h>
#include "kalman.h"
#include "mpu6050.h"

float Angle_gy;         //ǰһʱ�̵Ļ��ֽǶ�
float Angle_A,Angle_B;  //X.Y������ٶ��ݴ�
float Angle_X,Angle_Y;  //�������X.Y

/* ���ٶȼƵķ���ֵ�ݴ���� */
float JSDx = 0;
float JSDy = 0;
float JSDz = 0;
/* �����ǵķ���ֵ�ݴ���� */
float TLYx = 0;
float TLYy = 0;
float TLYz = 0;


//�����˲�
//-------------------------------------------------------
//-------------------------------------------------------
float bias_cfx;
float angle_dotx;   //�ⲿ��Ҫ���õı���
const float dtx=0.0048;
//-------------------------------------------------------
void complement_filter(float angle_m_cfx,float gyro_m_cfx)
{
    bias_cfx*=0.0001;          //��������Ʈ��ͨ�˲���500�ξ�ֵ��0.998 
    bias_cfx+=gyro_m_cfx*0.009;		   //0.002
    angle_dotx=gyro_m_cfx-bias_cfx;		   
    Angle_X=angle_m_cfx*0.02 + (Angle_X+angle_dotx*dtx)*0.98;	
    //���ٶȵ�ͨ�˲���20�ξ�ֵ����100��ÿ����㣬��ͨ5Hz��0.90 0.05
}
//-------------------------------------------------------
//-------------------------------------------------------
float bias_cfy;
float angle_doty;   //�ⲿ��Ҫ���õı���
const float dty=0.0048;
//-------------------------------------------------------
void complement_filterY(float angle_m_cfy,float gyro_m_cfy)
{
	bias_cfy*=0.0001;			       //��������Ʈ��ͨ�˲���500�ξ�ֵ��0.998 
	bias_cfy+=gyro_m_cfy*0.009;		   //0.002
	angle_doty=gyro_m_cfy-bias_cfy;		   
	Angle_Y=angle_m_cfy*0.02 + (Angle_Y+angle_doty*dty)*0.98;	
	//���ٶȵ�ͨ�˲���20�ξ�ֵ����100��ÿ����㣬��ͨ5Hz��0.90 0.05
}
