#include <stm32f10x.h>
#include "pid.h"
#include "pwmRotor4.h"
#include "mpu6050.h"
#include "stdlib.h"
#include "stdio.h"
#include "math.h"
#include "kalman.h"

/*------------------------------------------
              电机物理限制                
------------------------------------------*/
#define STROKE_LIMIT     (90000)   //物理极限行程<0-80000>
#define SPEED_LIMIT      (65535)   //物理极限速度<65535>
#define POWER_LIMIT      (3000)    //物理极限功率<0-3000>

#define STROKE_MAX       (80000)   //软件极限行程<0-80000>
#define SPEED_MAX        (300)     //软件极限速度<0-300>
#define POWER_MAX        (2400)    //软件极限功率<0-3000>

#define TIMEOUT_CNT      (5000)    //归零超时计数器ms<0-10000>
#define ZERO_POINT       (10000)   //设置参考零点为<0-80000>
#define LENGTH           (45.0f)   //杆长

/*------------------------------------------
                全局变量                
------------------------------------------*/
M1TypeDef M1;
M2TypeDef M2;

PIDTypdDef M1PID;
PIDTypdDef M2PID;
extern AHRS_EulerAngleTypeDef EulerAngle;

float R = 10.0;                      //半径设置(cm)
float angle = 40.0;                  //摆动角度设置(°)
uint8_t RoundDir = 0;                //正反转控制


/*------------------------------------------
 函数功能:初始化M1PID结构体参数
 函数说明:          
------------------------------------------*/
void PID_M1_Init(void)
{
    M1PID.LastError  = 0;           //Error[-1]
    M1PID.PrevError  = 0;           //Error[-2]
    M1PID.Proportion = 0;           //比例常数 Proportional Const
    M1PID.Integral   = 0;           //积分常数 Integral Const
    M1PID.Derivative = 0;           //微分常数 Derivative Const
    M1PID.SetPoint   = 0;
    M1PID.SumError   = 0;
}
/*------------------------------------------
 函数功能:初始化M2PID结构体参数
 函数说明:          
------------------------------------------*/
void PID_M2_Init(void)
{
    M2PID.LastError  = 0;           //Error[-1]
    M2PID.PrevError  = 0;           //Error[-2]
    M2PID.Proportion = 0;           //比例常数 Proportional Const
    M2PID.Integral   = 0;           //积分常数 Integral Const
    M2PID.Derivative = 0;           //微分常数 Derivative Const
    M2PID.SetPoint   = 0;
    M2PID.SumError   = 0;
}
/*------------------------------------------
 函数功能:设置M1PID期望值
 函数说明:          
------------------------------------------*/
void PID_M1_SetPoint(float setpoint)
{   
    M1PID.SetPoint = setpoint;  
}
/*------------------------------------------
 函数功能:设置M2期望值
 函数说明:          
------------------------------------------*/
void PID_M2_SetPoint(float setpoint)
{   
    M2PID.SetPoint = setpoint;  
}
/*------------------------------------------
 函数功能:设置M1PID比例系数
 函数说明:浮点型           
------------------------------------------*/
void PID_M1_SetKp(float dKpp)
{   
    M1PID.Proportion = dKpp;    
}
/*------------------------------------------
 函数功能:设置M2比例系数
 函数说明:浮点型           
------------------------------------------*/
void PID_M2_SetKp(float dKpp)
{   
    M2PID.Proportion = dKpp;    
}
/*------------------------------------------
 函数功能:设置M1PID积分系数
 函数说明:浮点型           
------------------------------------------*/
void PID_M1_SetKi(float dKii)
{   
    M1PID.Integral = dKii;  
}
/*------------------------------------------
 函数功能:设置M2积分系数
 函数说明:浮点型           
------------------------------------------*/
void PID_M2_SetKi(float dKii)
{   
    M2PID.Integral = dKii;  
}
/*------------------------------------------
 函数功能:设置M1PID微分系数
 函数说明:浮点型           
------------------------------------------*/
void PID_M1_SetKd(float dKdd)
{   
    M1PID.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:设置M2微分系数
 函数说明:浮点型           
------------------------------------------*/
void PID_M2_SetKd(float dKdd)
{   
    M2PID.Derivative = dKdd;
}
/*------------------------------------------
 函数功能:电机1位置式PID计算
 函数说明:      
------------------------------------------*/
int32_t PID_M1_PosLocCalc(float NextPoint)
{
    register float  iError,dError;

    iError = M1PID.SetPoint - NextPoint;        // 偏差
    M1PID.SumError += iError;                   // 积分
    if(M1PID.SumError > 2300.0)                 //积分限幅2300
        M1PID.SumError = 2300.0;
    else if(M1PID.SumError < -2300.0)
        M1PID.SumError = -2300.0;   
    dError = iError - M1PID.LastError;          // 当前微分
    M1PID.LastError = iError;
    
    return(int32_t)(  M1PID.Proportion * iError             // 比例项
                    + M1PID.Integral   * M1PID.SumError         // 积分项
                    + M1PID.Derivative * dError);
}

/*------------------------------------------
 函数功能:电机2位置式PID计算
 函数说明:          
------------------------------------------*/
int32_t PID_M2_PosLocCalc(float NextPoint)
{
    register float  iError,dError;

    iError = M2PID.SetPoint - NextPoint;        // 偏差
    M2PID.SumError += iError;
    if(M2PID.SumError > 2300.0)                 //积分限幅
        M2PID.SumError = 2300.0;
    else if(M2PID.SumError < -2300.0)
        M2PID.SumError = -2300.0;
    dError = iError - M2PID.LastError;          // 当前微分
    M2PID.LastError = iError;
    
    return(int32_t)(  M2PID.Proportion * iError             // 比例项
                    + M2PID.Integral   * M2PID.SumError         // 积分项
                    + M2PID.Derivative * dError);
}




/*------------------------------------------
 函数功能:控制器软件复位
 函数说明:强制复位          
------------------------------------------*/
void MCU_Reset(void) 
{
    __set_FAULTMASK(1);   // 关闭所有中断
    NVIC_SystemReset();   // 复位
}
/*------------------------------------------
 函数功能:初始化M1结构体参数
 函数说明:          
------------------------------------------*/
void M1TypeDef_Init(void)
{
    M1.CurPos    = 0.0;
    M1.PrevPos   = 0.0;
    M1.CurAcc    = 0.0;
    M1.PrevSpeed = 0.0;
    M1.Offset    = 0.1;   //允许偏差量
    M1.CurSpeed  = 0.0;  //当前速度矢量
    M1.PWM = 0;          //PWM
}
/*------------------------------------------
 函数功能:初始化M2结构体参数
 函数说明:          
------------------------------------------*/
void M2TypeDef_Init(void)
{
    M2.CurPos    = 0.0;
    M2.PrevPos   = 0.0;
    M2.CurAcc    = 0.0;
    M2.PrevSpeed = 0.0;
    M2.Offset    = 0.1;   //允许偏差量
    M2.CurSpeed  = 0.0;  //当前速度矢量
    M2.PWM = 0;          //PWM      
}
/*------------------------------------------
 函数功能:
------------------------------------------*/
void Mode_0(void)
{
    
        
}
/*------------------------------------------
 函数功能:第1问PID计算
 函数说明:
------------------------------------------*/
void Mode_1(void)
{
    const float priod = 1410.0;  //单摆周期(毫秒)
    static uint32_t MoveTimeCnt = 0;
    float set_y = 0.0;
    float A = 0.0;
    float Normalization = 0.0;
    float Omega = 0.0;
                
    MoveTimeCnt += 5;                            //每5ms运算1次
    Normalization = (float)MoveTimeCnt / priod;  //对单摆周期归一化
    Omega = 2.0 * 3.14159 * Normalization;       //对2π进行归一化处理
    A = atan((R / LENGTH)) * 57.2958f;           //根据摆幅求出角度A,88为摆杆距离地面长度cm
    set_y = A * sin(Omega);                        //计算出当前摆角  
        
    PID_M1_SetPoint(0);         //X方向PID定位目标值0
    PID_M1_SetKp(60);   
    PID_M1_SetKi(0.79);  
    PID_M1_SetKd(800);
    
    PID_M2_SetPoint(set_y);     //Y方向PID跟踪目标值sin
    PID_M2_SetKp(60);    
    PID_M2_SetKi(0.79);     
    PID_M2_SetKd(800);   
    
    M1.PWM = PID_M1_PosLocCalc(M1.CurPos);  //Pitch
    M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
    
    if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
    if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;    
    
    if(M2.PWM > POWER_MAX)  M2.PWM = POWER_MAX;
    if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;        
    
    MotorMove(M1.PWM,M2.PWM);
}
/*------------------------------------------
 函数功能:第2问PID计算
 函数说明:
------------------------------------------*/
void Mode_2(void)
{
    const float priod = 1410.0;  //单摆周期(毫秒)
    static uint32_t MoveTimeCnt = 0;
    float set_x = 0.0;
    float A = 0.0;
    float Normalization = 0.0;
    float Omega = 0.0;              
    MoveTimeCnt += 5;                            //每5ms运算1次
    Normalization = (float)MoveTimeCnt / priod;  //对单摆周期归一化
    Omega = 2.0*3.14159*Normalization;           //对2π进行归一化处理
    A = atan((R/88.0f))*57.2958f;//根据摆幅求出角度A,88为摆杆离地高度
    set_x = A*sin(Omega);                        //计算出当前摆角          
    PID_M1_SetPoint(set_x); //X方向PID跟踪目标值sin
    PID_M1_SetKp(60);   
    PID_M1_SetKi(0.79);  
    PID_M1_SetKd(800);  
    PID_M2_SetPoint(0);     //Y方向PID定位目标值0
    PID_M2_SetKp(60);    
    PID_M2_SetKi(0.79);     
    PID_M2_SetKd(800);      
    M1.PWM = PID_M1_PosLocCalc(M1.CurPos);  //X方向PID计算
    M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Y方向PID计算  
    if(M1.PWM > POWER_MAX) M1.PWM  =  POWER_MAX;//输出限幅
    if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;    
    if(M2.PWM > POWER_MAX) M2.PWM  =  POWER_MAX;
    if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;            
    MotorMove(M1.PWM,M2.PWM);//电机输出
}
/*------------------------------------------
 函数功能:第3问PID计算
 函数说明:
------------------------------------------*/ 
void Mode_3(void)
{
    const float priod = 1410.0;  //单摆周期(毫秒)
                 //相位补偿 0, 10   20   30   40   50   60   70   80   90   100  110  120  130  140  150  160  170 180
    const float Phase[19]= {0,-0.1,-0.05,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.05,0.05,0.05,0.07,0};
    static uint32_t MoveTimeCnt = 0;
    float set_x = 0.0;
    float set_y = 0.0;
    float Ax = 0.0;
    float Ay = 0.0;
    float A = 0.0;
    uint32_t pOffset = 0;
    float Normalization = 0.0;
    float Omega = 0.0;
    
    pOffset = (uint32_t)(angle/10.0f);           //相位补偿数组下标
    MoveTimeCnt += 5;                            //每5ms运算1次
    Normalization = (float)MoveTimeCnt / priod;  //对单摆周期归一化
    Omega = 2.0*3.14159*Normalization;           //对2π进行归一化处理
    A = atan((R/88.0f))*57.2958f;//根据摆幅求出角度A,88为摆杆离地高度                                          
    Ax = A*cos(angle*0.017453);  //计算出X方向摆幅分量0.017453为弧度转换
    Ay = A*sin(angle*0.017453);  //计算出Y方向摆幅分量
    set_x = Ax*sin(Omega);       //计算出X方向当前摆角
    set_y = Ay*sin(Omega+Phase[pOffset]); //计算出Y方向当前摆角
        
    PID_M1_SetPoint(set_x); //X方向PID跟踪目标值sin
    PID_M1_SetKp(60);   
    PID_M1_SetKi(0.79);  
    PID_M1_SetKd(800);

    PID_M2_SetPoint(set_y); //Y方向PID跟踪目标值sin
    PID_M2_SetKp(60);    
    PID_M2_SetKi(0.79);     
    PID_M2_SetKd(800);   
    
    M1.PWM = PID_M1_PosLocCalc(M1.CurPos);  //Pitch
    M2.PWM = PID_M2_PosLocCalc(M2.CurPos);  //Roll
    
    if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
    if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
                
    if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
    if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;        

    MotorMove(M1.PWM,M2.PWM);   
}
/*------------------------------------------
 函数功能:第4问PID计算
 函数说明:
------------------------------------------*/ 
void Mode_4(void)
{   
    if(abs(M1.CurPos)<45.0 && abs(M2.CurPos)<45.0)  //小于45度才进行制动
    {       
        PID_M1_SetPoint(0);   //X方向PID定位目标值0
        PID_M1_SetKp(85);       
        PID_M1_SetKi(0);     
        PID_M1_SetKd(2000);

        PID_M2_SetPoint(0);   //Y方向PID定位目标值0
        PID_M2_SetKp(85);       
        PID_M2_SetKi(0);    
        PID_M2_SetKd(2000);
            
        M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Pitch
        M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
        
        if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
        if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;

        if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
        if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;
    }
    else    
    {
        M1.PWM = 0;
        M2.PWM = 0; 
    }
    
    MotorMove(M1.PWM,M2.PWM);
}
/*------------------------------------------
 函数功能:第5问PID计算
 函数说明:
------------------------------------------*/
void Mode_5(void)
{
    const float priod = 1410.0;  //单摆周期(毫秒)
    static uint32_t MoveTimeCnt = 0;
    float set_x = 0.0;
    float set_y = 0.0;
    float A = 0.0;
    float phase = 0.0;
    float Normalization = 0.0;
    float Omega = 0.0;
    
    MoveTimeCnt += 5;                            //每5ms运算1次
    Normalization = (float)MoveTimeCnt / priod;  //对单摆周期归一化
    Omega = 2.0*3.14159*Normalization;           //对2π进行归一化处理               
    A = atan((R/88.0f))*57.2958f;    //根据半径求出对应的振幅A
    
    if(RoundDir == 0)             
        phase = 3.141592/2.0;        //逆时针旋转相位差90° 
    else if(RoundDir == 1)  
        phase = (3.0*3.141592)/2.0;  //顺时针旋转相位差270°
    
    set_x = A*sin(Omega);            //计算出X方向当前摆角
    set_y = A*sin(Omega+phase);      //计算出Y方向当前摆角
     
    PID_M1_SetPoint(set_x); //X方向PID跟踪目标值sin
    PID_M1_SetKp(60);   
    PID_M1_SetKi(0.79);  
    PID_M1_SetKd(800);

    PID_M2_SetPoint(set_y); //Y方向PID跟踪目标值cos
    PID_M2_SetKp(60);    
    PID_M2_SetKi(0.79);     
    PID_M2_SetKd(800);       
    
    M1.PWM = PID_M1_PosLocCalc(M1.CurPos); //Pitch
    M2.PWM = PID_M2_PosLocCalc(M2.CurPos); //Roll
    
    if(M1.PWM > POWER_MAX)  M1.PWM =  POWER_MAX;
    if(M1.PWM < -POWER_MAX) M1.PWM = -POWER_MAX;
                
    if(M2.PWM > POWER_MAX)  M2.PWM =  POWER_MAX;
    if(M2.PWM < -POWER_MAX) M2.PWM = -POWER_MAX;        

    MotorMove(M1.PWM,M2.PWM);
    
}
/*------------------------------------------
 函数功能:第6问PID计算
 函数说明:
------------------------------------------*/
void Mode_6(void)
{

}
/*------------------------------------------
 函数功能:电机底层驱动函数
 函数说明:
------------------------------------------*/
void MotorMove(int32_t pwm1,int32_t pwm2)
{
    if(pwm1 > 0)
    {
        PWM_M2_Forward(pwm1);
        PWM_M4_Backward(pwm1);
    }
    else if(pwm1 < 0)
    {
        PWM_M2_Backward(abs(pwm1));
        PWM_M4_Forward(abs(pwm1));  
    }

    if(pwm2 > 0)
    {
        PWM_M1_Forward(pwm2);
        PWM_M3_Backward(pwm2);
    }
    else if(pwm2 < 0)
    {
        PWM_M1_Backward(abs(pwm2));
        PWM_M3_Forward(abs(pwm2));  
    }   
}
