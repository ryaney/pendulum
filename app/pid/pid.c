#include "pid.h"
#include "pwmRotor4.h"

extern float Angle_X;
float Kp = 160, Ki = 1.5, Kd = 60,  output_date;
void Dianji_PID(float target) //qiwang期望为转到多少度
{
    static float err, last_err = 0, last_last_err, jifeng;
    static  int flag;
    err = target - Angle_X;
    output_date = output_date + (err - last_err) * Kp  + (err - 2*last_err + last_last_err)* Kd;
    last_last_err = last_err;
    last_err = err;
    if(err > -2 || err < 2)
    {
        if(err > 0 && flag == 1)
        {
            jifeng += err;
        }
        else
        {
            if(flag == 1)
            {
                jifeng = 0;
            }
            flag = 0;
        }
        
        if(err > 0 && flag == 0)
        {
            jifeng += err;
        }
        else
        {
            if(flag == 0)
            {
                jifeng = 0;
            }
            flag = 1;	
        }
    }
    //角度
    if(output_date > 0)
    {
        pwmRotor4.setMotor1F(70 + output_date + jifeng * Ki);
        pwmRotor4.setMotor3F(70 + output_date + jifeng * Ki);
        pwmRotor4.setMotor2F(0);
        pwmRotor4.setMotor4F(0);		
    }
    else
    {
        pwmRotor4.setMotor2F(70 + output_date + jifeng * Ki);
        pwmRotor4.setMotor4F(70 + output_date + jifeng * Ki);
        pwmRotor4.setMotor1F(0);
        pwmRotor4.setMotor3F(0);		
    }
}
