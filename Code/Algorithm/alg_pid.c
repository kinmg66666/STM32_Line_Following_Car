#include "alg_pid.h"
#include "math.h"

static inline float Limit(float value, float max)
{
    if (value > max)
        return max;
    else if (value < -max)
        return -max;
    else
        return value;
}

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Previous_Error = 0.0f;
    pid->Out_i = 0.0f;
    pid->Out_p = 0.0f;
    pid->Out_d = 0.0f;
    pid->Integral = 0.0f;
    pid->Target_Value = 0.0f;
    pid->Integral_Separation = 99999.0f;
    pid->Integral_Max = 99999.0f;
    pid->Output_Max = 99999.0f;
    pid->Output = 0.0f;
}

void PID_SetLimit(PID_t *pid, float integral_max, float output_max)
{
    pid->Integral_Max = integral_max;
    pid->Output_Max = output_max;
}

void PID_SetIntegralSeparation(PID_t *pid, float separation)
{
    pid->Integral_Separation = separation;
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->Target_Value = target;
}

float PID_Compute(PID_t *pid, float current_value)
{
    float error = pid->Target_Value - current_value;

    

    if(fabsf(error) < pid->Integral_Separation)
    {
        pid->Integral += error;

        pid->Integral= Limit(pid->Integral, pid->Integral_Max);
        
    }
    else 
    {
        pid->Integral = 0;          
    }

    float derivative = error - pid->Previous_Error;
    pid->Previous_Error = error;

    pid->Out_p = pid->Kp * error;
    pid->Out_i = pid->Ki * pid->Integral;
    pid->Out_d = pid->Kd * derivative;

    float Output = pid->Out_p + pid->Out_i + pid->Out_d;
    Output = Limit(Output, pid->Output_Max);
    pid->Output = Output;

    return Output;
}


