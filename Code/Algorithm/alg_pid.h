#ifndef ALG_PID_H
#define ALG_PID_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Target_Value;

    float Integral;
    float Previous_Error;

    float Integral_Separation;
    float Integral_Max;
    float Output_Max;


    float Out_p;
    float Out_i;
    float Out_d;

    float Output;
    

} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd);
void PID_SetTarget(PID_t *pid, float target);
float PID_Compute(PID_t *pid, float current_value);
void PID_SetLimit(PID_t *pid, float integral_max, float output_max);
void PID_SetIntegralSeparation(PID_t *pid, float separation);

#endif
