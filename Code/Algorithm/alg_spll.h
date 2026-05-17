#ifndef __ALG_SPLL_H
#define __ALG_SPLL_H

#include "alg_pid.h"

typedef struct
{
    float Ts;              // 采样周期，单位 s

    float theta;           // 当前估计相位，单位 rad
    float omega;           // 当前估计角速度，单位 rad/s
    float omega_nominal;   // 标称角速度，单位 rad/s

    float sin_theta;       // 内部正弦
    float cos_theta;       // 内部余弦

    float phase_error;     // 鉴相误差
    float phase_error_lpf; // 低通后的相位误差

    float lpf_alpha;       // 低通滤波系数，0~1

    PID_t pid;             // 使用你写的 PID 作为环路滤波器

} SPLL_t;

void SPLL_Init(SPLL_t *spll,
               float freq_hz,
               float sample_freq_hz,
               float Kp,
               float Ki,
               float Kd);

float SPLL_Update(SPLL_t *spll, float input);

float SPLL_GetPhase(SPLL_t *spll);
float SPLL_GetFreq(SPLL_t *spll);
float SPLL_GetSin(SPLL_t *spll);
float SPLL_GetCos(SPLL_t *spll);

#endif