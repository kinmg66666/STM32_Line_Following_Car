#include "alg_spll.h"
#include <math.h>

#define TWO_PI 6.2831853071795864769f

static inline float Wrap_0_2PI(float angle)
{
    while (angle >= TWO_PI)
        angle -= TWO_PI;

    while (angle < 0.0f)
        angle += TWO_PI;

    return angle;
}

void SPLL_Init(SPLL_t *spll,
               float freq_hz,
               float sample_freq_hz,
               float Kp,
               float Ki,
               float Kd)
{
    spll->Ts = 1.0f / sample_freq_hz;

    spll->theta = 0.0f;
    spll->omega_nominal = TWO_PI * freq_hz;
    spll->omega = spll->omega_nominal;

    spll->sin_theta = 0.0f;
    spll->cos_theta = 1.0f;

    spll->phase_error = 0.0f;
    spll->phase_error_lpf = 0.0f;

    /*
     * lpf_alpha 越大，响应越快，但抗噪声能力越弱
     * lpf_alpha 越小，响应越慢，但更平滑
     */
    spll->lpf_alpha = 0.05f;

    /*
     * PID 作为环路滤波器
     * 软件 PLL 通常先用 PI 就够了，所以 Kd 可以先设为 0
     */
    PID_Init(&spll->pid, Kp, Ki, Kd);

    /*
     * PID 目标是让相位误差为 0
     */
    PID_SetTarget(&spll->pid, 0.0f);

    /*
     * PID 输出是频率修正量，单位 rad/s
     * 例如这里限制最大修正范围为 ±2π*20 rad/s，也就是 ±20Hz
     */
    PID_SetLimit(&spll->pid,
                 1000.0f,
                 TWO_PI * 20.0f);

    /*
     * 暂时不启用积分分离
     */
    PID_SetIntegralSeparation(&spll->pid, 99999.0f);
}

float SPLL_Update(SPLL_t *spll, float input)
{
    /*
     * input 建议是归一化后的正弦信号，范围大约 -1 ~ 1
     *
     * 如果输入为：
     * input = sin(phi)
     *
     * 内部余弦为：
     * cos(theta)
     *
     * 则：
     * input * cos(theta)
     * = sin(phi) * cos(theta)
     * = 0.5 * [sin(phi + theta) + sin(phi - theta)]
     *
     * 经过低通滤波后，高频项 sin(phi + theta) 被滤掉，
     * 剩下近似：
     * 0.5 * sin(phi - theta)
     *
     * 这就是相位误差。
     */
    spll->phase_error = input * spll->cos_theta;

    /*
     * 一阶低通滤波
     */
    spll->phase_error_lpf += spll->lpf_alpha *
                             (spll->phase_error - spll->phase_error_lpf);

    /*
     * 你的 PID 里面：
     * error = Target_Value - current_value
     *
     * 我们希望 phase_error_lpf 为正时，PID 输出也为正，
     * 所以这里传入 -phase_error_lpf。
     */
    float omega_correction = PID_Compute(&spll->pid, -spll->phase_error_lpf);

    /*
     * 修正内部角速度
     */
    spll->omega = spll->omega_nominal + omega_correction;

    /*
     * 积分得到相位
     */
    spll->theta += spll->omega * spll->Ts;
    spll->theta = Wrap_0_2PI(spll->theta);

    /*
     * 生成内部正交信号
     */
    spll->sin_theta = sinf(spll->theta);
    spll->cos_theta = cosf(spll->theta);

    /*
     * 返回锁相后的正弦信号
     */
    return spll->sin_theta;
}

float SPLL_GetPhase(SPLL_t *spll)
{
    return spll->theta;
}

float SPLL_GetFreq(SPLL_t *spll)
{
    return spll->omega / TWO_PI;
}

float SPLL_GetSin(SPLL_t *spll)
{
    return spll->sin_theta;
}

float SPLL_GetCos(SPLL_t *spll)
{
    return spll->cos_theta;
}