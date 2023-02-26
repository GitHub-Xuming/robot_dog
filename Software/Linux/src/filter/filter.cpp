#include <stdio.h>
#include <filter.h>

Filter::Filter()
{}

void Filter::KalmanFilterInit(float first_val, float Q, float R)
{
    x_last = first_val;
    ProcessNiose_Q = Q;
    ProcessNiose_R = R;
}

float Filter::KalmanFilterFun(float input)
{
    float x_mid = x_last;
    float x_now = 0;
    float p_mid = 0;
    float p_now = 0;
    float kg = 0;

    x_mid = x_last;
    p_mid = p_last + ProcessNiose_Q;

    kg = p_mid / (p_mid + ProcessNiose_R);
    x_now = x_mid + kg * (input - x_mid);
    p_now = (1 - kg) * p_mid; 
    p_last = p_now;
    x_last = x_now;

    return x_now;
}