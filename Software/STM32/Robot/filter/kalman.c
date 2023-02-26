#include "kalman.h"

/**
  * @brief   
  * @param  
  * @param  
  */
void KalmanFilterInit(KalmanFilter *obj, double first_val, double Q, double R)
{
    obj->x_last = first_val;
    obj->ProcessNiose_Q = Q;
    obj->ProcessNiose_R = R;
}

/**
  * @brief   
  * @param  
  * @param  
  */
double KalmanFilterFun(KalmanFilter *obj, double data)
{
    double x_mid = obj->x_last;
    double x_now = 0.0;
    double p_mid = 0.0;
    double p_now = 0.0;
    double kg = 0.0;

    x_mid = obj->x_last;
    p_mid = obj->p_last + obj->ProcessNiose_Q; 

    kg = p_mid / (p_mid + obj->ProcessNiose_R);
    x_now = x_mid + kg * (data - x_mid);
    p_now = (1 - kg) * p_mid; 
    obj->p_last = p_now;
    obj->x_last = x_now;

    return x_now;
}


