#ifndef __KALMAN_H
#define __KALMAN_H

/*卡尔曼滤波器类*/
typedef struct
{
    double x_last;
    double p_last;
    double ProcessNiose_Q;
    double ProcessNiose_R;
}KalmanFilter;



void KalmanFilterInit(KalmanFilter *obj, double first_val, double Q, double R);
double KalmanFilterFun(KalmanFilter *obj, double data);

#endif  

