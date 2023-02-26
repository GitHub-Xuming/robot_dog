#pragma once
#include <stdio.h>
#include <iostream>

class Filter
{
public:
	Filter();
	~Filter(){};
	void KalmanFilterInit(float first_val, float Q, float R);
	float KalmanFilterFun(float input);
private:
    float x_last;
    float p_last;
    float ProcessNiose_Q;
    float ProcessNiose_R;
};