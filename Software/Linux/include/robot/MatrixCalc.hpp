
#ifndef MATRIX_CALC_HPP
#define MATRIX_CALC_HPP

#include <stdint.h>
#include <iostream>


class MatrixCalc
{
public:
	MatrixCalc(){};
	~MatrixCalc(){};

	void MatrixMull(float m1[][4], float m2[][4], float ret[][4])
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ret[i][j] = m1[i][0] * m2[0][j] + m1[i][1] * m2[1][j] + m1[i][2] * m2[2][j] + m1[i][3] * m2[3][j];
			}
		}
	}

	void MatrixTransmit(const float m1[4][4], float m2[4][4])
	{
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				m2[i][j] = m1[i][j];
			}
		}
	}

	void MatrixInv(float in_m[][4], float out_m[][4])
	{
		float mid_m[3][3] = { { 1,0,0 },{ 0,1,0 },{ 0,0,1 } };

		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				out_m[i][j] = 0;
			}
		}
		out_m[3][3] = 1;   
		
		for (int i = 0; i < 3; i++)  
		{
			for (int j = 0; j < 3; j++)
			{
				mid_m[j][i] = in_m[i][j];
			}
		}	
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				out_m[i][j] = mid_m[i][j];
			}
		}	
		for (int i = 0; i < 3; i++)   
		{
			out_m[i][3] = (-1)*mid_m[i][0] * in_m[0][3] + (-1)*mid_m[i][1] * in_m[1][3] + (-1)*mid_m[i][2] * in_m[2][3];	
		}	
	} 
};
#endif


