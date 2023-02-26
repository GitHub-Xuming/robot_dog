#include "CubicInterpolation.h"

CubicInterpolation::CubicInterpolation(double period)
{
    m_interpolate_count = 0;
    m_number_of_interpolation = 0;
    m_interpolater_period = period;
}

void CubicInterpolation::CubicInterpolationParaClear()
{
	m_interpolate_count = 0;
	m_delta_t = 0;
	m_run_time = 0;
	m_a = 0;
	m_b = 0;
	m_c = 0;
	m_d = 0;
}

void CubicInterpolation::CubicInterpolationBegin()
{
	CubicInterpolationParaClear();
	m_start_time = m_cubic_start.time;
	m_delta_t = m_cubic_end.time - m_cubic_start.time;
	m_number_of_interpolation = m_delta_t / m_interpolater_period;

	m_a = (-2.0 * m_cubic_end.position + m_cubic_end.velocity * m_delta_t + m_cubic_start.velocity * m_delta_t + 2.0 * m_cubic_start.position) / (m_delta_t * m_delta_t * m_delta_t);
	m_b = ( 3.0 * m_cubic_end.position - m_cubic_end.velocity * m_delta_t - 2.0 * m_cubic_start.velocity * m_delta_t - 3.0 * m_cubic_start.position) / (m_delta_t * m_delta_t);
	m_c = m_cubic_start.velocity; 
	m_d = m_cubic_start.position;
}

bool CubicInterpolation::CubicInterpolationRuner(double &pos_out)
{
	m_run_time = m_start_time + m_interpolate_count * m_interpolater_period;

	m_p_cubic_term = m_a * (m_run_time - m_start_time) * (m_run_time - m_start_time) * (m_run_time - m_start_time);
	m_p_quadratic_term = m_b * (m_run_time - m_start_time) * (m_run_time - m_start_time);
	m_p_linear_term = m_c * (m_run_time - m_start_time);
	m_p_zero_order_term = m_d;

	// m_v_quadratic_term = 3.0 * m_a * (m_run_time - m_start_time) * (m_run_time - m_start_time);
	// m_v_linear_term = 2.0 * m_b * (m_run_time - m_start_time);
	// m_v_zero_order_term = m_c;

	pos_out = m_p_cubic_term + m_p_quadratic_term + m_p_linear_term + m_p_zero_order_term;
	//cubic_vel = m_v_quadratic_term + m_v_linear_term + m_v_zero_order_term;
	//vel_out = cubic_vel;

	m_interpolate_count++;
	if(m_interpolate_count > m_number_of_interpolation)
	{
		m_interpolate_count = 0;
		return false;
	}
	return true;
}