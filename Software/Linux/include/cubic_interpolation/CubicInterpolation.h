#pragma once
#include <vector>
#include <stdint.h>
#include <memory>

typedef struct 
{
    double position;
    double velocity;
    double time;
}CubicPoint;

class CubicInterpolation
{
public:
    CubicInterpolation(double period);
    ~CubicInterpolation(){};

    void CubicInterpolationParaClear();
    void CubicInterpolationBegin();
    bool CubicInterpolationRuner(double &pos_out);

    CubicPoint m_cubic_start;
    CubicPoint m_cubic_end;

private:
    uint16_t m_number_of_interpolation = 0;
    uint16_t m_interpolate_count = 0;
    double m_interpolater_period = 0;
    double m_run_time = 0.0;
    double m_start_time = 0.0;
    double m_delta_t = 0.0;

    double m_p_zero_order_term = 0.0;
    double m_p_linear_term = 0.0;
    double m_p_quadratic_term = 0.0;
    double m_p_cubic_term = 0.0;
    double m_v_zero_order_term = 0.0;
    double m_v_linear_term = 0.0;
    double m_v_quadratic_term = 0.0;
    double m_a = 0.0;
    double m_b = 0.0;
    double m_c = 0.0;
    double m_d = 0.0;
};

using CubicPtr = std::shared_ptr<CubicInterpolation>;

