#include "lib/pid_rbl.h"

PidRBL::PidRBL(double setpoint, double kp, double ki, double kd) : m_setpoint(setpoint), m_kp(kp), m_ki(ki), m_kd(kd), m_dt(0.02f) {}
PidRBL::PidRBL(double kp, double ki, double kd) : m_setpoint(0.0f), m_kp(kp), m_ki(ki), m_kd(kd), m_dt(0.02f) {}

void PidRBL::SetSetpoint(double setpoint)
{
    m_setpoint = setpoint;
}
void PidRBL::SetGains(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}
void PidRBL::SetTolerance(double tolerance)
{
    m_tolerance = tolerance;
}
void PidRBL::SetOutputLimits(double min, double max)
{
    m_outputMin = min;
    m_outputMax = max;
}

double PidRBL::Calculate(double measurement)
{
    m_error = m_setpoint - measurement;                                     // Calculate error between setpoint and measurement
    m_integrative += m_error * m_dt;                                        // Accumulate error over time
    m_derivative = (m_error - m_lastError) / m_dt;                          // Calculate derivative term
    m_lastError = m_error;                                                  // Save previous error for the next iteration
    m_output = m_kp * m_error + m_ki * m_integrative + m_kd * m_derivative; // Compute PID output
    if (AtSetpoint())                                                       // Check if the measurement is in the tolerance range
    {
        m_output = 0.0; // Stop motor if error is within tolerance
        return m_output;
    }
    return SaturateOutput(); // Saturate output to the limits if it is necessary
}
double PidRBL::Calculate(double setpoint, double measurement)
{
    m_setpoint = setpoint;
    return Calculate(measurement);
}

void PidRBL::Reset(double setpoint)
{
    m_setpoint = setpoint;
    m_lastError = m_error;
    m_error = 0.0f;
    m_output = 0.0f;
    m_integrative = 0.0f;
    m_derivative = 0.0f;
}
bool PidRBL::AtSetpoint()
{
    // return true if the error is within the tolerance
    return abs(m_error) <= m_tolerance;
}
double PidRBL::SaturateOutput()
{
    if (m_output > m_outputMax)
    {
        m_output = m_outputMax;
    }
    else if (m_output < m_outputMin)
    {
        m_output = m_outputMin;
    }
    return m_output;
}
