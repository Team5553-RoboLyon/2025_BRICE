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
    m_integrative += m_error;                                               // Update integrative term
    m_derivative = m_error - m_lastError;                                   // Calculate derivative term
    m_lastError = m_error;                                                  // Save current error
    m_output = m_kp * m_error + m_ki * m_integrative + m_kd * m_derivative; // Compute PID output
    if (AtSetpoint())
    {
        m_output = 0.0; // Stop motor if error is within tolerance
    }
    return m_output;
}
double PidRBL::Calculate(double setpoint, double measurement)
{
    m_setpoint = setpoint;
    return Calculate(measurement);
}

void PidRBL::Reset(double error, double lastError, double integrative, double derivative, double output)
{
    m_error = error;
    m_lastError = lastError;
    m_integrative = integrative;
    m_derivative = derivative;
    m_output = output;
}
bool PidRBL::AtSetpoint()
{
    // return true if the error is close to the tolerance
    return abs(m_error) < m_tolerance;

    m_integrative += m_error * m_dt;                                        // Accumulate error over time
    m_derivative = (m_error - m_lastError) / m_dt;                          // Calculate derivative term
    m_lastError = m_error;                                                  // Save previous error for the next iteration
    m_output = m_kp * m_error + m_ki * m_integrative + m_kd * m_derivative; // Compute PID output
    if (AtSetpoint())                                                       // Check if the measurement is in the tolerance range
    {
        m_output = 0.0; // Stop motor if error is within tolerance
        return m_output;
    }
    //Saturate output to the limits if it is necessary
    if (m_output > m_outputMax)
        m_output = m_outputMax;
    else if (m_output < m_outputMin)
        m_output = m_outputMin;
    return m_output;
}
double PidRBL::Calculate(double setpoint, double measurement) {
    m_setpoint = setpoint;
    return Calculate(measurement);
}
double PidRBL::CalculateWithError(double error) {
    m_error = error;
    m_integrative += m_error * m_dt;                                        // Accumulate error over time
    m_derivative = (m_error - m_lastError) / m_dt;                          // Calculate derivative term
    m_lastError = m_error;                                                  // Save previous error for the next iteration
    m_output = m_kp * m_error + m_ki * m_integrative + m_kd * m_derivative; // Compute PID output
    if (AtSetpoint())                                                       // Check if the measurement is in the tolerance range
    {
        m_output = 0.0; // Stop motor if error is within tolerance
        return m_output;
    }
    //Saturate output to the limits if it is necessary
    if (m_output > m_outputMax)
        m_output = m_outputMax;
    else if (m_output < m_outputMin) 
        m_output = m_outputMin;
    return m_output;
}

void PidRBL::Reset(double setpoint)
{
    m_setpoint = setpoint;
    m_lastError = m_error;
    m_error = 0.0;
    m_output = 0.0;
    m_integrative = 0.0;
    m_derivative = 0.0;
}
bool PidRBL::AtSetpoint()
{
    // return true if the error is within the tolerance
    return NABS(m_error) <= m_tolerance;
}