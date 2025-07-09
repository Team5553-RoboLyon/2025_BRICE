#include "lib/pidRBL.h"

#include "lib/UtilsRBL.h"
PidRBL::PidRBL(const double kp, const double ki, const double kd) 
                :   m_kp(kp),  
                    m_ki(ki), 
                    m_kd(kd), 
                    m_kf {0.0} // Default feedforward gain is 0.0
{}
PidRBL::PidRBL(const double kp, const double ki, const double kd, const double kf)
                :   m_kp(kp),  
                    m_ki(ki), 
                    m_kd(kd), 
                    m_kf {kf}
{}

void PidRBL::SetGains(const double kp, const double ki, const double kd, const double kf)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
    m_kf = kf;
}

void PidRBL::SetSetpoint(const double setpoint)
{
    if(m_isInputLimitsActive) 
    {
        if(setpoint > m_inputMax)
            m_setpoint = m_inputMax;
        else if(setpoint < m_inputMin)
            m_setpoint = m_inputMin;
        else 
            m_setpoint = setpoint;
    }
    else 
    {
        m_setpoint = setpoint;
    }
}

void PidRBL::SetTolerance(const double tolerance)
{
    m_tolerance = tolerance;
}

void PidRBL::SetOutputLimits(const double min, const double max)
{
    m_outputMin = min;
    m_outputMax = max;
}

void PidRBL::SetInputLimits(const double min, const double max)
{
    m_inputMin = min;
    m_inputMax = max;
}

void PidRBL::SetInputLimits(const bool isActive)
{
    m_isInputLimitsActive = isActive;
}

void PidRBL::SetContinuous(const bool isContinuous)
{
    m_isContinuous = isContinuous;
}

double PidRBL::GetKP() const
{
    return m_kp;
}

double PidRBL::GetKI() const
{
    return m_ki;
}

double PidRBL::GetKD() const
{
    return m_kd;
}

double PidRBL::GetKF() const
{
    return m_kf;
}

double PidRBL::GetError() const
{
    return m_currentError;
}

double PidRBL::GetSetpoint() const
{
    return m_setpoint;
}

std::string PidRBL::GetState() const
{
    std::string state = "PID State: ";
    state += "Kp: " + std::to_string(m_kp) + "\n";
    state += "Ki: " + std::to_string(m_ki) + "\n";
    state += "Kd: " + std::to_string(m_kd) + "\n";
    state += "Kf: " + std::to_string(m_kf) + "\n";
    state += "Setpoint: " + std::to_string(m_setpoint) + "\n";
    state += "Input Min: " + std::to_string(m_inputMin) + "\n";
    state += "Input Max: " + std::to_string(m_inputMax) + "\n";
    state += "Current Error: " + std::to_string(m_currentError) + "\n";
    state += "Output: " + std::to_string(m_output) + "\n";
    state += "Output Min: " + std::to_string(m_outputMin) + "\n";
    state += "Output Max: " + std::to_string(m_outputMax) + "\n";
    state += "Tolerance: " + std::to_string(m_tolerance) + "\n";
    state += "Last Timestamp: " + std::to_string(m_lastTimestamp) + "\n";
    state += "Delta Time: " + std::to_string(m_dt) + "\n";
    state += "Is Continuous: " + std::string(m_isContinuous ? "true" : "false") + "\n";
    state += "Integrative: " + std::to_string(m_integrative) + "\n";
    state += "Previous Error: " + std::to_string(m_previousError) + "\n";
    return state;
}

double PidRBL::CalculateWithRealTime(const double measurement, const double timestamp) {
    // Recalculate dt based on timestamps for real-time systems
    m_dt = timestamp - m_lastTimestamp;
    m_lastTimestamp = timestamp;

    m_currentError = m_setpoint - measurement;
    
    if(m_isContinuous)
    {
        if(NABS(m_currentError) > (m_inputMax - m_inputMin) / 2.0)
        {
            if(m_currentError > 0.0)
            {
                m_currentError -= (m_inputMax - m_inputMin);
            }
            else
            {
                m_currentError += (m_inputMax - m_inputMin);
            }
        }
    }

     // Anti-windup: accumulate integrative error only if P-term is within output bounds
    if((m_currentError * m_kp) < m_outputMax && (m_currentError * m_kp) > m_outputMin)
    {
        m_integrative += m_currentError * m_dt;          
    }
    else 
    {
        m_integrative = 0.0;
    }

    // If error is above tolerance, calculate full PID output
    if(NABS(m_currentError) >= m_tolerance) 
    {
        m_output =  m_kp * m_currentError + 
                    m_ki * m_integrative + 
                    m_kd * ((m_currentError - m_previousError) / m_dt) + 
                    m_kf * m_setpoint;
    } 
    else
    {
        // Near target: skip proportional term to reduce overshoot
        m_output = m_ki * m_integrative + 
                    m_kd * ((m_currentError - m_previousError) / m_dt) + 
                    m_kf * m_setpoint;
    }
    m_previousError = m_currentError;                    

    // Clamp output within allowed range
    if (m_output > m_outputMax)
        m_output = m_outputMax;
    else if (m_output < m_outputMin)
        m_output = m_outputMin;

    return m_output;
}

double PidRBL::CalculateWithRealTime(const double setpoint, const double measurement, const double timestamp) {
    SetSetpoint(setpoint);
    return CalculateWithRealTime(measurement, timestamp);
}

double PidRBL::Calculate(const double measurement)
{
    return CalculateWithRealTime(measurement, m_lastTimestamp + THEORETICAL_DT);
}

double PidRBL::Calculate(const double setpoint, const double measurement) {
    SetSetpoint(setpoint);
    return CalculateWithRealTime(measurement, m_lastTimestamp + THEORETICAL_DT);
}
void PidRBL::Reset()
{
    m_setpoint = 0.0;
    m_previousError = 0.0;
    m_currentError = 0.0;
    m_output = 0.0;
    m_integrative = 0.0;
}

void PidRBL::Reset(const double setpoint)
{
    SetSetpoint(setpoint);
    m_previousError = 0.0;
    m_currentError = 0.0;
    m_output = 0.0;
    m_integrative = 0.0;
}

void PidRBL::ResetIntegrative()
{
    m_integrative = 0.0;
}

bool PidRBL::AtSetpoint() const
{
    return NABS(m_currentError) <= m_tolerance;
}