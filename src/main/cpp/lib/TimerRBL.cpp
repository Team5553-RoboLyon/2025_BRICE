#include "lib/TimerRBL.h"

double TimerRBL::GetFPGATimestampInSeconds() 
{
    return frc::RobotController::GetFPGATime() / 1e6;
}

uint64_t TimerRBL::GetFPGATimestampInMicroSeconds() 
{ 
    return frc::RobotController::GetFPGATime();
}

units::time::second_t TimerRBL::GetPeriodRemainingTime() 
{
    return frc::DriverStation::GetMatchTime();
}

TimerRBL::TimerRBL()
{
    Reset();
}

double TimerRBL::GetElapsedTimeSeconds() const 
{
    if (m_isRunning) 
    {
        return m_accumulatedSeconds + (GetSystemMilliseconds() - m_startMilliseconds) / 1000.0;
    } 
    else 
    {
        return m_accumulatedSeconds;
    }
}

void TimerRBL::Reset() 
{
    m_accumulatedSeconds = 0.0;
    m_startMilliseconds = GetSystemMilliseconds();
}

void TimerRBL::Start() 
{
    if (!m_isRunning) 
    {
        m_startMilliseconds = GetSystemMilliseconds();
        m_isRunning = true;
    }
}

void TimerRBL::Stop() 
{
    m_accumulatedSeconds = GetElapsedTimeSeconds();
    m_isRunning = false;
}

void TimerRBL::Restart() 
{
    if (m_isRunning) 
    {
        Stop();
    }
    Reset();
    Start();
}

bool TimerRBL::IsRunning() const 
{
    return m_isRunning;
}

double TimerRBL::GetSystemMilliseconds() const 
{
    return frc::RobotController::GetTime() / 1000.0;
}