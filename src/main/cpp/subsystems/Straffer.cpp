// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Straffer.h"

Straffer::Straffer() 
{
    // Set the motor configs
    m_motorConfig.SetIdleMode(strafferConstants::Motor::IDLE_MODE)
        .Inverted(strafferConstants::Motor::INVERTED)
        .SmartCurrentLimit(strafferConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(strafferConstants::Motor::RAMP_RATE)
        .VoltageCompensation(strafferConstants::Motor::VOLTAGE_COMPENSATION);

    // Apply the configs to the motors
    m_motor.Configure(m_motorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(strafferConstants::Sensor::Encoder::DISTANCE_PER_PULSE);

    m_strafferPIDController.SetTolerance(strafferConstants::PID::TOLERANCE);
    m_strafferPIDController.SetOutputLimits(strafferConstants::Speed::MIN, strafferConstants::Speed::MAX);

    m_rateLimiter.Reset(0.0, 0.0, strafferConstants::Settings::RATE_LIMITER);

}
void Straffer::SetJoystickInput(double input) 
{
    // assert(((input <= 1) && (input >=-1)) && "Input Joustick Straffer out of range [-1;1].");
    m_joystickInput = m_rateLimiter.Update(input);
}
void Straffer::SetControlMode(ControlMode mode) 
{
    m_state = State ::IDLE;
    m_controlMode = mode;
    m_rateLimiter.m_current = 0.0;
}
ControlMode Straffer::GetControlMode() 
{
    return m_controlMode;
}
double Straffer::GetPosition()
{
    return m_width;
}

void Straffer::Reset() 
{
    if(m_isLeftLimitSwitchTriggered)
    {
        m_motor.Set(0.0);
        m_output = 0.0;
        m_rateLimiter.Reset(0.0, 0.0, strafferConstants::Settings::RATE_LIMITER);
        isInitialized = true;
        m_encoder.Reset();
        m_state = State::STRAFF_TO_STATION;
        m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::CENTER);
    }
    else
    {
        m_motor.Set(strafferConstants::Speed::CALIBRATION);
    }
}
// This method will be called once per scheduler run
void Straffer::Periodic() {
    // ---------------- Save sensors value -----------------
    m_isLeftLimitSwitchTriggered = m_leftLimitSwitch.Get() == strafferConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_isRightLimitSwitchTriggered = m_rightLimitSwitch.Get() == strafferConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_width = m_encoder.GetDistance();

    frc::SmartDashboard::PutBoolean("sLeftLimit", m_isLeftLimitSwitchTriggered);
    frc::SmartDashboard::PutBoolean("sRightLimit", m_isRightLimitSwitchTriggered);
    frc::SmartDashboard::PutNumber("sWidth", m_width);
    frc::SmartDashboard::PutNumber("sState", (int)m_state);

    if(!isInitialized)
    {
        Reset();
        return;
    }
    switch (m_controlMode)
    {
    case ControlMode::CLOSED_LOOP:
        ClosedLoopControl();
        frc::SmartDashboard::PutString("sControlMode", "ClosedLoop");
        break;
    case ControlMode::OPEN_LOOP:
        OpenLoopControl();
        frc::SmartDashboard::PutString("sControlMode", "OpenLoop");
        break;
    case ControlMode::AUTO_LOOP:
        frc::SmartDashboard::PutString("sControlMode", "AutoLoop");
        break;
    default:
        break;
    }
    

    frc::SmartDashboard::PutNumber("sMotor",m_motor.GetAppliedOutput());
}

void Straffer::ClosedLoopControl()
{   
    switch (m_state)
    {
    case State::IDLE :
        break;
    case State::SEEK_APRIL_TAG :
        if(m_counter == 0)
        {
            if(m_lowestAmbiguity > 0.2)
            {
                m_bestAprilTagOffset = 0.0; // default value 
            }
            m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::ORIGIN - m_bestAprilTagOffset + m_targetOffset);
            m_state = State::STRAFF_TO_REEF;
            m_counter = 50;
        }
        else 
        {
            m_counter--;
            m_camera.Update();
            if(m_camera.HasTargets())
            {  
                double currentAmbiguity =  m_camera.GetAmbiguity(m_camera.GetBestTarget());
                if (currentAmbiguity <= m_lowestAmbiguity)
                {
                    m_lowestAmbiguity = currentAmbiguity;
                    m_bestAprilTagOffset = m_camera.GetHorizontalDistance(m_camera.GetBestTarget());
                    frc::SmartDashboard::PutNumber("best", m_bestAprilTagOffset);
                }
            }
        }
        break;
    case State::STRAFF_TO_REEF: 
        if(m_strafferPIDController.AtSetpoint() || (m_counter == 0)) // too slow drop
        {
            m_state = State::AT_REEF;
        }
        else 
        {
            m_counter--;
        }
        break;
    case State::STRAFF_TO_STATION :
        if(m_strafferPIDController.AtSetpoint())
        {
            m_state = State::AT_STATION;
        }
        break;
    case State::AT_REEF :
        break;
    case State::AT_STATION : 
        break;
    default:
        break;
    }
    m_output = m_strafferPIDController.Calculate(m_width);
    if(m_isLeftLimitSwitchTriggered && m_output < 0.0) 
    {
        m_rateLimiter.m_current = 0.0; // useless
        m_output = strafferConstants::Speed::REST;
    }
    else if(m_isRightLimitSwitchTriggered && m_output > 0.0) 
    {
        m_rateLimiter.m_current = 0.0; // useless
        m_output = strafferConstants::Speed::REST;
    }
    m_motor.Set(m_output);
}
void Straffer::OpenLoopControl()
{   
    m_output = m_joystickInput;
    if(m_width < strafferConstants::Settings::LEFT_LIMIT && m_output < 0.0)
    {
        m_rateLimiter.m_current = 0.0;
        m_output = strafferConstants::Speed::REST;
    }
    else if(m_width > strafferConstants::Settings::RIGHT_LIMIT && m_output > 0.0)
    {
        m_rateLimiter.m_current = 0.0;
        m_output = strafferConstants::Speed::REST;
    }
    if(m_isLeftLimitSwitchTriggered && m_output < 0.0) 
    {
        m_rateLimiter.m_current = 0.0;
        m_output = strafferConstants::Speed::REST;
    }
    else if(m_isRightLimitSwitchTriggered && m_output > 0.0) 
    {
        m_rateLimiter.m_current = 0.0;
        m_output = strafferConstants::Speed::REST;
    }
    m_motor.Set(m_output);
}