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
    assert(((input <= 1) && (input >=-1)) && "Input Joustick Straffer out of range [-1;1].");
    m_joystickInput = m_rateLimiter.Update(input);
}
void Straffer::SetControlMode(ControlMode mode) 
{
    m_controlMode = mode;
    m_rateLimiter.m_current = 0.0;
}
void Straffer::SetDesiredPosition(double Position) 
{
    assert(((Position >= strafferConstants::Settings::LEFT_LIMIT) && (Position <= strafferConstants::Settings::RIGHT_LIMIT)) 
            && "Straffer Desired position out of range.");
    m_strafferPIDController.SetSetpoint(Position);
}
ControlMode Straffer::GetControlMode() 
{
    return m_controlMode;
}
double Straffer::GetPosition()
{
    return m_width;
}
bool Straffer::IsAtDesiredPosition() 
{
    switch (m_controlMode)
    {
    case ControlMode::OPEN_LOOP:
        return true;
        break;
    case ControlMode::CLOSED_LOOP:
        return m_strafferPIDController.AtSetpoint();    
        break;
    case ControlMode::AUTO_LOOP:
        return false;
        break;
    default:
        break;
    }
}
void Straffer::SetDesiredSide(Side side)
{
    switch (side)
    {
    case Side::CENTER:
        m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::CENTER);
        break;
    case Side::RIGHT:
        //TODO : add error compute with Camera
        m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::RIGHT_SIDE);
        break;
    case Side::LEFT:
        //TODO : add error compute with Camera  
        m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::LEFT_SIDE);
    default:
        break;
    }
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
        //TODO
        break;
    default:
        break;
    }

    frc::SmartDashboard::PutNumber("sMotor",m_motor.GetAppliedOutput());
}

void Straffer::ClosedLoopControl()
{
    m_output = m_strafferPIDController.Calculate(m_width);
    if(m_isLeftLimitSwitchTriggered && m_output < 0.0) 
    {
        m_rateLimiter.m_current = 0.0;
        m_output = 0.0;
    }
    else if(m_isRightLimitSwitchTriggered && m_output > 0.0) 
    {
        m_rateLimiter.m_current = 0.0;
        m_output = 0.0;
    }
    m_motor.Set(m_output);
}
void Straffer::OpenLoopControl()
{   
    m_output = m_joystickInput;
    if(m_width < strafferConstants::Settings::LEFT_LIMIT && m_output < 0.0)
    {
        m_rateLimiter.m_current = 0.0;
        m_output = 0.0;
    }
    else if(m_width > strafferConstants::Settings::RIGHT_LIMIT && m_output > 0.0)
    {
        m_rateLimiter.m_current = 0.0;
        m_output = 0.0;
    }
    if(m_isLeftLimitSwitchTriggered && m_output < 0.0) 
    {
        m_rateLimiter.m_current = 0.0;
        m_output = 0.0;
    }
    else if(m_isRightLimitSwitchTriggered && m_output > 0.0) 
    {
        m_rateLimiter.m_current = 0.0;
        m_output = 0.0;
    }
    m_motor.Set(m_output);
}