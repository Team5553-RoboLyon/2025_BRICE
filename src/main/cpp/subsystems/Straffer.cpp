// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Straffer.h"

Straffer::Straffer(Camera *pCamera)  : m_pCamera(pCamera)
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
    m_joystickInput = m_rateLimiter.Update(std::sin(input * (M_PI / 2.0)));
}
void Straffer::SetControlMode(ControlMode mode) 
{
    m_state = State ::IDLE;
    m_controlMode = mode;
    m_rateLimiter.m_current = 0.0;
    m_motor.Set(strafferConstants::Speed::REST);
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
        m_motor.Set(strafferConstants::Speed::REST); 
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
    switch (m_state)
    {
    case State::IDLE:
        frc::SmartDashboard::PutString("sState", "Idle");
        break;
    
    case State::AT_REEF :
    frc::SmartDashboard::PutString("sState", "At Reef");
        break;
    case State::AT_STATION:
        frc::SmartDashboard::PutString("sState", "At Station");
        break;
    case State::SEEK_APRIL_TAG:
        frc::SmartDashboard::PutString("sState", "Seek April Tag");
        break;
    case State::STRAFF_TO_REEF:
        frc::SmartDashboard::PutString("sState", "Straff to Reef");
        break;
    case State::STRAFF_TO_STATION:
        frc::SmartDashboard::PutString("sState", "Straff to station");
        break;
    default:
        break;
    }

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
    frc::SmartDashboard::PutNumber("sJoystick", m_joystickInput);
    frc::SmartDashboard::PutNumber("sSetpoint", m_strafferPIDController.GetSetpoint());
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
            double target = strafferConstants::Setpoint::ORIGIN - m_bestAprilTagOffset + m_targetOffset;
            if(target >= strafferConstants::Settings::LEFT_LIMIT && target <= strafferConstants::Settings::RIGHT_LIMIT)
            {
                m_state = State::STRAFF_TO_REEF;
                m_counter = strafferConstants::Counter::STRAFF_TO_REEF;
                m_strafferPIDController.SetSetpoint(target);
            }
            else
            { 
                target = strafferConstants::Setpoint::CENTER;
                m_rumble = true;
                m_state = State::STRAFF_TO_STATION;
                m_counter = strafferConstants::Counter::STRAFF_TO_STATION;
                m_strafferPIDController.SetSetpoint(target);
            }
        }
        else 
        {
            m_counter--;
            m_pCamera->Update();
            if(m_pCamera->HasTargets())
            {  
                double currentAmbiguity =  m_pCamera->GetAmbiguity(m_pCamera->GetBestTarget());
                if (currentAmbiguity <= m_lowestAmbiguity)
                {
                    m_lowestAmbiguity = currentAmbiguity;
                    m_bestAprilTagOffset = m_pCamera->GetHorizontalDistance(m_pCamera->GetBestTarget());
                    frc::SmartDashboard::PutNumber("sBest AprilTag Offset", m_bestAprilTagOffset);
                }
            }
        }
        break;
    case State::STRAFF_TO_REEF: 
        if(m_strafferPIDController.AtSetpoint() || (m_counter == 0))
        {
            m_state = State::AT_REEF;
        }
        else 
        {
            m_counter--;
        }
        break;
    case State::STRAFF_TO_STATION :
        if(m_strafferPIDController.AtSetpoint()  || (m_counter == 0))
        {
            m_state = State::AT_STATION;
        }
        else 
        {
            m_counter--;
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