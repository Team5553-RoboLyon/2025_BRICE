// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"
#include <iostream>

Elevator::Elevator() 
{
    // Set the left motor configs
    m_leftMotorConfig.SetIdleMode(elevatorConstants::Motors::Left::IDLE_MODE)
        .Inverted(elevatorConstants::Motors::Left::INVERTED)
        .SmartCurrentLimit(elevatorConstants::Motors::Left::CURRENT_LIMIT)
        .ClosedLoopRampRate(elevatorConstants::Motors::Left::RAMP_RATE)
        .VoltageCompensation(elevatorConstants::Motors::Left::VOLTAGE_COMPENSATION);

    // Set the right motor configs
    m_rightMotorConfig.SetIdleMode(elevatorConstants::Motors::Right::IDLE_MODE)
        .Inverted(elevatorConstants::Motors::Right::INVERTED)
        .SmartCurrentLimit(elevatorConstants::Motors::Right::CURRENT_LIMIT)
        .ClosedLoopRampRate(elevatorConstants::Motors::Right::RAMP_RATE)
        .VoltageCompensation(elevatorConstants::Motors::Right::VOLTAGE_COMPENSATION);

    // Apply the configs to the motors
    m_leftMotor.Configure(  m_leftMotorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_rightMotor.Configure( m_rightMotorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(elevatorConstants::Sensor::Encoder::DISTANCE_PER_PULSE);

    m_elevatorPIDController.SetTolerance(elevatorConstants::PID::TOLERANCE);
    m_elevatorPIDController.Reset(elevatorConstants::Setpoint::HOME);
    m_elevatorPIDController.SetOutputLimits(elevatorConstants::Speed::MIN, elevatorConstants::Speed::MAX);

    m_rateLimiter.Reset(0.0, 0.0, elevatorConstants::Settings::RATE_LIMITER);
}
void Elevator::SetDesiredHeight(double height) 
{
    // assert(((height >= elevatorConstants::Settings::BOTTOM_LIMIT) && (height <= elevatorConstants::Settings::TOP_LIMIT)) && "Elevator Desired height out of range.");
    if(height < elevatorConstants::Setpoint::HOME) //PROTECTION FOR MATCHES ONLY
    {
        height = elevatorConstants::Setpoint::HOME;
    } 
    else if(height > elevatorConstants::Setpoint::L4)
    {
        height = elevatorConstants::Setpoint::L4;
    } 
    m_elevatorPIDController.SetSetpoint(height);
}
void Elevator::SetDesiredStage(Stage stage) 
{
    m_WantedStage = stage;
    switch (stage)
    {
    case Stage::HOME:
        m_elevatorPIDController.SetSetpoint(elevatorConstants::Setpoint::HOME);
        break;
    case Stage::L1:
        m_elevatorPIDController.SetSetpoint(elevatorConstants::Setpoint::L1);
        break;
    case Stage::CORAL_STATION:
        m_elevatorPIDController.SetSetpoint(elevatorConstants::Setpoint::CORAL_STATION);
        break;
    case Stage::L2:
        m_elevatorPIDController.SetSetpoint(elevatorConstants::Setpoint::L2);
        break;
    case Stage::L3:
        m_elevatorPIDController.SetSetpoint(elevatorConstants::Setpoint::L3);
        break;
    case Stage::L4:
        m_elevatorPIDController.SetSetpoint(elevatorConstants::Setpoint::L4);
        break;
    default:
        // assert(false && "Stage unkown");
        break;
    }
}
double Elevator::GetHeight() 
{
    return m_height;
}
bool Elevator::IsAtCoralStation()
{
    if(NABS(elevatorConstants::Setpoint::CORAL_STATION - m_height) <= 0.05)
        return true;
    else 
        return false;
}
bool Elevator::IsAtL4() 
{
    if(NABS(elevatorConstants::Setpoint::L4 - m_height) <= 0.04)
        return true;
    else 
        return false;
}
void Elevator::SetJoystickInput(double input) 
{
    // assert(((input <= 1) && (input >=-1)) && "Input Joustick Elevator out of range [-1;1].");
    m_joystickInput = m_rateLimiter.Update(std::sin(input * (M_PI / 2.0)));
}
void Elevator::SetControlMode(ControlMode mode) 
{
    m_controlMode = mode;
    m_rateLimiter.m_current = 0.0;
    m_leftMotor.Set(elevatorConstants::Speed::REST);
    m_rightMotor.Set(elevatorConstants::Speed::REST);
}
ControlMode Elevator::GetControlMode() 
{
    return m_controlMode;
}
bool Elevator::IsAtDesiredStage() 
{
    switch (m_controlMode)
    {
    case ControlMode::OPEN_LOOP:
        return true;
        break;
    case ControlMode::CLOSED_LOOP:
        return m_elevatorPIDController.AtSetpoint();    
        break;
    case ControlMode::AUTO_LOOP:
        return false;
        break;
    default:
        break;
    }
}

void Elevator::Periodic() {


    // // ----------------- Save sensors value -----------------
    m_isBottomLimitSwitchTriggered = m_bottomLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED 
                                    || m_bottomLimitSwitch2.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_height = m_encoder.GetDistance();

    if(isInitialized)
    {
        switch (m_controlMode)
        {
        case ControlMode::CLOSED_LOOP:
            m_output = m_elevatorPIDController.Calculate(m_height);
            frc::SmartDashboard::PutString("eControlMode", "ClosedLoop");
            break;
        case ControlMode::OPEN_LOOP:
            m_output = m_joystickInput;
            frc::SmartDashboard::PutString("eControlMode", "OpenLoop");
            break;
        case ControlMode::AUTO_LOOP:
            frc::SmartDashboard::PutString("eControlMode", "AutoLoop");
            break;
        default:
            break;
        }
    }


    // ----------------- Limits -----------------
    if (m_isBottomLimitSwitchTriggered)
    {
        m_output = NMAX(0.0, m_output);
        m_rateLimiter.m_current = 0.0;

        if(!isEncoderAlreadyReset)
        {
            m_encoder.Reset();
            isEncoderAlreadyReset = true;
            isInitialized = true;
        }
    }
    else 
    {
        isEncoderAlreadyReset = false;

        if(m_height > elevatorConstants::Settings::TOP_LIMIT && m_output > 0.0) 
        {
            m_rateLimiter.m_current = 0.0;
            m_output = 0.0;
        }
        else if(m_height < elevatorConstants::Settings::BOTTOM_LIMIT && m_output < 0.0)
        {
            m_rateLimiter.m_current = 0.0;
            m_output = 0.0;
        }
    }
    // std::cout << "m_output" << m_output << std::endl;
    m_leftMotor.Set(m_output);
    m_rightMotor.Set(m_output);

    frc::SmartDashboard::PutNumber("eLeftMotor", m_leftMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("eRightMotor", m_rightMotor.GetAppliedOutput());
    frc::SmartDashboard::PutBoolean("eIs At Setpoint", m_elevatorPIDController.AtSetpoint());
    frc::SmartDashboard::PutBoolean("eBottomSide triggered", m_isBottomLimitSwitchTriggered);
    frc::SmartDashboard::PutNumber("eHeight", m_height);
}