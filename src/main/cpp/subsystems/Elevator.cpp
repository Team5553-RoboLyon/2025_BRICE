// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

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
        .VoltageCompensation(elevatorConstants::Motors::Right::VOLTAGE_COMPENSATION)
        .Follow(m_leftMotor, true);

    // Apply the configs to the motors
    m_leftMotor.Configure(  m_leftMotorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_rightMotor.Configure( m_rightMotorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(elevatorConstants::Sensor::Encoder::DISTANCE_PER_PULSE);
    m_encoder.SetReverseDirection(elevatorConstants::Sensor::Encoder::REVERSED);

    m_elevatorPIDController.SetTolerance(elevatorConstants::PID::TOLERANCE);
    m_elevatorPIDController.Reset(elevatorConstants::Setpoint::HOME);
    m_elevatorPIDController.SetOutputLimits(elevatorConstants::Speed::MIN, elevatorConstants::Speed::MAX);
}
void Elevator::SetDesiredHeight(double height) 
{
    if(height < elevatorConstants::Setpoint::HOME)
    {
        height = elevatorConstants::Setpoint::HOME;
        // TODO : add assert
    } 
    else if(height > elevatorConstants::Setpoint::L4)
    {
        height = elevatorConstants::Setpoint::L4;
        // TODO : add assert
    } 
    m_elevatorPIDController.SetSetpoint(height);
}
void Elevator::SetDesiredStage(Stage stage) 
{
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
        break;
    }
}
double Elevator::GetHeight() 
{
    return m_height;
}
void Elevator::SetJoystickInput(double input) 
{
    m_joystickInput = input;
}
void Elevator::SetControlMode(DriveMode mode) 
{
    m_driveMode = mode;
}
DriveMode Elevator::GetControlMode() 
{
    return m_driveMode;
}
bool Elevator::IsAtDesiredStage() 
{
    return m_elevatorPIDController.AtSetpoint();
}
// This method will be called once per scheduler run
void Elevator::Periodic() {

    // ----------------- Save sensors value -----------------
    m_isBottomLimitSwitchTriggered = m_bottomLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_isTopLimitSwitchTriggered = m_topLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED || m_topLimitSwitch2.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_height = m_encoder.GetDistance();

    switch (m_driveMode)
    {
    case DriveMode::CLOSED_LOOP:
        ClosedLoopControl();
        break;
    case DriveMode::OPEN_LOOP:
        OpenLoopControl();
        break;
    default:
        break;
    }
    m_leftMotor.Set(m_output);
}

void Elevator::ClosedLoopControl()
{
    m_output = m_elevatorPIDController.Calculate(m_encoder.GetDistance());
    if(m_isTopLimitSwitchTriggered && m_output > 0.0) 
    {
        m_output = 0.0;
    }
    else if(m_isBottomLimitSwitchTriggered && m_output < 0.0) 
    {
        m_output = 0.0;
    }
}
void Elevator::OpenLoopControl()
{   
    m_output = m_joystickInput;
    if(m_isTopLimitSwitchTriggered && m_output > 0.0) 
    {
        m_output = 0.0;
    }
    else if(m_isBottomLimitSwitchTriggered && m_output < 0.0) 
    {
        m_output = 0.0;
    }

}
