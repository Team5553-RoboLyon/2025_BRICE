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
}
void Straffer::SetJoystickInput(double input) 
{
    m_joystickInput = input;
}
void Straffer::SetControlMode(ControlMode mode) 
{
    m_controlMode = mode;
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
    return m_strafferPIDController.AtSetpoint();
}
// This method will be called once per scheduler run
void Straffer::Periodic() {
    //TODO add reset beginning

    // ----------------- Save sensors value -----------------
    m_isLeftLimitSwitchTriggered = m_leftLimitSwitch.Get() == strafferConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_isRightLimitSwitchTriggered = m_rightLimitSwitch.Get() == strafferConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    m_width = m_encoder.GetDistance();

    switch (m_controlMode)
    {
    case ControlMode::CLOSED_LOOP:
        ClosedLoopControl();
        break;
    case ControlMode::OPEN_LOOP:
        OpenLoopControl();
        break;
    default:
        break;
    }
    m_motor.Set(m_output);
}

void Straffer::ClosedLoopControl()
{
    // à toi de jouer Virgule
    // Tu peux par exemple avoir une methode setsetpoint et actualiser ici.
    // A toi de voir si la logique doit plutot être ici ou dans la commande
    // Le plus important c'est que ça puisse être MODULABLE facilement
    // 
    // Je t'ai aussi rajouté le subsystem Camera pour avoir une base (avec Gaspard)
    // Bonne Chance :-)

    //protection de fin de course
    if(m_isLeftLimitSwitchTriggered && m_output < 0.0) 
    {
        m_output = 0.0;
    }
    else if(m_isRightLimitSwitchTriggered && m_output > 0.0) 
    {
        m_output = 0.0;
    }
}
void Straffer::OpenLoopControl()
{   
    m_output = m_joystickInput;
    if(m_isLeftLimitSwitchTriggered && m_output < 0.0) 
    {
        m_output = 0.0;
    }
    else if(m_isRightLimitSwitchTriggered && m_output > 0.0) 
    {
        m_output = 0.0;
    }
}