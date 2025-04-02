// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Outtake.h"

Outtake::Outtake()
{
    // Set the motor config
    m_motorConfig.SetIdleMode(outtakeConstants::Motor::IDLE_MODE)
        .Inverted(outtakeConstants::Motor::INVERTED)
        .SmartCurrentLimit(outtakeConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(outtakeConstants::Motor::RAMP_RATE)
        .VoltageCompensation(outtakeConstants::Motor::VOLTAGE_COMPENSATION);

    m_motor.Configure(  m_motorConfig, 
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Outtake::SetSpeed(double speed) 
{
    m_output = speed;
}
void Outtake::SetControlMode(ControlMode mode) 
{
    m_controlMode = mode;
}
void Outtake::AskToCatch() 
{
    if(m_state == State::REST)
    {
        m_canCatch = true;
    }
}
void Outtake::AskToDrop() 
{
    if(m_state == State::CAUGHT)
    {
        m_canDrop = true;
    }
}
void Outtake::StopAsking() 
{
    m_canCatch = false;
    m_canDrop = false;
}
bool Outtake::IsCaught() 
{
    return m_state == State::CAUGHT;
}
bool Outtake::IsDropped() 
{
    return m_state == State::REST;
}

ControlMode Outtake::GetControlMode()
{
    return m_controlMode;
}


// This method will be called once per scheduler run
void Outtake::Periodic() 
{
    m_isIRBreakerDownTriggered = m_IRBreakerDown.Get() == outtakeConstants::Sensor::IRbreaker::IS_TRIGGERED;
    m_isIRBreakerUpTriggered = m_IRBreakerUp.Get() == outtakeConstants::Sensor::IRbreaker::IS_TRIGGERED 
                            || m_IRBreakerUp2.Get() == outtakeConstants::Sensor::IRbreaker::IS_TRIGGERED;

    switch (m_controlMode)
    {
    case ControlMode::CLOSED_LOOP:
        closedLoop();
        break;
    case ControlMode::OPEN_LOOP:
        openLoop();
        break;
    default:
        break;
    }
}

void Outtake::closedLoop() 
{
    switch (m_state)
    {
    case State::REST:
        if(m_isIRBreakerDownTriggered && !m_isIRBreakerUpTriggered) // is loaded
        {
            m_state = State::STARTING;
            m_motor.Set(outtakeConstants::Speed::UP);
            m_canCatch = true;
        }
        else if(m_isIRBreakerDownTriggered) {
            m_state = State::STARTING;
            m_motor.Set(outtakeConstants::Speed::STARTING);
            m_canCatch = true;
        }
        else if(m_canCatch)
        {
            m_state = State::STARTING;
            m_motor.Set(outtakeConstants::Speed::STARTING);
        }
        break; // end of State::REST
    
    case State::STARTING:
        if(m_isIRBreakerUpTriggered) // is catching
        {
            m_state = State::CATCHING;
            m_motor.Set(outtakeConstants::Speed::CATCHING);
        }
        else if(!m_canCatch) // stop catching
        {
            m_state = State::REST;
            m_motor.Set(outtakeConstants::Speed::REST);
        }
        break; // end of State::STARTING
    
    case State::CATCHING:
        if(m_isIRBreakerDownTriggered && !m_isIRBreakerUpTriggered) // is caught
        {
            m_state = State::CAUGHT;
            m_motor.Set(outtakeConstants::Speed::REST);
            m_canCatch = false;
            isRumbled = true;
            rumbleTime = outtakeConstants::TIME_RUMBLE_CAUGHT;
        }
        break; // end of State::CATCHING

    case State::CAUGHT:
        if(m_canDrop)
        {
            m_state = State::DROPPING;
            m_motor.Set(outtakeConstants::Speed::DROPPING);
            m_counter = 0.0;
        }
        break; // end of State::CAUGHT
    
    case State::DROPPING:
        if(!m_isIRBreakerDownTriggered)
        {
            if(m_counter > outtakeConstants::TIME_FOR_DROP)
            {
                m_state = State::REST;
                m_motor.Set(outtakeConstants::Speed::REST);
                isRumbled = true;
                rumbleTime = outtakeConstants::TIME_RUMBLE_DROPPED;
                m_counter = 0.0;
            }
            else
            {
                m_counter++;
            }
        }
        else if(!m_canDrop)
        {
            if(!m_isIRBreakerDownTriggered) 
            {
                m_state = State::REST;
                m_motor.Set(outtakeConstants::Speed::REST);
                isRumbled = true;
                rumbleTime = outtakeConstants::TIME_RUMBLE_DROPPED;
                m_counter = 0.0;
            }
            else
            {
                m_state = State::STARTING;
                m_motor.Set(outtakeConstants::Speed::UP);
                m_canCatch = true;
            }
        }
        break; // end of State::DROPPING

    default:
        break;
    } // end of switch (m_state)

}
void Outtake::openLoop() 
{
    m_motor.Set(m_output);
}
