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
  
void Outtake::SetControlMode(ControlMode mode) 
{
    m_controlMode = mode;
}
ControlMode Outtake::GetControlMode()
{
    return m_controlMode;
}


void Outtake::SetSpeed(double speed) 
{
    m_output = speed;
}
// This method will be called once per scheduler run
void Outtake::Periodic() 
{
    m_isIRBreakerDownTriggered = m_IRBreakerDown.Get() == outtakeConstants::Sensor::IRbreaker::IS_TRIGGERED;
    m_isIRBreakerUpTriggered = m_IRBreakerUp.Get() == outtakeConstants::Sensor::IRbreaker::IS_TRIGGERED;

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
        if(m_isIRBreakerDownTriggered) // is loaded
        {
            m_state = State::CAUGHT;
            canCatch = false;
        }
        else if(canCatch)
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
        else if(!canCatch) // stop catching
        {
            m_state = State::REST;
            m_motor.Set(outtakeConstants::Speed::REST);
        }
        break; // end of State::STARTING
    
    case State::CATCHING:
        if(m_isIRBreakerDownTriggered) // is caught
        {
            m_state = State::CAUGHT;
            m_motor.Set(outtakeConstants::Speed::REST);
            canCatch = false;
        }
        break; // end of State::CATCHING

    case State::CAUGHT:
        if(canDrop)
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
                m_state = State::DROPPED;
                m_motor.Set(outtakeConstants::Speed::REST);
                isRumbled = true;
                m_counter = 0.0;
            }
            else
            {
                m_counter++;
            }
        }
        else if(!canDrop)
        {
            if(!m_isIRBreakerDownTriggered) 
            {
                m_state = State::DROPPED;
                m_motor.Set(outtakeConstants::Speed::REST);
                isRumbled = true;
                m_counter = 0.0;
            }
            else
            {
                m_state = State::STARTING;
                m_motor.Set(outtakeConstants::Speed::UP);
                canCatch = true;
            }
        }
        break; // end of State::DROPPING
    
    case State::DROPPED: // TODO : add this logic in Robot.cpp
        if(m_counter > outtakeConstants::TIME_FOR_RUMBLE)
        {
            m_state = State::REST;
            m_motor.Set(outtakeConstants::Speed::REST);
            m_counter = 0.0;
            isRumbled = false;
        }
        else
        {
            m_counter++;
        }
        break; // end of State::DROPPED

    default:
        break;
    } // end of switch (m_state)

}
void Outtake::openLoop() 
{
    m_motor.Set(m_output);
}
