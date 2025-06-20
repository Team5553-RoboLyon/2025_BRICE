// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gripper.h"

Gripper::Gripper()
{
    // Set the motor config
    m_outtakeMotorConfig.SetIdleMode(gripperConstants::Motors::Outtake::IDLE_MODE)
        .Inverted(gripperConstants::Motors::Outtake::INVERTED)
        .SmartCurrentLimit(gripperConstants::Motors::Outtake::CURRENT_LIMIT)
        .ClosedLoopRampRate(gripperConstants::Motors::Outtake::RAMP_RATE)
        .VoltageCompensation(gripperConstants::Motors::Outtake::VOLTAGE_COMPENSATION);
    
    m_intakeMotorConfig.SetIdleMode(gripperConstants::Motors::Intake::IDLE_MODE)
        .Inverted(gripperConstants::Motors::Intake::INVERTED)
        .SmartCurrentLimit(gripperConstants::Motors::Intake::CURRENT_LIMIT)
        .ClosedLoopRampRate(gripperConstants::Motors::Intake::RAMP_RATE)
        .VoltageCompensation(gripperConstants::Motors::Intake::VOLTAGE_COMPENSATION);

    m_outtakeMotor.Configure(  m_outtakeMotorConfig, 
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_intakeMotor.Configure(  m_intakeMotorConfig, 
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void Gripper::SetSpeedIntake(double intakeSpeed)
{
    m_intakeMotor.Set(intakeSpeed);
}
void Gripper::SetSpeedOuttake(double outtakeSpeed)
{
    m_outtakeMotor.Set(outtakeSpeed);
}
void Gripper::SetControlMode(ControlMode mode) 
{
    m_controlMode = mode;
    m_state = State::IDLE; // reset state when changing control mode
    m_outtakeMotor.Set(gripperConstants::Speed::REST);
    m_intakeMotor.Set(gripperConstants::Speed::REST);
}

ControlMode Gripper::GetControlMode()
{
    return m_controlMode;
}
void Gripper::ToggleControlMode() // only between Open and Closed Loop
{
    switch (m_controlMode)
    {
    case ControlMode::OPEN_LOOP :
        m_controlMode = ControlMode::CLOSED_LOOP;
        m_state = State::IDLE; // reset state when changing control mode
        m_outtakeMotor.Set(gripperConstants::Speed::REST);
        m_intakeMotor.Set(gripperConstants::Speed::REST);
        break;
    case ControlMode::CLOSED_LOOP :
        m_controlMode = ControlMode::OPEN_LOOP;
        m_state = State::IDLE; // reset state when changing control mode
        m_outtakeMotor.Set(gripperConstants::Speed::REST);
        m_intakeMotor.Set(gripperConstants::Speed::REST);
        break;
    default:
        break;
    }
}
bool Gripper::IsMoving() 
{
    if(m_outtakeMotor.GetAppliedOutput() != gripperConstants::Speed::REST)
        return true;
    else
        return false;
}


// This method will be called once per scheduler run
void Gripper::Periodic() 
{
    m_isIRBreakerDownTriggered = m_IRBreakerDown.Get() == gripperConstants::Sensor::IRbreaker::IS_TRIGGERED;
    m_isIRBreakerUpTriggered = m_IRBreakerUp.Get() == gripperConstants::Sensor::IRbreaker::IS_TRIGGERED 
                            || m_IRBreakerUp2.Get() == gripperConstants::Sensor::IRbreaker::IS_TRIGGERED;

    switch (m_state)
    {
    case State::IDLE:
        frc::SmartDashboard::PutString("gState", "Idle");
        break;
    case State::INTAKE_EMPTY:
        frc::SmartDashboard::PutString("gState", "Intake Empty");
        break;
    case State::INTAKE_FEEDING_BACKWARD:
        frc::SmartDashboard::PutString("gState", "Intake Feeding Backward");
        break;
    case State::INTAKE_FEEDING_FORWARD:
        frc::SmartDashboard::PutString("gState", "Intake Feeding Forward");
        break;
    case State::INTAKE_FEEDING_FORWARD_SHY:
        frc::SmartDashboard::PutString("gState", "Intake Feeding Forward Shy");
        break;
    case State::PRESHOOT :
        frc::SmartDashboard::PutString("gState", "Preshoot");
        break;
    case State::REST_EMPTY :
        frc::SmartDashboard::PutString("gState", "Rest Empty");
        break;
    case State::REST_LOADED :
        frc::SmartDashboard::PutString("gState", "Rest Loaded");
        break;
    case State::SHOOT :
        frc::SmartDashboard::PutString("gState", "Shoot");
        break;
    default:
        break;
    }
    switch (m_controlMode)
    {
    case ControlMode::CLOSED_LOOP:
        ClosedLoopControl();
        frc::SmartDashboard::PutString("gControlMode", "ClosedLoop");
        break;
    case ControlMode::OPEN_LOOP:
        OpenLoopControl();
        frc::SmartDashboard::PutString("gControlMode", "OpenLoop");
        break;
    case ControlMode::AUTO_LOOP:
        frc::SmartDashboard::PutString("gControlMode", "AutoLoop");
        break;
    default:
        break;
    }

    frc::SmartDashboard::PutBoolean("gIRbreakerDown", m_isIRBreakerDownTriggered);
    frc::SmartDashboard::PutBoolean("gIRbreakerUp", m_isIRBreakerUpTriggered);
    // frc::SmartDashboard::PutNumber("gState", (int)m_state);
    frc::SmartDashboard::PutNumber("gOuttakeMotor", m_outtakeMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("gIntakeMotor", m_intakeMotor.GetAppliedOutput());
}

void Gripper::ClosedLoopControl() 
{
    switch (m_state)
    {
    case State::IDLE:
        if(m_isIRBreakerUpTriggered)
        {
            m_state = State::INTAKE_FEEDING_FORWARD;
            m_outtakeMotor.Set(gripperConstants::Speed::FEEDING_FORWARD);
        }
        else if(m_isIRBreakerDownTriggered)
        {
           m_state = State::INTAKE_FEEDING_BACKWARD;
           m_outtakeMotor.Set(gripperConstants::Speed::FEEDING_BACKWARD);
        }
        else
        {
            m_state = State::REST_EMPTY;
            m_outtakeMotor.Set(gripperConstants::Speed::REST);
        }
        break; // end of State::IDLE
    
    case State::REST_EMPTY: // etat d'attente/vide/voila
        // Peut se rendre en Intake Empty via la commandde
        break; // end of State::REST_EMPTY
    
    case State::INTAKE_EMPTY:
        if(m_isIRBreakerUpTriggered)
        {
            m_state = State::INTAKE_FEEDING_FORWARD;
            m_outtakeMotor.Set(gripperConstants::Speed::FEEDING_FORWARD);
            m_intakeMotor.Set(gripperConstants::Speed::REST);
        }
        break; //end of State::INTAKE_EMPTY
    
    case State::INTAKE_FEEDING_FORWARD :
        if(!m_isIRBreakerUpTriggered)
        {
            m_state = State::INTAKE_FEEDING_BACKWARD;
            m_outtakeMotor.Set(gripperConstants::Speed::FEEDING_BACKWARD);
        }
        break; //end of State::INTAKE_FEEDING_FORWARD
    
    case State::INTAKE_FEEDING_BACKWARD :
        if(m_isIRBreakerUpTriggered)
        {
            m_state = State::INTAKE_FEEDING_FORWARD_SHY;
            m_outtakeMotor.Set(gripperConstants::Speed::SHY);
        }
        break; //end of State::INTAKE_FEEDING_BACKWARD
    
    case State::INTAKE_FEEDING_FORWARD_SHY :
        if(!m_isIRBreakerUpTriggered)
        {
            m_state = State::REST_LOADED;
            // m_targetRumble = Rumble::CAUGHT;
            m_rumble = true;
            m_outtakeMotor.Set(gripperConstants::Speed::REST);
        }
        break; //end of State::INTAKE_FEEDING_FORWARD_SHY
    
    case State::PRESHOOT :
        if(m_counter!=0)
            m_counter--;
        else 
        {
            m_state = State::SHOOT;
            m_outtakeMotor.Set(m_ShoooootttttSpeed);
            m_counter = gripperConstants::Counter::SHOOT; // durée shoot
        }
        break; //end of State::PRESHOOT

    case State::SHOOT :
        if(m_counter!=0)
            m_counter--;
        else 
        {
            m_state = State::REST_EMPTY;
            m_outtakeMotor.Set(gripperConstants::Speed::REST);
            // m_targetRumble = Rumble::DROPPED;
            m_rumble = true;
        }
        break; //end of State::SHOOT

    case State::REST_LOADED :
        // Peut se rendre en State::PRESHOOT via la commande :
        // PEut se rendre en State::INTAKE_FEEDING_BACKWARD via la commande:
        if(!m_isIRBreakerDownTriggered && !m_isIRBreakerUpTriggered)
        {
            m_state = State::REST_EMPTY;
            m_outtakeMotor.Set(gripperConstants::Speed::REST); // inutile mais JM le veut
        }
        break; // end of State::REST_LOADED
        
    default:
        break;
    }
}
void Gripper::OpenLoopControl() 
{
    // in setSpeed
}