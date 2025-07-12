#include "subsystems/gripper/GripperSubsystem.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include "lib/DebugUtils.h"
GripperSubsystem::GripperSubsystem(GripperIO *pIo) : 
                                                    m_pGripperIO(pIo)
{
}
void GripperSubsystem::SetControlMode(const ControlMode mode)
{
    m_controlMode = mode;
    m_wantedState = WantedState::STAND_BY;
    m_systemState = SystemState::IDLE;
    m_feederOutput = 0.0;
    m_outtakeOutput = 0.0;
}
ControlMode GripperSubsystem::GetControlMode()
{
    return m_controlMode;
}
void GripperSubsystem::ToggleControlMode()
{
    m_wantedState = WantedState::STAND_BY;
    m_systemState = SystemState::IDLE;
    m_feederOutput = 0.0;
    m_outtakeOutput = 0.0;
    switch (m_controlMode)
    {
    case gripperConstants::DefaultMode :
        m_controlMode = ControlMode::OPEN_LOOP;
        break;
    case ControlMode::OPEN_LOOP : 
        m_controlMode = gripperConstants::DefaultMode;
        break;
    default:
        DEBUG_ASSERT(false,"Gripper : Toggle impossible with an unrecognized mode.");
        break;
    }
}
void GripperSubsystem::SetWantedState(const WantedState wantedState)
{
    m_wantedState = wantedState;
}
GripperSubsystem::SystemState GripperSubsystem::GetSystemState()
{
    return m_systemState;
}
void GripperSubsystem::SetOutputInOpenLoop(double dutyCycle)
{
    if(m_controlMode == ControlMode::OPEN_LOOP)
    {
        DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
            , "Gripper Duty Cycle out of range");
        m_feederOutput = (std::sin(dutyCycle * (M_PI / 2.0)) /gripperConstants::OPEN_LOOP_REDUC);
        m_outtakeOutput = (std::sin(dutyCycle * (M_PI / 2.0)) /gripperConstants::OPEN_LOOP_REDUC);
    }
    else 
    {
        DEBUG_ASSERT(false,"Gripper : Open Loop Output set while Closed Loop is used");
    }
}
void GripperSubsystem::SetOutputInOpenLoop(double feederDutyCycle, double outtakeDutyCycle)
{
    if(m_controlMode == ControlMode::OPEN_LOOP)
    {
        DEBUG_ASSERT((feederDutyCycle <= 1.0) && (feederDutyCycle >= -1.0) 
            , "Feeder Duty Cycle out of range");
        DEBUG_ASSERT((outtakeDutyCycle <= 1.0) && (outtakeDutyCycle >= -1.0) 
            , "Outtake Duty Cycle out of range");
        m_feederOutput = (std::sin(feederDutyCycle * (M_PI / 2.0)) /gripperConstants::OPEN_LOOP_REDUC);
        m_outtakeOutput = (std::sin(outtakeDutyCycle * (M_PI / 2.0)) /gripperConstants::OPEN_LOOP_REDUC);
    }
    else 
    {
        DEBUG_ASSERT(false, "Gripper : Open Loop Output set while Closed Loop is used");
    }
}
bool GripperSubsystem::IsResting()
{
    DEBUG_ASSERT(ALLOWS_STATE_MACHINE(m_controlMode), "Gripper : IsResting() is used while Open Loop");
    return ((m_systemState == SystemState::REST_EMPTY) || 
            (m_systemState == SystemState::REST_LOADED) || 
            (m_systemState == SystemState::REST_SHIFTED));
}

// This method will be called once per scheduler run
void GripperSubsystem::Periodic() 
{
    m_currentWantedState = m_wantedState;

    m_pGripperIO->UpdateInputs(inputs);
    m_feederMotorDisconnected.Set(!inputs.isFeederMotorConnected);
    m_outtakeMotorDisconnected.Set(!inputs.isOuttakeMotorConnected);
    m_feederOverheating.Set(inputs.feederTemperature > feederConstants::Motor::OVERHEATING_THRESHOLD); 
    m_feederHot.Set(inputs.feederTemperature > feederConstants::Motor::HOT_THRESHOLD);
    m_outtakeOverheating.Set(inputs.outtakeTemperature > outtakeConstants::Motor::OVERHEATING_THRESHOLD); 
    m_outtakeHot.Set(inputs.outtakeTemperature > outtakeConstants::Motor::HOT_THRESHOLD);




    if(ALLOWS_STATE_MACHINE(m_controlMode))
    {
        RunStateMachine();
    }

    switch (m_controlMode) // Actualise motion
    {
    case ControlMode::DUTY_CYCLE :
        switch (m_systemState)
        {
        case SystemState::COLLECTING_EMPTY :
            m_feederOutput = feederConstants::DutyCycle::COLLECTING;
            m_outtakeOutput = outtakeConstants::DutyCycle::FEEDING_EMPTY;
            break; //end of SystemState::COLLECTING_EMPTY

        case SystemState::FEEDING_BACKWARD : 
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::FEEDING_BACKWARD;
            break; //end of SystemState::FEEDING_BACKWARD

        case SystemState::FEEDING_FORWARD : 
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::FEEDING_FORWARD;
            break; //end of SystemState::FEEDING_FORWARD

        case SystemState::FEEDING_FORWARD_SHY :
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::SHY;
            break; //end of SystemState::FEEDING_FORWARD_SHY

        case SystemState::PRESHOOT :
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::PRESHOOT;
            break; //end of SystemState::PRESHOOT

        case SystemState::REJECTING_BACKWARD :
            m_feederOutput = feederConstants::DutyCycle::REJECTING_BACKWARD;
            m_outtakeOutput = outtakeConstants::DutyCycle::REJECTING_BACKWARD;
            break; //end of SystemState::REJECTING_BACKWARD

        case SystemState::REJECTING_FORWARD :
            m_feederOutput = feederConstants::DutyCycle::REJECTING_FORWARD;    
            m_outtakeOutput = outtakeConstants::DutyCycle::REJECTING_FORWARD;
            break; //end of SystemState::REJECTING_FORWARD

        case SystemState::SHIFTING_FORWARD : 
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::FEEDING_FORWARD;
            break; //end of SystemState::SHIFTING_FORWARD

        case SystemState::HIGH_SHOOTING :
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::HIGH_SHOOTING;
            break; //end of SystemState::HIGH_SHOOTING
        
        case SystemState::MIDDLE_SHOOTING :
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::MIDDLE_SHOOTING;
            break; //end of SystemState::MIDDLE_SHOOTING
        
        case SystemState::LOW_SHOOTING :
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::LOW_SHOOTING;
            break; //end of SystemState::LOW_SHOOTING
            
        case SystemState::REST_EMPTY :
        case SystemState::REST_LOADED : 
        case SystemState::REST_SHIFTED :
        case SystemState::IDLE :
            m_feederOutput = feederConstants::DutyCycle::REST;
            m_outtakeOutput = outtakeConstants::DutyCycle::REST;
            break; //end of "resting" states

        default:
            DEBUG_ASSERT(false, "Gripper : impossible state");
            break;
        }

        m_pGripperIO->SetFeederDutyCycle(m_feederOutput);
        m_pGripperIO->SetOuttakeDutyCycle(m_outtakeOutput);
        break; //end of ControlMode::DUTY_CYCLE

    case ControlMode::OPEN_LOOP :
        m_pGripperIO->SetFeederDutyCycle(m_feederOutput);
        m_pGripperIO->SetOuttakeDutyCycle(m_outtakeOutput);
        break; //end of ControlMode::OPEN_LOOP

    case ControlMode::VELOCITY :
        switch (m_systemState)
        {
        case SystemState::COLLECTING_EMPTY :
            m_feederOutput = feederConstants::RPM::COLLECTING;
            m_outtakeOutput = outtakeConstants::RPM::FEEDING_EMPTY;
            break; //end of SystemState::COLLECTING_EMPTY

        case SystemState::FEEDING_BACKWARD : 
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::FEEDING_BACKWARD;
            break; //end of SystemState::FEEDING_BACKWARD

        case SystemState::FEEDING_FORWARD : 
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::FEEDING_FORWARD;
            break; //end of SystemState::FEEDING_FORWARD

        case SystemState::FEEDING_FORWARD_SHY :
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::SHY;
            break; //end of SystemState::FEEDING_FORWARD_SHY

        case SystemState::PRESHOOT :
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::PRESHOOT;
            break; //end of SystemState::PRESHOOT

        case SystemState::REJECTING_BACKWARD :
            m_feederOutput = feederConstants::RPM::REJECTING_BACKWARD;
            m_outtakeOutput = outtakeConstants::RPM::REJECTING_BACKWARD;
            break; //end of SystemState::REJECTING_BACKWARD

        case SystemState::REJECTING_FORWARD :
            m_feederOutput = feederConstants::RPM::REJECTING_FORWARD;
            m_outtakeOutput = outtakeConstants::RPM::REJECTING_FORWARD;
            break; //end of SystemState::REJECTING_FORWARD

        case SystemState::SHIFTING_FORWARD : 
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::SHIFTING;
            break; //end of SystemState::SHIFTING_FORWARD

        case SystemState::HIGH_SHOOTING :
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::HIGH_SHOOTING;
            break; //end of SystemState::HIGH_SHOOTING

        case SystemState::MIDDLE_SHOOTING :
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::MIDDLE_SHOOTING;
            break; //end of SystemState::MIDDLE_SHOOTING
        
        case SystemState::LOW_SHOOTING :
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::LOW_SHOOTING;
            break; //end of SystemState::LOW_SHOOTING

        case SystemState::REST_EMPTY :
        case SystemState::REST_LOADED : 
        case SystemState::REST_SHIFTED :
        case SystemState::IDLE :
            m_feederOutput = feederConstants::RPM::REST;
            m_outtakeOutput = outtakeConstants::RPM::REST;
            break; //end of "resting" states

        default:
            DEBUG_ASSERT(false, "Gripper : impossible state");
            break;
        }

        m_pGripperIO->SetFeederRPM(m_feederOutput);
        m_pGripperIO->SetOuttakeRPM(m_outtakeOutput);
        break; //end of ControlMode::Velocity
    default:
        DEBUG_ASSERT(false, "Gripper : wrong ControlMode chosen");
        m_feederOutput = 0.0; // protection
        m_outtakeOutput = 0.0; // protection
        break;
    }

    //LOG
    frc::SmartDashboard::PutNumber("G.WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("G.SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("G.ControlMode", (int)m_controlMode);
}

void GripperSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::LOAD :
        if(m_systemState == SystemState::REST_EMPTY)
        {
            m_systemState = SystemState::COLLECTING_EMPTY;
        }
        break; //end of WantedState::LOAD

    case WantedState::SCORE_HIGH :
    case WantedState::SCORE_MIDDLE : 
    case WantedState::SCORE_LOW :
        if (m_systemState == SystemState::REST_LOADED)
        {
            m_systemState = SystemState::PRESHOOT;
            m_counter = gripperConstants::Counter::PRESHOOT;
        }
        break; //end of WantedState::Score

    case WantedState::TOGGLE :
        if ( (m_systemState == SystemState::REST_LOADED)
         || (m_systemState == SystemState::REST_SHIFTED) )
        {
            m_systemState = SystemState::FEEDING_BACKWARD;
        }
        break; //end of WantedState::TOGGLE

    case WantedState::SHIFT_FRONT :
        if(m_systemState == SystemState::REST_LOADED)
        {
            m_systemState = SystemState::SHIFTING_FORWARD;
        }
        break; //end of WantedState::SHIFT_FRONT

    case WantedState::STAND_BY :
        if(m_systemState == SystemState::COLLECTING_EMPTY)
        {
            m_systemState = SystemState::REST_EMPTY;
        }
        break; //end of WantedState::STAND_BY

    // In these cases, the wantedState is applied regardless of the current systemState.
    // This helps resynchronize the state machine with the robot's actual state in case of a discrepancy or fault.
    case WantedState::REJECT_BACKWARD :
        m_systemState = SystemState::REJECTING_BACKWARD;
        break; //end of WantedState::REJECT_BACKWARD
    case WantedState::REJECT_FORWARD :
        m_systemState = SystemState::REJECTING_FORWARD;
        break; //end of WantedState::REJECT_FORWARD

    default:
        DEBUG_ASSERT(false, "Gripper : impossible state");
        break;
    } // switch(m_currentWantedState)

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE :
        if(inputs.IRBreakerUp || inputs.IRBreakerUp2)
        {
            m_systemState = SystemState::FEEDING_FORWARD;
        }
        else if(inputs.IRBreakerDown)
        {
            m_systemState = SystemState::FEEDING_BACKWARD;
        }
        else // nothing inside the gripper
        {
            m_systemState = SystemState::REST_EMPTY;
        }
        break; //end of SystemState::IDLE 

    case SystemState::COLLECTING_EMPTY :
        if(inputs.IRBreakerUp || inputs.IRBreakerUp2)
        {
            m_systemState = SystemState::FEEDING_FORWARD;
        }
        break; // end of SystemState::COLLECTING_EMPTY

    case SystemState::FEEDING_BACKWARD : 
        if(inputs.IRBreakerUp || inputs.IRBreakerUp2)
        {
            m_systemState = SystemState::FEEDING_FORWARD_SHY;
        }
        break;

    case SystemState::FEEDING_FORWARD : 
        if(!(inputs.IRBreakerUp || inputs.IRBreakerUp2))
        {
            m_systemState = SystemState::FEEDING_BACKWARD;
        }
        break;
    
    case SystemState::FEEDING_FORWARD_SHY :
        if(!(inputs.IRBreakerUp || inputs.IRBreakerUp2))
        {
            m_systemState = SystemState::REST_LOADED;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
            CanRumble = true;
        }
        break;
    
    case SystemState::PRESHOOT :
        if(m_counter!=0)
            m_counter--;
        else 
        {
            switch (m_currentWantedState)
            {
            case WantedState::SCORE_HIGH :
                m_systemState = SystemState::HIGH_SHOOTING;
                break;
            case WantedState::SCORE_MIDDLE :
                m_systemState = SystemState::MIDDLE_SHOOTING;
                break;
            case WantedState::SCORE_LOW :
                m_systemState = SystemState::HIGH_SHOOTING;
                break;
            default:
                DEBUG_ASSERT(false, "Gripper : No shoot desired after Preshoot ???");
                break;
            }
            m_counter = gripperConstants::Counter::SHOOT;
        }
        break;

    case SystemState::REJECTING_BACKWARD :
        if(m_counter!=0)
            m_counter--;
        else 
        {
            m_systemState = SystemState::REST_EMPTY;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
            CanRumble = true;
        }
        break;
    
    case SystemState::REJECTING_FORWARD :
        if(m_counter!=0) 
            m_counter--;
        else 
        {
            m_systemState = SystemState::REST_EMPTY;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
            CanRumble = true;
        }
        break;
    
    case SystemState::SHIFTING_FORWARD : 
        if(m_counter!=0)
            m_counter--;
        else 
        {
            m_systemState = SystemState::REST_SHIFTED;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
            CanRumble = true;
        }
        break;
    
    case SystemState::HIGH_SHOOTING :
    case SystemState::MIDDLE_SHOOTING :
    case SystemState::LOW_SHOOTING :
        if(m_counter!=0)
            m_counter--;
        else 
        {
            m_systemState = SystemState::REST_EMPTY;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY;
            CanRumble = true;
        }
        break;
    
    case SystemState::REST_EMPTY :
    case SystemState::REST_LOADED : 
    case SystemState::REST_SHIFTED :
        break;

    default:
        DEBUG_ASSERT(false, "Gripper : impossible state");
        break;
    }
}