#include "subsystems/elevator/ElevatorSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/Timer.h>
#include "lib/DebugUtils.h"

ElevatorSubsystem::ElevatorSubsystem(ElevatorIO *pIO) : 
                                                    m_pElevatorIO(pIO)
{
    m_elevatorPIDController.SetTolerance(elevatorConstants::PID::TOLERANCE);
    m_elevatorPIDController.Reset(elevatorConstants::Setpoint::HOME);
    m_elevatorPIDController.SetOutputLimits(elevatorConstants::Speed::MIN, elevatorConstants::Speed::MAX);
    m_elevatorPIDController.SetInputLimits( elevatorConstants::Settings::BOTTOM_LIMIT, 
                                            elevatorConstants::Settings::TOP_LIMIT);
}

void ElevatorSubsystem::SetWantedState(const WantedState wantedState)
{
    if(wantedState == WantedState::INITIALIZATION)
    {
        if(!m_isInitialized)  // Skip initialization if the subsystem is already initialized
            m_wantedState = WantedState::INITIALIZATION; 
    }
    else // if(wantedState != WantedState::INITIALIZATION)
    {
        m_wantedState = wantedState;
    }
}

ElevatorSubsystem::SystemState ElevatorSubsystem::GetSystemState()
{
    return m_systemState;
}

void ElevatorSubsystem::SetControlMode(const ControlMode mode)
{
    m_controlMode = mode;
    m_wantedState = WantedState::STAND_BY;
    m_systemState = SystemState::IDLE;
    m_rateLimiter.Reset();
    m_output = elevatorConstants::Speed::REST;
}

ControlMode ElevatorSubsystem::GetControlMode()
{
    return m_controlMode;
}

bool ElevatorSubsystem::IsResting()
{  
    DEBUG_ASSERT(ALLOWS_STATE_MACHINE(m_controlMode), "Straffer : IsResting() is used while Open Loop");
    return ((m_systemState == SystemState::AT_HOME) ||
            (m_systemState == SystemState::AT_L1) ||
            (m_systemState == SystemState::AT_L2) ||
            (m_systemState == SystemState::AT_L3) ||
            (m_systemState == SystemState::AT_L4) ||
            (m_systemState == SystemState::AT_STATION) ||
            (m_systemState == SystemState::AT_VISION));
}

void ElevatorSubsystem::SetOutputInOpenLoop(const double dutyCycle)
{
    if(m_controlMode == ControlMode::OPEN_LOOP)
    {
        DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
            , "Elevator Duty Cycle out of range");
        m_output = m_rateLimiter.Update((std::sin(dutyCycle * (M_PI / 2.0)) / elevatorConstants::OPEN_LOOP_REDUC) );
    }
    else 
    {
        DEBUG_ASSERT(false, "Elevator : Open Loop Output set while Closed Loop is used");
    }
}

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic()
{
    m_currentWantedState = m_wantedState;
    m_timestamp = frc::Timer::GetFPGATimestamp().value();

    m_pElevatorIO->UpdateInputs(inputs);
    m_leftMotorDisconnected.Set(!inputs.isLeftMotorConnected);
    m_rightMotorDisconnected.Set(!inputs.isRightMotorConnected);
    m_leftMotorHot.Set(inputs.leftMotorTemperature > elevatorConstants::Motors::Left::HOT_THRESHOLD);
    m_rightMotorHot.Set(inputs.rightMotorTemperature > elevatorConstants::Motors::Right::HOT_THRESHOLD);
    m_leftMotorOverheating.Set(inputs.leftMotorTemperature > elevatorConstants::Motors::Left::OVERHEATING_THRESHOLD);
    m_rightMotorOverheating.Set(inputs.rightMotorTemperature > elevatorConstants::Motors::Right::OVERHEATING_THRESHOLD);

    if(!m_isInitialized)
    {
        if(m_currentWantedState == WantedState::INITIALIZATION)
        {
            m_output = elevatorConstants::Speed::CALIBRATION;
        }
    }
    else 
    {
        if(ALLOWS_STATE_MACHINE(m_controlMode))
        {
            RunStateMachine();
        }

        switch (m_controlMode) //actualise motion
        {
        case ControlMode::POSITION_PID :
            switch (m_systemState)
            {
            //HACK : same behaviour for steady and transition state to ensure PID stability
            case SystemState::MOVING_TO_HOME :
            case SystemState::AT_HOME :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::HOME,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_HOME
            case SystemState::MOVING_TO_STATION :
            case SystemState::AT_STATION :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::CORAL_STATION,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_STATION
            case SystemState::MOVING_TO_VISION : 
            case SystemState::AT_VISION :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::VISION,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_VISION           
            case SystemState::MOVING_TO_L1 :
            case SystemState::AT_L1 :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::L1,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_L1
            case SystemState::MOVING_TO_L2 :
            case SystemState::AT_L2 :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::L2,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_L2
            case SystemState::MOVING_TO_L3 :
            case SystemState::AT_L3 :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::L3,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_L3
            case SystemState::MOVING_TO_L4 :
            case SystemState::AT_L4 :
                m_output = m_elevatorPIDController.CalculateWithRealTime(elevatorConstants::Setpoint::L4,
                                                                        inputs.heightPosition,
                                                                        m_timestamp);
                break; //end of SystemState::MOVING_TO_L4    

            case SystemState::IDLE :
                m_output = elevatorConstants::Speed::REST;
                break;      
            default:
                DEBUG_ASSERT(false, "Elevator : impossible state");
                break;
            }
            break; //end of ControlMode::POSITION_PID
        case ControlMode::OPEN_LOOP :
            //look at void SetOutputInOpenLoop(const double dutyCycle)
            break; //end of ControlMode::OPEN_LOOP

        case ControlMode::PROFILED_PID :
        case ControlMode::MOTION_PROFILING :
            //TODO later
            break; //end of Motion Profiling
        default:
            DEBUG_ASSERT(false, "Elevator : impossible state");
            break;
        }
    }


     // ----------------- Limits -----------------
    if(inputs.limitSwitchBottom || inputs.limitSwitchBottom2)
    {
        m_output = NMAX(0.0, m_output); // prevent the elevator to go through the bottom
        m_rateLimiter.Reset(); // prevent the "rate Limiter's inertia" to go through the left side
        if(!m_isEncoderAlreadyReset)
        {
            m_pElevatorIO->ResetPosition();
            m_isEncoderAlreadyReset = true; // prevent the encoder to reset many times
            if(!m_isInitialized)
            {
                m_isInitialized = true;
                m_wantedState = WantedState::STAND_BY;
                m_currentWantedState = WantedState::STAND_BY;
            }
        }
    }
    else if(inputs.heightPosition > elevatorConstants::Settings::TOP_LIMIT)
    {
        m_output = NMIN(0.0, m_output); // prevent the straffer to go through the right side
        m_rateLimiter.Reset(); // prevent the "rate Limiter's inertia" to go through the right side
    }
    else
    {
        //IFBUG : add slowed zones on bottom and top sides if the move is too brutal .(eg : If zones -> speed/2.0) ?
        m_isEncoderAlreadyReset = false;
    }
    m_pElevatorIO->SetDutyCycle(m_output);



        //LOG
    frc::SmartDashboard::PutNumber("E.WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("E.SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("E.ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutNumber("E.Setpoint", m_elevatorPIDController.GetSetpoint());
    frc::SmartDashboard::PutBoolean("E.isInit", m_isInitialized);
}

void ElevatorSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::L1 :
        m_systemState = SystemState::MOVING_TO_L1;
        break; //end of WantedState::L1
    case WantedState::L2 :
        m_systemState = SystemState::MOVING_TO_L2;
        break;  //end of WantedState::L2
    case WantedState::L3 :
        m_systemState = SystemState::MOVING_TO_L3;
        break;  //end of WantedState::L3
    case WantedState::L4 : 
        m_systemState = SystemState::MOVING_TO_L4;
        break;  //end of WantedState::L4
    case WantedState::CORAL_STATION : 
        m_systemState = SystemState::MOVING_TO_STATION;
        break;  //end of WantedState::CORAL_STATION
    case WantedState::HOME : 
        m_systemState = SystemState::MOVING_TO_HOME;
        break;  //end of WantedState::HOME
    case WantedState::VISION_POSITION :
        m_systemState = SystemState::MOVING_TO_VISION;
        break;  //end of WantedState::VISION_POSITION
    case WantedState::INITIALIZATION :
    case WantedState::STAND_BY :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "Elevator : impossible state");
        break;
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
            //TODO : add verif steady pos
            m_systemState = SystemState::MOVING_TO_HOME;
        break; //end of SystemState::IDLE
    case SystemState::MOVING_TO_L1:
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::L1) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_L1;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_L1
    case SystemState::MOVING_TO_L2:
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::L2) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_L2;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_L2
    case SystemState::MOVING_TO_L3:
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::L3) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_L3;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_L3
    case SystemState::MOVING_TO_L4:
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::L4) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_L4;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_L4
    case SystemState::MOVING_TO_HOME:
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::HOME) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_HOME;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_HOME
    case SystemState::MOVING_TO_STATION :
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::CORAL_STATION) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_STATION;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_STATION
    case SystemState::MOVING_TO_VISION :
        if(NABS(inputs.heightPosition - elevatorConstants::Setpoint::VISION) < elevatorConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_VISION;
            m_wantedState = WantedState::STAND_BY;
            m_currentWantedState = WantedState::STAND_BY; 
        }
        break; //end of SystemState::MOVING_TO_STATION
    case SystemState::AT_L1 :
    case SystemState::AT_L2 :
    case SystemState::AT_L3 :
    case SystemState::AT_L4 :
    case SystemState::AT_STATION :
    case SystemState::AT_VISION :
    case SystemState::AT_HOME :
        break; //end of other states
    default:
        DEBUG_ASSERT(false, "Elevator : impossible state");
        break;
    }
}