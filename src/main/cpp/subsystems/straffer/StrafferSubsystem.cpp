#include "subsystems/straffer/StrafferSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "lib/TimerRBL.h"
#include "lib/DebugUtils.h"

//FIXME : implement straffer length in SystemState::SEEKING_APRIL_TAG
StrafferSubsystem::StrafferSubsystem(StrafferIO *pIo, Camera *pCamera) : 
                                                    m_pStrafferIO(pIo),
                                                    m_pCamera(pCamera)
{
    if(m_controlMode == ControlMode::POSITION_DUTYCYCLE_PID)
    {
        m_strafferPIDController.SetGains(strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KP, 
                                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KI, 
                                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KD);
        m_strafferPIDController.SetTolerance(strafferConstants::Gains::POSITION_DUTYCYCLE_PID::TOLERANCE);
    }
    else if(m_controlMode == ControlMode::MANUAL_SETPOINT)
    {
        m_strafferPIDController.SetGains(strafferConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                strafferConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                strafferConstants::Gains::MANUAL_SETPOINT_PID::KD);
        m_strafferPIDController.SetTolerance(strafferConstants::Gains::MANUAL_SETPOINT_PID::TOLERANCE);
    }
    m_strafferPIDController.Reset(m_timestamp);
    m_strafferPIDController.SetOutputLimits(strafferConstants::Speed::MIN, strafferConstants::Speed::MAX);
    m_strafferPIDController.SetInputLimits(true);
    m_strafferPIDController.SetInputLimits(strafferConstants::Settings::LEFT_LIMIT, strafferConstants::Settings::RIGHT_LIMIT);

}
void StrafferSubsystem::SetControlMode(const ControlMode mode)
{
    m_controlMode = mode;
    m_wantedState = WantedState::STAND_BY;
    m_systemState = SystemState::IDLE;
    m_rateLimiter.Reset();
    m_strafferPIDController.Reset(m_timestamp);
    if(m_controlMode == ControlMode::POSITION_DUTYCYCLE_PID)
    {
        m_strafferPIDController.SetGains(strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KP, 
                                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KI, 
                                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KD);
        m_strafferPIDController.SetTolerance(strafferConstants::Gains::POSITION_DUTYCYCLE_PID::TOLERANCE);
    }
    else if(m_controlMode == ControlMode::MANUAL_SETPOINT)
    {
        m_strafferPIDController.SetGains(strafferConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                strafferConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                strafferConstants::Gains::MANUAL_SETPOINT_PID::KD);
        m_strafferPIDController.SetTolerance(strafferConstants::Gains::MANUAL_SETPOINT_PID::TOLERANCE);
    }
    m_output = strafferConstants::Speed::REST;
    m_manualControlInput = 0.0;
}
ControlMode StrafferSubsystem::GetControlMode()
{
    return m_controlMode;
}
void StrafferSubsystem::ToggleControlMode()
{
    m_wantedState = WantedState::STAND_BY;
    m_systemState = SystemState::IDLE;
    m_output = strafferConstants::Speed::REST;
    m_manualControlInput = 0.0;
    m_rateLimiter.Reset();
    m_strafferPIDController.Reset(m_timestamp);
    switch (m_controlMode)
    {
    case strafferConstants::MainControlMode :
        m_controlMode = strafferConstants::EmergencyControlMode;
        break;
    case strafferConstants::EmergencyControlMode : 
        m_controlMode = strafferConstants::MainControlMode;
        break;
    default:
        DEBUG_ASSERT(false,"Straffer : Toggle impossible with an unrecognized mode.");
        break;
    }

    if(m_controlMode == ControlMode::POSITION_DUTYCYCLE_PID)
    {
        m_strafferPIDController.SetGains(strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KP, 
                                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KI, 
                                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KD);
        m_strafferPIDController.SetTolerance(strafferConstants::Gains::POSITION_DUTYCYCLE_PID::TOLERANCE);
    }
    else if(m_controlMode == ControlMode::MANUAL_SETPOINT)
    {
        m_strafferPIDController.SetGains(strafferConstants::Gains::MANUAL_SETPOINT_PID::KP, 
                                strafferConstants::Gains::MANUAL_SETPOINT_PID::KI, 
                                strafferConstants::Gains::MANUAL_SETPOINT_PID::KD);
        m_strafferPIDController.SetTolerance(strafferConstants::Gains::MANUAL_SETPOINT_PID::TOLERANCE);
    }
}
void StrafferSubsystem::SetWantedState(const WantedState wantedState)
{
    if(wantedState == WantedState::INITIALIZATION)
    {
        if(!m_isInitialized) // Skip initialization if the subsystem is already initialized
            m_wantedState = WantedState::INITIALIZATION; 
    }
    else // if(wantedState != WantedState::INITIALIZATION)
    {
        m_wantedState = wantedState;
    }
}
StrafferSubsystem::SystemState StrafferSubsystem::GetSystemState()
{
    return m_systemState;
}
void StrafferSubsystem::SetManualAxis(const double value)
{
    if(BYPASS_STATE_MACHINE(m_controlMode))
    {
        DEBUG_ASSERT((value <= 1.0) && (value >= -1.0) 
            , "Straffer Manual value out of range");
        m_manualControlInput = value;
    }
    else 
    {
        DEBUG_ASSERT(false , "Straffer : Manual value set while StateMachine is used");
    }
}
bool StrafferSubsystem::IsResting()
{
    DEBUG_ASSERT(ALLOWS_STATE_MACHINE(m_controlMode) , "Straffer : IsResting() is used while Open Loop");
    return ((m_systemState == SystemState::AT_STATION) || 
            (m_systemState == SystemState::AT_LEFT_REEF) || 
            (m_systemState == SystemState::AT_RIGHT_REEF) ||
            (m_systemState == SystemState::AT_LEFT_SIDE) ||
            (m_systemState == SystemState::AT_RIGHT_SIDE));
}

// This method will be called once per scheduler run
void StrafferSubsystem::Periodic() 
{
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_currentWantedState = m_wantedState;

    m_pStrafferIO->UpdateInputs(inputs);
    m_logger.Log(inputs);
    m_motorDisconnected.Set(!inputs.isMotorConnected);
    m_motorOverheating.Set(inputs.temperature > strafferConstants::Motor::OVERHEATING_THRESHOLD);
    m_motorHot.Set(inputs.temperature > strafferConstants::Motor::HOT_THRESHOLD);

    if(!m_isInitialized)
    {
        if(m_currentWantedState == WantedState::INITIALIZATION)
        {
            m_output = strafferConstants::Speed::CALIBRATION;
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
        case ControlMode::POSITION_DUTYCYCLE_PID :
            switch (m_systemState)
            {
            case SystemState::AT_STATION :
            case SystemState::AT_LEFT_REEF :
            case SystemState::AT_LEFT_SIDE :
            case SystemState::AT_RIGHT_REEF :
            case SystemState::AT_RIGHT_SIDE :
            case SystemState::IDLE :
            case SystemState::SEEKING_APRIL_TAG :
                m_output = strafferConstants::Speed::REST; 
                break;
            case SystemState::STRAFFING_TO_LEFT_REEF :
            case SystemState::STRAFFING_TO_RIGHT_REEF :
                m_strafferPIDController.SetFeedforward(
                NSIGN(m_selectedReefWidthPosition - inputs.widthPosition) * 
                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KS);
                m_output = m_strafferPIDController.CalculateWithRealTime(m_selectedReefWidthPosition,
                                                                        inputs.widthPosition,
                                                                        m_timestamp);
                
                break;
            case SystemState::STRAFFING_TO_LEFT_SIDE :
                m_strafferPIDController.SetFeedforward(
                NSIGN(strafferConstants::Setpoint::LEFT_SIDE - inputs.widthPosition) * 
                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KS);
                m_output = m_strafferPIDController.CalculateWithRealTime(strafferConstants::Setpoint::LEFT_SIDE,
                                                                        inputs.widthPosition,
                                                                        m_timestamp);
                break;
            case SystemState::STRAFFING_TO_RIGHT_SIDE :
                m_strafferPIDController.SetFeedforward(
                NSIGN(strafferConstants::Setpoint::RIGHT_SIDE - inputs.widthPosition) * 
                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KS);
                m_output = m_strafferPIDController.CalculateWithRealTime(strafferConstants::Setpoint::RIGHT_SIDE,
                                                                        inputs.widthPosition,
                                                                        m_timestamp);
                break;
            case SystemState::STRAFFING_TO_STATION :
                m_strafferPIDController.SetFeedforward(
                NSIGN(strafferConstants::Setpoint::CENTER - inputs.widthPosition) * 
                strafferConstants::Gains::POSITION_DUTYCYCLE_PID::KS);
                m_output = m_strafferPIDController.CalculateWithRealTime(strafferConstants::Setpoint::CENTER,
                                                                        inputs.widthPosition,
                                                                        m_timestamp);
                break;
            default:
                DEBUG_ASSERT(false, "Straffer : impossible state");
                break;
            }
            break;
        case ControlMode::MOTION_PROFILING :
            // TODO : implement motion profiling logic here.
            break;
        case ControlMode::PROFILED_PID : 
            //TODO : later
            break;
        case ControlMode::MANUAL_DUTY_CYCLE :
            m_output = m_rateLimiter.Update(std::sin(m_manualControlInput * (M_PI / 2.0)));
            break;
        case ControlMode::MANUAL_SETPOINT :
            //adapt the manual value to changing setpoint
            m_manualControlInput = m_strafferPIDController.GetSetpoint() + m_manualControlInput * strafferConstants::Settings::MANUAL_SETPOINT_CHANGE_LIMIT;
            
            m_strafferPIDController.SetFeedforward(NSIGN(m_manualControlInput - inputs.widthPosition) * 
                                    strafferConstants::Gains::MANUAL_SETPOINT_PID::KS);
            m_output = m_strafferPIDController.CalculateWithRealTime(m_manualControlInput,
                                                                        inputs.widthPosition,
                                                                        m_timestamp);
            break;
        default:
            DEBUG_ASSERT(false , "Straffer : wrong ControlMode chosen");
            m_output = 0.0; // protection
            break;
        }
    }


    // ----------------- Limits -----------------
    if(inputs.limitSwitchLeft)
    {
        m_output = NMAX(0.0, m_output); // prevent the straffer to go through the left side
        m_rateLimiter.Reset(); // prevent the "rate Limiter's inertia" to go through the left side
        if(!m_isEncoderAlreadyReset)
        {
            m_pStrafferIO->ResetPositionLeft();
            m_isEncoderAlreadyReset = true; // prevent the encoder to reset many times
            if(!m_isInitialized)
            {
                m_isInitialized = true;
                m_currentWantedState = WantedState::STAND_BY;
            }
        }
    }
    else if(inputs.limitSwitchRight)
    {
        m_output = NMIN(0.0, m_output); // prevent the straffer to go through the right side
        m_rateLimiter.Reset(); // prevent the "rate Limiter's inertia" to go through the right side
        if(!m_isEncoderAlreadyReset)
        {
            m_pStrafferIO->ResetPositionRight();
            m_isEncoderAlreadyReset = true; // prevent the encoder to reset many times
        }
    }
    else // if(!inputs.limitSwitchLeft && !inputs.limitSwitchRight)
    {
        //IFBUG : add slowed zones on left and right sides if the move is too brutal .(eg : If zones -> speed/2.0) ?
        m_isEncoderAlreadyReset = false;
    }
    m_pStrafferIO->SetDutyCycle(m_output);





    //LOG
    frc::SmartDashboard::PutNumber("Straffer/WantedState", (int)m_currentWantedState);
    frc::SmartDashboard::PutNumber("Straffer/SystemState", (int)m_systemState);
    frc::SmartDashboard::PutNumber("Straffer/ControlMode", (int)m_controlMode);
    frc::SmartDashboard::PutNumber("Straffer/Setpoint", m_strafferPIDController.GetSetpoint());
    frc::SmartDashboard::PutBoolean("Straffer/isInit", m_isInitialized);
}

void StrafferSubsystem::RunStateMachine()
{
    switch (m_currentWantedState) //Handle State transition
    {
    case WantedState::ALIGN_LEFT_REEF :
    case WantedState::ALIGN_RIGHT_REEF :
    case WantedState::AUTO_ALIGN :
        if( (m_systemState != SystemState::SEEKING_APRIL_TAG) &&
            (m_systemState != SystemState::STRAFFING_TO_LEFT_REEF) &&
            (m_systemState != SystemState::STRAFFING_TO_RIGHT_REEF) )
        {
            m_systemState = SystemState::SEEKING_APRIL_TAG;
            m_counter = strafferConstants::Seeking::COUNTER;
            m_lowestAmbiguity = 1.0;
            m_bestAprilTagOffset = 0.0;
            // m_currentWantedState = WantedState::STAND_BY; 
            // m_wantedState = WantedState::STAND_BY;
            // //HACK : Ensures these values are set only once during each call to avoid redundant resets
        }
        break; //end of WantedState::ALIGN_TO_REEF
    
    case WantedState::GO_TO_LEFT_SIDE :
        m_systemState = SystemState::STRAFFING_TO_LEFT_SIDE;
        break; //end of WantedState::GO_TO_LEFT_SIDE
    case WantedState::GO_TO_STATION :
        m_systemState = SystemState::STRAFFING_TO_STATION;
        break; //end of WantedState::GO_TO_STATION
    case WantedState::GO_TO_RIGHT_SIDE :
        m_systemState = SystemState::STRAFFING_TO_RIGHT_SIDE;
        break; //end of WantedState::GO_TO_RIGHT_SIDE

    case WantedState::INITIALIZATION :
    case WantedState::STAND_BY :
        break; //end of Others States
    default:
        DEBUG_ASSERT(false, "Straffer : impossible state");
        break;
    }

    switch (m_systemState) // Change System State
    {
    case SystemState::IDLE:
        if(NABS(inputs.widthPosition - strafferConstants::Setpoint::LEFT_SIDE) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_LEFT_SIDE;
        }
        else if(NABS(inputs.widthPosition - strafferConstants::Setpoint::RIGHT_SIDE) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_RIGHT_SIDE;
        }
        else if(NABS(inputs.widthPosition - strafferConstants::Setpoint::CENTER) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_STATION;
        }
        else
        {
            // The current width position does not match any predefined setpoints (LEFT_SIDE, RIGHT_SIDE, CENTER).
            // This indicates an unexpected state, so we reset the system state to STRAFFING_TO_STATION
            // to ensure the subsystem moves to the center position as a safe fallback.
            m_systemState = SystemState::STRAFFING_TO_STATION;
        }
        break; //end of SystemState::IDLE
    case SystemState::STRAFFING_TO_STATION:
        if(NABS(inputs.widthPosition - strafferConstants::Setpoint::CENTER) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_STATION;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
        }
        break; //end of SystemState::STRAFFING_TO_STATION
    case SystemState::STRAFFING_TO_LEFT_SIDE:
        if(NABS(inputs.widthPosition - strafferConstants::Setpoint::LEFT_SIDE) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_LEFT_SIDE;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
        }
        break;//end of SystemState::STRAFFING_TO_LEFT_SIDE
    case SystemState::STRAFFING_TO_RIGHT_SIDE:
        if(NABS(inputs.widthPosition - strafferConstants::Setpoint::RIGHT_SIDE) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_RIGHT_SIDE;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
        }
        break;//end of SystemState::STRAFFING_TO_RIGHT_SIDE
    case SystemState::STRAFFING_TO_LEFT_REEF:
        if(NABS(inputs.widthPosition - m_selectedReefWidthPosition) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_LEFT_REEF;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
        }
        break;//end of SystemState::STRAFFING_TO_LEFT_REEF
    case SystemState::STRAFFING_TO_RIGHT_REEF:
        if(NABS(inputs.widthPosition - m_selectedReefWidthPosition) < strafferConstants::Setpoint::TOLERANCE)
        {
            m_systemState = SystemState::AT_RIGHT_REEF;
            m_currentWantedState = WantedState::STAND_BY;
            m_wantedState = WantedState::STAND_BY;
        }
        break;//end of SystemState::STRAFFING_TO_RIGHT_REEF
    case SystemState::SEEKING_APRIL_TAG :
        //TODO : rework camera's usage
        if(m_counter == 0)
        { 
            if (m_lowestAmbiguity > strafferConstants::Seeking::HIGHEST_AMBIGUITY_ACCEPTED) 
            {
                m_systemState = SystemState::STRAFFING_TO_STATION;
                m_wantedState = WantedState::STAND_BY;
                m_currentWantedState = WantedState::STAND_BY;
                CanRumble = true;
            }
            else 
            {
                double baseTarget = strafferConstants::Setpoint::CENTER - m_bestAprilTagOffset;
                double offsetSide = 0.0;

                switch (m_currentWantedState) {
                    case WantedState::ALIGN_LEFT_REEF:
                        offsetSide = strafferConstants::Seeking::LEFT_OFFSET;
                        m_systemState = SystemState::STRAFFING_TO_LEFT_REEF;
                        break;
                    case WantedState::ALIGN_RIGHT_REEF:
                        offsetSide = strafferConstants::Seeking::RIGHT_OFFSET;
                        m_systemState = SystemState::STRAFFING_TO_RIGHT_REEF;
                        break;
                    case WantedState::AUTO_ALIGN:
                        if(baseTarget >= strafferConstants::Setpoint::CENTER)
                        {
                            offsetSide = strafferConstants::Seeking::RIGHT_OFFSET;
                            m_systemState = SystemState::STRAFFING_TO_RIGHT_REEF;
                        }
                        else
                        {
                            offsetSide = strafferConstants::Seeking::LEFT_OFFSET;
                            m_systemState = SystemState::STRAFFING_TO_LEFT_REEF;
                        }
                        break;
                    default:
                        DEBUG_ASSERT(false, "Straffer : impossible state");
                        break;
                }
                m_selectedReefWidthPosition = baseTarget + offsetSide;
                if (m_selectedReefWidthPosition < strafferConstants::Settings::LEFT_LIMIT ||
                    m_selectedReefWidthPosition > strafferConstants::Settings::RIGHT_LIMIT) {
                    m_systemState = SystemState::STRAFFING_TO_STATION;
                    m_wantedState = WantedState::STAND_BY;
                    m_currentWantedState = WantedState::STAND_BY;
                    CanRumble = true;
                }
            }
        }
        else 
        {
            m_counter--;
            m_pCamera->Update();
            if(m_pCamera->HasTargets())
            {  
                photon::PhotonTrackedTarget bestTarget = m_pCamera->GetBestTarget();
                double currentAmbiguity =  m_pCamera->GetAmbiguity(bestTarget);
                if (currentAmbiguity <= m_lowestAmbiguity)
                {
                    m_lowestAmbiguity = currentAmbiguity;
                    m_bestAprilTagOffset = m_pCamera->GetHorizontalDistance(bestTarget);
                }
            }
        }
        break; //end of SystemState::SEEKING_APRIL_TAGS
    case SystemState::AT_STATION :
    case SystemState::AT_LEFT_REEF :
    case SystemState::AT_LEFT_SIDE :
    case SystemState::AT_RIGHT_REEF :
    case SystemState::AT_RIGHT_SIDE :
        break; //end of other states
    default:
        DEBUG_ASSERT(false, "SuperStructure : impossible state");
        break;
    }
}