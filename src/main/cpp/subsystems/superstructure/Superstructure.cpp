#include "subsystems/superstructure/Superstruture.h"

#include "frc/smartdashboard/SmartDashboard.h"
Superstructure::Superstructure(StrafferSubsystem *pStrafferSubsystem,
                                 ElevatorSubsystem *pElevatorSubsystem,
                                 GripperSubsystem *pGripperSubsystem) 
    : m_pStrafferSubsystem(pStrafferSubsystem),
      m_pElevatorSubsystem(pElevatorSubsystem),
      m_pGripperSubsystem(pGripperSubsystem)
{ 
};

Superstructure::Superstructure(StrafferSubsystem *pStrafferSubsystem,
                                 ElevatorSubsystem *pElevatorSubsystem,
                                 GripperSubsystem *pGripperSubsystem, 
                                 double *pElevatorAxis,
                                 double *pStrafferAxis,
                                 double *pGripperAxis) 
    : m_pStrafferSubsystem(pStrafferSubsystem),
      m_pElevatorSubsystem(pElevatorSubsystem),
      m_pGripperSubsystem(pGripperSubsystem),
      m_pElevatorAxis(pElevatorAxis),
      m_pStrafferAxis(pStrafferAxis),
      m_pGripperAxis(pGripperAxis)
{ 
};

void Superstructure::SetWantedSuperState(const WantedSuperState wantedState)
{
    if(wantedState == WantedSuperState::INITIALIZATION)
    {
        if(!m_isInitialized) // Skip initialization if the robot is already initialized
            m_wantedSuperState = WantedSuperState::INITIALIZATION; 
    }
    else // if(wantedState != WantedState::INITIALIZATION)
    {
        m_wantedSuperState = wantedState;
    }
}

Superstructure::SystemSuperState Superstructure::GetSystemSuperState()
{
    return m_systemSuperState;
}

void Superstructure::SetAssistMode(bool alignAssist, bool shootAssist)
{
    m_alignAssistEnabled = alignAssist;
    m_shootAssistEnabled = shootAssist;
}
void Superstructure::ToggleAssistMode()
{
    m_alignAssistEnabled = !m_alignAssistEnabled;
    m_shootAssistEnabled = !m_shootAssistEnabled;
}
void Superstructure::ToggleAlignAssist()
{
    m_alignAssistEnabled = !m_alignAssistEnabled;
}
void Superstructure::ToggleShootAssist()
{
    m_shootAssistEnabled = !m_shootAssistEnabled;
}

void Superstructure::Periodic() 
{
    m_currentWantedSuperState = m_wantedSuperState;
    frc::SmartDashboard::PutNumber("WantedSuperState", (int)m_currentWantedSuperState);
    frc::SmartDashboard::PutNumber("SystemSuperState", (int)m_systemSuperState);

    if(m_currentWantedSuperState == WantedSuperState::INITIALIZATION)
    {
        if(m_pElevatorSubsystem->IsInitialized() &&
           m_pStrafferSubsystem->IsInitialized())
        {
            m_isInitialized = true;
            m_wantedSuperState = WantedSuperState::STAND_BY;
            m_currentWantedSuperState = WantedSuperState::STAND_BY;
        }
        else
        {
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::INITIALIZATION);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::INITIALIZATION);
        }

    }
    else
    {
        RunSuperStateMachine();

        switch (m_systemSuperState) // Act on the Subsystems
        {
        case SystemSuperState::PREPARING_TO_COLLECT :
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::CORAL_STATION);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::GO_TO_STATION);
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::STAND_BY);
            break;
        case SystemSuperState::PREPARING_TO_SCORE :
            //HACK : Straffer is set a single time to avoid multiple SEEKING_APRIL_TAG calls in RunSuperStateMachine()
            switch (m_currentWantedSuperState)
            {
            case WantedSuperState::ALIGN_L1 :
                m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::L1);
                break;
            case WantedSuperState::ALIGN_L2 :
            case WantedSuperState::ALIGN_L2_A :
            case WantedSuperState::ALIGN_L2_B :
                m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::L2);
                break;
            case WantedSuperState::ALIGN_L3 :
            case WantedSuperState::ALIGN_L3_A :
            case WantedSuperState::ALIGN_L3_B :
                m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::L3);
                break;
            case WantedSuperState::ALIGN_L4 :
            case WantedSuperState::ALIGN_L4_A :
            case WantedSuperState::ALIGN_L4_B :
                m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::L4);
                break;
            default:
                break;
            }
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::STAND_BY);
            break;
        case SystemSuperState::RETURNING_TO_HOME_EMPTY :
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::HOME);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::GO_TO_STATION);
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::STAND_BY);
            break;
        case SystemSuperState::RETURNING_TO_HOME_COLLECTED :
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::HOME);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::GO_TO_STATION);
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::STAND_BY);
            break;
        case SystemSuperState::COLLECTING :
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::LOAD);
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::STAND_BY);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::STAND_BY);
            break;
        case SystemSuperState::SCORING :
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::STAND_BY);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::STAND_BY);
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L1) 
            {
                m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::SCORE_LOW);
            }
            else if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L4) 
            {
                m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::SCORE_HIGH);
            }
            else if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L2 ||
                    m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L3) 
            {
                m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::SCORE_MIDDLE);
            }
            break;
        case SystemSuperState::TOGGLING :
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::STAND_BY);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::STAND_BY);
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::TOGGLE);
            break;
        case SystemSuperState::READY_TO_COLLECT :
        case SystemSuperState::READY_TO_SCORE :
        case SystemSuperState::AT_HOME_EMPTY :
        case SystemSuperState::AT_HOME_COLLECTED :
        case SystemSuperState::AT_STATION_COLLECTED :
            m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::STAND_BY);
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::STAND_BY);
            m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::STAND_BY);
            break;
        
        case SystemSuperState::IDLE:
            if(m_isInitialized)
            {
                m_pElevatorSubsystem->SetWantedState(ElevatorSubsystem::WantedState::HOME);
                m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::GO_TO_STATION);
                m_pGripperSubsystem->SetWantedState(GripperSubsystem::WantedState::STAND_BY);
            }
        default:
            break;
        }
    }
}


void Superstructure::RunSuperStateMachine()
{
    switch (m_currentWantedSuperState) //Handle Super State transition
    {
    case WantedSuperState::STAND_BY :
        if(m_systemSuperState == SystemSuperState::COLLECTING)
        {
            m_systemSuperState = SystemSuperState::READY_TO_SCORE;
        }
        break;
    
    case WantedSuperState::SCORE :
        if(m_systemSuperState == SystemSuperState::READY_TO_SCORE)
        {
            m_systemSuperState = SystemSuperState::SCORING;
        }
        break;
    case WantedSuperState::COLLECT :
        if(m_systemSuperState == SystemSuperState::READY_TO_COLLECT)
        {
            m_systemSuperState = SystemSuperState::COLLECTING;
        }
        break;
    case WantedSuperState::TOGGLE :
        if(m_systemSuperState == SystemSuperState::AT_STATION_COLLECTED ||
           m_systemSuperState == SystemSuperState::READY_TO_SCORE ||
            m_systemSuperState == SystemSuperState::AT_HOME_COLLECTED)
        {
            m_systemSuperState = SystemSuperState::TOGGLING;
        }
        break;
    case WantedSuperState::MOVE_TO_STATION :
        if(m_systemSuperState == SystemSuperState::AT_HOME_EMPTY ||
            m_systemSuperState == SystemSuperState::RETURNING_TO_HOME_EMPTY)
        {
            m_systemSuperState = SystemSuperState::PREPARING_TO_COLLECT;
        }
        break;
    case WantedSuperState::MOVE_TO_HOME :
        if(m_systemSuperState == SystemSuperState::READY_TO_COLLECT ||
           m_systemSuperState == SystemSuperState::PREPARING_TO_COLLECT)
        {
            m_systemSuperState = SystemSuperState::RETURNING_TO_HOME_EMPTY;
        }
        else if(m_systemSuperState == SystemSuperState::READY_TO_SCORE ||
                m_systemSuperState == SystemSuperState::PREPARING_TO_SCORE ||
                m_systemSuperState == SystemSuperState::AT_STATION_COLLECTED)
        {
            m_systemSuperState = SystemSuperState::RETURNING_TO_HOME_COLLECTED;
        }
        break;
    case WantedSuperState::ALIGN_L1 :
    case WantedSuperState::ALIGN_L2 :
    case WantedSuperState::ALIGN_L3 :
    case WantedSuperState::ALIGN_L4 :
        if(m_systemSuperState == SystemSuperState::AT_HOME_COLLECTED ||
           m_systemSuperState == SystemSuperState::AT_STATION_COLLECTED ||
           m_systemSuperState == SystemSuperState::PREPARING_TO_SCORE ||
           m_systemSuperState == SystemSuperState::READY_TO_SCORE ||
           m_systemSuperState == SystemSuperState::RETURNING_TO_HOME_COLLECTED) 
        {
            m_systemSuperState = SystemSuperState::PREPARING_TO_COLLECT;
            if(m_alignAssistEnabled)
            {
                m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::AUTO_ALIGN);
            }
        }
        break;  
    case WantedSuperState::ALIGN_L2_A :
    case WantedSuperState::ALIGN_L3_A :
    case WantedSuperState::ALIGN_L4_A :
        if(m_systemSuperState == SystemSuperState::AT_HOME_COLLECTED ||
           m_systemSuperState == SystemSuperState::AT_STATION_COLLECTED ||
           m_systemSuperState == SystemSuperState::PREPARING_TO_SCORE ||
           m_systemSuperState == SystemSuperState::READY_TO_SCORE ||
           m_systemSuperState == SystemSuperState::RETURNING_TO_HOME_COLLECTED) 
        {
            m_systemSuperState = SystemSuperState::PREPARING_TO_COLLECT;
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::ALIGN_LEFT_REEF);
        }
        break;  
    case WantedSuperState::ALIGN_L2_B :
    case WantedSuperState::ALIGN_L3_B :
    case WantedSuperState::ALIGN_L4_B :
        if(m_systemSuperState == SystemSuperState::AT_HOME_COLLECTED ||
           m_systemSuperState == SystemSuperState::AT_STATION_COLLECTED ||
           m_systemSuperState == SystemSuperState::PREPARING_TO_SCORE ||
           m_systemSuperState == SystemSuperState::READY_TO_SCORE ||
           m_systemSuperState == SystemSuperState::RETURNING_TO_HOME_COLLECTED) 
        {
            m_systemSuperState = SystemSuperState::PREPARING_TO_COLLECT;
            m_pStrafferSubsystem->SetWantedState(StrafferSubsystem::WantedState::ALIGN_RIGHT_REEF);
        }
        break;
    default:
        break;
    }

    switch (m_systemSuperState) // Change System Super State
    {
    case SystemSuperState::IDLE:
        if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_HOME &&
           m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_STATION)
        {
            if(m_pGripperSubsystem->GetSystemState() == GripperSubsystem::SystemState::REST_EMPTY)
            {
                m_systemSuperState = SystemSuperState::AT_HOME_EMPTY;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            else if(m_pGripperSubsystem->GetSystemState() == GripperSubsystem::SystemState::REST_LOADED)
            {
                m_systemSuperState = SystemSuperState::AT_HOME_COLLECTED;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
        }
        break;
    
    case SystemSuperState::READY_TO_SCORE:
        if(m_shootAssistEnabled)
        {
            m_systemSuperState = SystemSuperState::SCORING;
        }
        break;
    case SystemSuperState::AT_HOME_EMPTY:
    case SystemSuperState::AT_HOME_COLLECTED:
    case SystemSuperState::AT_STATION_COLLECTED:
    case SystemSuperState::READY_TO_COLLECT:
        break; //steady states, no transition
    case SystemSuperState::PREPARING_TO_COLLECT:
        if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_STATION &&
           m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_STATION)
        {
            m_systemSuperState = SystemSuperState::READY_TO_COLLECT;
            m_wantedSuperState = WantedSuperState::STAND_BY;
            m_currentWantedSuperState = WantedSuperState::STAND_BY;  
        }
        break;
    case SystemSuperState::PREPARING_TO_SCORE:
        switch (m_currentWantedSuperState)
        {
        case WantedSuperState::ALIGN_L1:
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L1 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_STATION)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L2:
            if((m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_LEFT_REEF ||
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_RIGHT_REEF) 
               && m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L2)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L3:
            if((m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_LEFT_REEF ||
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_RIGHT_REEF) 
               && m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L3)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L4:
            if((m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_LEFT_REEF ||
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_RIGHT_REEF) 
               && m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L4)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L2_A:
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L2 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_LEFT_REEF)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L2_B:
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L2 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_RIGHT_REEF)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L3_A:
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L3 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_LEFT_REEF)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L3_B:
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L3 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_RIGHT_REEF)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;  
            }
            break;
        case WantedSuperState::ALIGN_L4_A:
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L4 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_LEFT_REEF)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;
            }
            break;
        case WantedSuperState::ALIGN_L4_B: 
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_L4 &&
               m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_RIGHT_REEF)
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;
            }
            break;
        default:
            break;
        }
        break;
    
    case SystemSuperState::RETURNING_TO_HOME_EMPTY:
        if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_HOME &&
           m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_STATION)
        {
            m_systemSuperState = SystemSuperState::AT_HOME_EMPTY;
            m_wantedSuperState = WantedSuperState::STAND_BY;
            m_currentWantedSuperState = WantedSuperState::STAND_BY;
        }
        break;
    case SystemSuperState::RETURNING_TO_HOME_COLLECTED:
        if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_HOME &&
           m_pStrafferSubsystem->GetSystemState() == StrafferSubsystem::SystemState::AT_STATION)
        {
            m_systemSuperState = SystemSuperState::AT_HOME_COLLECTED;
            m_wantedSuperState = WantedSuperState::STAND_BY;
            m_currentWantedSuperState = WantedSuperState::STAND_BY;
        }
        break;
    case SystemSuperState::COLLECTING:
        if(m_pGripperSubsystem->GetSystemState() == GripperSubsystem::SystemState::REST_LOADED)
        {
            m_systemSuperState = SystemSuperState::AT_STATION_COLLECTED;
            m_wantedSuperState = WantedSuperState::STAND_BY;
            m_currentWantedSuperState = WantedSuperState::STAND_BY;
        }
        break;
    case SystemSuperState::SCORING:
        if(m_pGripperSubsystem->GetSystemState() == GripperSubsystem::SystemState::REST_EMPTY)
        {
            m_systemSuperState = SystemSuperState::RETURNING_TO_HOME_EMPTY;
            m_wantedSuperState = WantedSuperState::STAND_BY;
            m_currentWantedSuperState = WantedSuperState::STAND_BY;
        }
        break;
    case SystemSuperState::TOGGLING:
        if(m_pGripperSubsystem->GetSystemState() == GripperSubsystem::SystemState::REST_LOADED)
        {
            if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_HOME)
            {
                m_systemSuperState = SystemSuperState::AT_HOME_COLLECTED;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;
            }
            else if(m_pElevatorSubsystem->GetSystemState() == ElevatorSubsystem::SystemState::AT_STATION)
            {
                m_systemSuperState = SystemSuperState::AT_STATION_COLLECTED;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;
            }
            else
            {
                m_systemSuperState = SystemSuperState::READY_TO_SCORE;
                m_wantedSuperState = WantedSuperState::STAND_BY;
                m_currentWantedSuperState = WantedSuperState::STAND_BY;
            }
        }
        break;
    default:
        break;
    }
}


