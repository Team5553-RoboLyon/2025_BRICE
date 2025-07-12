// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetWantedSuperStateCmd.h"

SetWantedSuperStateCmd::SetWantedSuperStateCmd(Superstructure *pSuperStructure, Superstructure::WantedSuperState wantedState)
    : m_pSuperstructure(pSuperStructure), m_wantedState(wantedState) {
  AddRequirements({m_pSuperstructure});
}

// Called when the command is initially scheduled.
void SetWantedSuperStateCmd::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void SetWantedSuperStateCmd::Execute() {
    m_pSuperstructure->SetWantedSuperState(m_wantedState);
}

// Called once the command ends or is interrupted.
void SetWantedSuperStateCmd::End(bool interrupted) {
  m_pSuperstructure->SetWantedSuperState(Superstructure::WantedSuperState::STAND_BY);
  
}

// Returns true when the command should end.
bool SetWantedSuperStateCmd::IsFinished() {
  switch (m_wantedState)
  {
  case Superstructure::WantedSuperState::STAND_BY:
    return true;
    break;

  case Superstructure::WantedSuperState::SCORE: 
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::RETURNING_TO_HOME_EMPTY)
      return true;
    break;
  
  case Superstructure::WantedSuperState::COLLECT:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::AT_STATION_COLLECTED)
      return true;
    break;
  
  case Superstructure::WantedSuperState::TOGGLE:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::AT_HOME_COLLECTED ||
       m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::AT_STATION_COLLECTED ||
       m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L1 ||
       m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L2 ||
       m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L3 ||
       m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L4)
      return true;
    break;
  
  case Superstructure::WantedSuperState::MOVE_TO_STATION:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_COLLECT)
      return true;
    break;
  
  case Superstructure::WantedSuperState::MOVE_TO_HOME:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::AT_HOME_EMPTY ||
       m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::AT_HOME_COLLECTED)
      return true;
    break;

  case Superstructure::WantedSuperState::ALIGN_L1:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L1)
      return true;
    break;
  case Superstructure::WantedSuperState::ALIGN_L2:
  case Superstructure::WantedSuperState::ALIGN_L2_A:
  case Superstructure::WantedSuperState::ALIGN_L2_B:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L2)
      return true;
    break;
  case Superstructure::WantedSuperState::ALIGN_L3:
  case Superstructure::WantedSuperState::ALIGN_L3_A:
  case Superstructure::WantedSuperState::ALIGN_L3_B:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L3)
      return true;
    break;
  case Superstructure::WantedSuperState::ALIGN_L4:
  case Superstructure::WantedSuperState::ALIGN_L4_A:
  case Superstructure::WantedSuperState::ALIGN_L4_B:
    if(m_pSuperstructure->GetSystemSuperState() == Superstructure::SystemSuperState::READY_TO_SCORE_AT_L4)
      return true;
    break;

  default:
    DEBUG_ASSERT(false, "Cmd : impossible state");
    break;
  }

  return false;
}