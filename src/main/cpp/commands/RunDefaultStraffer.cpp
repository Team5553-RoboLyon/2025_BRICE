// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunDefaultStraffer.h"

RunDefaultStraffer::RunDefaultStraffer(Straffer *pStraffer, frc::PS4Controller *pGamepad) : m_pStraffer(pStraffer), m_pGamepad(pGamepad) {
  AddRequirements(m_pStraffer);
}

// Called when the command is initially scheduled.
void RunDefaultStraffer::Initialize() {}

// TODO : verif si protection needed avec Gripper
// Called repeatedly when this Command is scheduled to run
void RunDefaultStraffer::Execute() {
  if(m_pStraffer->GetControlMode() == ControlMode::OPEN_LOOP) 
  {
    m_pStraffer->SetJoystickInput(m_pGamepad->GetRawAxis(4)/1.5); // TODO : add settings
  }
}

// Called once the command ends or is interrupted.
void RunDefaultStraffer::End(bool interrupted) {
  m_pStraffer->SetJoystickInput(strafferConstants::Speed::REST);
}

// Returns true when the command should end.
bool RunDefaultStraffer::IsFinished() {
  return false;
}