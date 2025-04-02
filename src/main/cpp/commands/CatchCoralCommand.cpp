// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CatchCoralCommand.h"

CatchCoralCommand::CatchCoralCommand(Outtake* outtake, double openSpeed) : m_outtake(outtake), m_openSpeed(openSpeed) {
  AddRequirements({m_outtake});
}

// Called when the command is initially scheduled.
void CatchCoralCommand::Initialize() {
  switch (m_outtake->GetControlMode())
  {
    case ControlMode::CLOSED_LOOP:
      m_outtake->AskToCatch();
      break; // end of ControlMode::CLOSED_LOOP
    
    case ControlMode::OPEN_LOOP:
      m_outtake->SetSpeed(m_openSpeed);
      break; // end of ControlMode::OPEN_LOOP
  }
}

// Called repeatedly when this Command is scheduled to run
void CatchCoralCommand::Execute() {}

// Called once the command ends or is interrupted.
void CatchCoralCommand::End(bool interrupted) {
  switch (m_outtake->GetControlMode())
  {
    case ControlMode::CLOSED_LOOP:
      m_outtake->AskToCatch();
      break; // end of ControlMode::CLOSED_LOOP
    
    case ControlMode::OPEN_LOOP:
      m_outtake->SetSpeed(outtakeConstants::Speed::REST);
      break; // end of ControlMode::OPEN_LOOP
  }
}

// Returns true when the command should end.
bool CatchCoralCommand::IsFinished() {
  return m_outtake->IsCaught();
}
