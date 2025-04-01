// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DropCoralCommand.h"

DropCoralCommand::DropCoralCommand(Outtake* outtake, double openSpeed) : m_outtake(outtake), m_openSpeed(openSpeed) {
  AddRequirements({m_outtake});
}

// Called when the command is initially scheduled.
void DropCoralCommand::Initialize() {
  switch (m_outtake->GetControlMode())
  {
    case ControlMode::CLOSED_LOOP:
      m_outtake->canDrop = true;
      break; // end of ControlMode::CLOSED_LOOP
    
    case ControlMode::OPEN_LOOP:
      m_outtake->SetSpeed(m_openSpeed);
      break; // end of ControlMode::OPEN_LOOP
  }
}

// Called repeatedly when this Command is scheduled to run
void DropCoralCommand::Execute() {}

// Called once the command ends or is interrupted.
void DropCoralCommand::End(bool interrupted) {
  switch (m_outtake->GetControlMode())
  {
    case ControlMode::CLOSED_LOOP:
      m_outtake->canDrop = false;  
      break; // end of ControlMode::CLOSED_LOOP
    
    case ControlMode::OPEN_LOOP:
      m_outtake->SetSpeed(outtakeConstants::Speed::REST);
      break; // end of ControlMode::OPEN_LOOP
  }
}

// Returns true when the command should end.
bool DropCoralCommand::IsFinished() {
  return m_outtake->canDrop;
}
