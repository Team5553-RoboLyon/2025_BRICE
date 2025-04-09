// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MainCommand.h"

MainCommand::MainCommand(Elevator *elevator, Straffer *straffer, frc::PS5Controller *gamepad) :
                                                                                            m_pElevator(elevator),
                                                                                            m_pStraffer(straffer),
                                                                                            m_pGamepad(gamepad) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({m_pElevator, m_pStraffer});
}

// Called when the command is initially scheduled.
void MainCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void MainCommand::Execute() {

  switch(m_pElevator->GetControlMode())
  {
    case ControlMode::OPEN_LOOP :
      m_pElevator->SetJoystickInput(-m_pGamepad->GetLeftY());
      break;
    default :
      break;
  }

  switch (m_pStraffer->GetControlMode())
  {
  case ControlMode::OPEN_LOOP:
    m_pStraffer->SetJoystickInput(m_pGamepad->GetRightX());
    break;
  default:
    break;
  }
}

// Called once the command ends or is interrupted.
void MainCommand::End(bool interrupted) {
}

// Returns true when the command should end.
bool MainCommand::IsFinished() {
}
