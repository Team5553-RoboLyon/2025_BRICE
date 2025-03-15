// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TakeCoral.h"

TakeCoral::TakeCoral(Gripper *gripper) :  m_gripper{gripper} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({gripper});
}

// Called when the command is initially scheduled.
void TakeCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TakeCoral::Execute() {
  m_gripper->SetIntakeSpeed(GrippperConstants::Speed::CATCH);
}

// Called once the command ends or is interrupted.
void TakeCoral::End(bool interrupted) {
  m_gripper->SetIntakeSpeed(GrippperConstants::Speed::REST);
}

// Returns true when the command should end.
bool TakeCoral::IsFinished() {
  return false;
}
