// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DropCoral.h"

DropCoral::DropCoral(Gripper *gripper) :  m_gripper{gripper} {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({gripper});
}

// Called when the command is initially scheduled.
void DropCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DropCoral::Execute() {
  m_gripper->SetDropSpeed(GrippperConstants::Speed::DROP);
}

// Called once the command ends or is interrupted.
void DropCoral::End(bool interrupted) {
  m_gripper->SetIntakeSpeed(GrippperConstants::Speed::REST);
}

// Returns true when the command should end.
bool DropCoral::IsFinished() {
  return false;
}