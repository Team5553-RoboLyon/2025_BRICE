// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveElevatorToLevel.h"

MoveElevatorToLevel::MoveElevatorToLevel(Elevator *elevator, u_int16_t level) : m_elevator(elevator), m_level(level) {
  AddRequirements({m_elevator});
}

// Called when the command is initially scheduled.
void MoveElevatorToLevel::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MoveElevatorToLevel::Execute() {
  m_elevator->SelectWantedStage(m_level);
}

// Called once the command ends or is interrupted.
void MoveElevatorToLevel::End(bool interrupted) {}

// Returns true when the command should end.
bool MoveElevatorToLevel::IsFinished() {
  return false;
}
