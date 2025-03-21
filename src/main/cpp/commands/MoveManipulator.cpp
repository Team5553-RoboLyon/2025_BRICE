// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveManipulator.h"

MoveManipulator::MoveManipulator(Manipulator *manipulator, std::function<double()> elevator, std::function<double()> planetary) : m_manipulator(manipulator), m_elevator(elevator), m_planetary(planetary) 
{
  AddRequirements({m_manipulator});
}

// Called when the command is initially scheduled.
void MoveManipulator::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void MoveManipulator::Execute() {
  
  m_elevatorSpeed = -m_elevator();
  m_planetarySpeed = m_planetary();
  m_manipulator->Move(m_elevatorSpeed, m_planetarySpeed);

}

// Called once the command ends or is interrupted.
void MoveManipulator::End(bool interrupted) {
}

// Returns true when the command should end.
bool MoveManipulator::IsFinished() {
  return false;
}