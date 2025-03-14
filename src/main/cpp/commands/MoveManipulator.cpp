// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/MoveManipulator.h"

MoveManipulator::MoveManipulator(Manipulator *manipulator, ManipulatorConstants::State target, std::function<bool()> drop) : m_manipulator(manipulator), m_target(target), m_Drop(drop) 
{
  AddRequirements({m_manipulator});
}

// Called when the command is initially scheduled.
void MoveManipulator::Initialize() {
    m_manipulator->SelectWantedStage(m_target);
    // std::cout << (int)m_target << std::endl;
}

// Called repeatedly when this Command is scheduled to run
void MoveManipulator::Execute() {
  m_manipulator->gripperActivated = m_Drop();
//   std::cout << m_Drop() << std::endl;
}

// Called once the command ends or is interrupted.
void MoveManipulator::End(bool interrupted) {
    m_manipulator->SelectWantedStage(ManipulatorConstants::State::Home);
    m_manipulator->gripperActivated = false;
}

// Returns true when the command should end.
bool MoveManipulator::IsFinished() {
  return false;
}