// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"
#include "frc/shuffleboard/Shuffleboard.h"

Climb::Climb(DeepClimb *pclimb, std::function<double()> speed) : m_climb(pclimb), m_speed(speed) {
  AddRequirements({m_climb});
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  //add a encoder reset here
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  double consigne = m_speed();
  m_climb->SetClimbSpeed(consigne);
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  //add a encoder reset here
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return false;
} 