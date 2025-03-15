// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"

Climb::Climb(DeepClimb *deepClimb) : m_deepClimb(deepClimb){
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_deepClimb);
}

// Called when the command is initially scheduled.
void Climb::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  m_deepClimb->SetClimbSpeed(DeepClimbConstants::SPEED::CLIMB);
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) { 
  m_deepClimb->SetClimbSpeed(DeepClimbConstants::SPEED::REST);
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return false;
}
