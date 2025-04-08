// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Command.h"

Command::Command(Straffer *straffer, std::function<double()> joystick) : 
    m_pStraffer(straffer),
    joystickInput(joystick) {
  AddRequirements({m_pStraffer});
}

// Called when the command is initially scheduled.
void Command::Initialize() {
  // 397
  m_pStraffer->SetControlMode(ControlMode::OPEN_LOOP);
}

// Called repeatedly when this Command is scheduled to run
void Command::Execute() {
  value = joystickInput();
    m_pStraffer->SetJoystickInput(value/1.0);
}

// Called once the command ends or is interrupted.
void Command::End(bool interrupted) {}

// Returns true when the command should end.
bool Command::IsFinished() {
  return false;
}
