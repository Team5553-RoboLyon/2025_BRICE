// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunElevator.h"

RunElevator::RunElevator(Elevator *elevator, std::function<double()> joystickInput) : p_elevator(elevator), m_joystickInput(joystickInput) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({p_elevator});
}

// Called when the command is initially scheduled.
void RunElevator::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunElevator::Execute() {
  double input = -m_joystickInput();
  p_elevator->SetJoystickInput(input);
}

// Called once the command ends or is interrupted.
void RunElevator::End(bool interrupted) {}

// Returns true when the command should end.
bool RunElevator::IsFinished() {
  if(p_elevator->GetControlMode() == DriveMode::OPEN_LOOP)
    return false;
  else
    return true;
}