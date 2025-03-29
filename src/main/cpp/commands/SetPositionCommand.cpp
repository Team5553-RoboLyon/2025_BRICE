// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetPositionCommand.h"

SetPositionCommand::SetPositionCommand(Elevator *elevator, Stage stage) : p_elevator(elevator), m_stage(stage) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({p_elevator});
}

// Called when the command is initially scheduled.
void SetPositionCommand::Initialize() {
  p_elevator->SetControlMode(DriveMode::CLOSED_LOOP);
  p_elevator->SetDesiredStage(m_stage);
}

// Called repeatedly when this Command is scheduled to run
void SetPositionCommand::Execute() {}

// Called once the command ends or is interrupted.
void SetPositionCommand::End(bool interrupted) {
  p_elevator->SetDesiredHeight(p_elevator->GetHeight());
}

// Returns true when the command should end.
bool SetPositionCommand::IsFinished() {
  return p_elevator->IsAtDesiredStage();
}
