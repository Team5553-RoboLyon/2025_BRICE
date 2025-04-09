// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetStageCommand.h"

SetStageCommand::SetStageCommand(Elevator *elevator, Stage stage) : m_pElevator(elevator), m_stage(stage) {
}

// Called when the command is initially scheduled.
void SetStageCommand::Initialize() {
  m_pElevator->SetControlMode(ControlMode::CLOSED_LOOP);
  m_pElevator->SetDesiredStage(m_stage);
}

// Called repeatedly when this Command is scheduled to run
void SetStageCommand::Execute() {
}

// Called once the command ends or is interrupted.
void SetStageCommand::End(bool interrupted) {
  //TODO : faire gaffe au gripper
  m_pElevator->SetDesiredStage(Stage::HOME);
}

// Returns true when the command should end.
bool SetStageCommand::IsFinished() {
  if(m_pElevator->GetControlMode() == ControlMode::OPEN_LOOP)
    return true;
  else 
    return false;
}