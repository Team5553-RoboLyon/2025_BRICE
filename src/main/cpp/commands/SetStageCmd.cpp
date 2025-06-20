// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetStageCmd.h"

SetStageCmd::SetStageCmd(Elevator *pElevator, Gripper *pGripper, Stage stage) : m_pElevator(pElevator),m_pGripper(pGripper), m_WantedStage(stage) {
  AddRequirements(m_pElevator);
}

// Called when the command is initially scheduled.
void SetStageCmd::Initialize() {
  m_pElevator->SetControlMode(ControlMode::CLOSED_LOOP);
  isFinished = false;
}

// Called repeatedly when this Command is scheduled to run
void SetStageCmd::Execute() {
  if(m_pGripper->m_state == Gripper::State::REST_EMPTY || m_pGripper->m_state == Gripper::State::REST_LOADED || m_pGripper->GetControlMode() == ControlMode::OPEN_LOOP)
  {
    m_pElevator->SetDesiredStage(m_WantedStage);
    isFinished = true;
  }
}

// Called once the command ends or is interrupted.
void SetStageCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool SetStageCmd::IsFinished() {
  if(m_pElevator->GetControlMode() == ControlMode::OPEN_LOOP)
    return true;
  else {
    return isFinished;
  }
}