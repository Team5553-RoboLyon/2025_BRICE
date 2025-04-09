// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignStrafferCommand.h"

AlignStrafferCommand::AlignStrafferCommand(Straffer * straffer, Side side)  : m_pStraffer(straffer), m_side(side){
}

// Called when the command is initially scheduled.
void AlignStrafferCommand::Initialize() {
  //TODO : add verif with Gripper
  m_pStraffer->SetControlMode(ControlMode::CLOSED_LOOP);
  m_pStraffer->SetDesiredSide(m_side);
}

// Called repeatedly when this Command is scheduled to run
void AlignStrafferCommand::Execute() {}

// Called once the command ends or is interrupted.
void AlignStrafferCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AlignStrafferCommand::IsFinished() {
  return false;
}
