// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignStrafferCmd.h"

AlignStrafferCmd::AlignStrafferCmd(Straffer* pStraffer, Gripper* pGripper, Side side)  : m_pStraffer(pStraffer), m_pGripper(pGripper), m_side(side){
  AddRequirements(m_pStraffer);
}

// Called when the command is initially scheduled.
void AlignStrafferCmd::Initialize() {
  // if(m_pGripper->IsCaught() || m_pGripper->IsDropped())
  // {
    m_pStraffer->SetControlMode(ControlMode::CLOSED_LOOP);
    m_pStraffer->SetDesiredSide(m_side);
  // }
}

// Called repeatedly when this Command is scheduled to run
void AlignStrafferCmd::Execute() {}

// Called once the command ends or is interrupted.
void AlignStrafferCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool AlignStrafferCmd::IsFinished() {
  if(m_pStraffer->GetControlMode() == ControlMode::OPEN_LOOP)
    return true;
  else 
    return false;
}