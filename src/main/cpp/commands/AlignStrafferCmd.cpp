// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignStrafferCmd.h"

static double offsetStrafferLUT[3] = {-0.20, 0.0, 0.165}; // LEFT, CENTER, RIGHT
AlignStrafferCmd::AlignStrafferCmd(Straffer* pStraffer, Gripper* pGripper, Side side)  
                  : m_pStraffer(pStraffer), m_pGripper(pGripper), m_side(side)
{
  AddRequirements(m_pStraffer);
  m_offset = offsetStrafferLUT[(int)side];
}

// Called when the command is initially scheduled.
void AlignStrafferCmd::Initialize() {
  m_pStraffer->SetControlMode(ControlMode::CLOSED_LOOP);
  if((m_pGripper->m_state == Gripper::State::REST_LOADED) || (m_pGripper->m_state == Gripper::State::REST_EMPTY))
  {
    if(m_side == Side::CENTER)
    {
      m_pStraffer->m_state = Straffer::State::STRAFF_TO_STATION;
      m_pStraffer->m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::CENTER);
    }
    else
    {
      m_pStraffer->m_state = Straffer::State::SEEK_APRIL_TAG;
      m_pStraffer->m_counter = 10; // counter for State::SEEK_APRIL_TAG 
      m_pStraffer->m_targetOffset = m_offset;
      m_pStraffer->m_lowestAmbiguity = 1.0;
    }
  }
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
  else if(m_pStraffer->m_state == Straffer::State::AT_REEF || m_pStraffer->m_state == Straffer::State::AT_STATION)
    return true;
  else 
    return false;
}