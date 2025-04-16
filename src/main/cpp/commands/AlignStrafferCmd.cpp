// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AlignStrafferCmd.h"

static double offsetStrafferLUT[3] = {-0.17, 0.0, 0.17}; // LEFT, CENTER, RIGHT
AlignStrafferCmd::AlignStrafferCmd(Straffer* pStraffer, Gripper* pGripper, Side side)  
                  : m_pStraffer(pStraffer), m_pGripper(pGripper), m_targetSide(side)
{
  AddRequirements(m_pStraffer);
  m_offset = offsetStrafferLUT[(int)side];
}

// Called when the command is initially scheduled.
void AlignStrafferCmd::Initialize() {
  m_pStraffer->SetControlMode(ControlMode::CLOSED_LOOP);
  isFinished = false;
}

// Called repeatedly when this Command is scheduled to run
void AlignStrafferCmd::Execute() {
  if((m_pGripper->m_state == Gripper::State::REST_LOADED) || (m_pGripper->m_state == Gripper::State::REST_EMPTY) || (m_pGripper->GetControlMode() == ControlMode::OPEN_LOOP))
  {
    if(m_targetSide == Side::CENTER)
    {
      m_pStraffer->m_state = Straffer::State::STRAFF_TO_STATION;
      m_pStraffer->m_strafferPIDController.SetSetpoint(strafferConstants::Setpoint::CENTER);
      m_pStraffer->m_counter = strafferConstants::Counter::STRAFF_TO_STATION; // counter for State::STRAFF_TO_STATION
    }
    else
    {
      m_pStraffer->m_state = Straffer::State::SEEK_APRIL_TAG;
      m_pStraffer->m_counter = strafferConstants::Counter::SEEK_APRIL_TAG; // counter for State::SEEK_APRIL_TAG 
      m_pStraffer->m_targetOffset = m_offset;
      m_pStraffer->m_lowestAmbiguity = 1.0; // Set the initial ambiguity at the highest possible (between 0.0 and 1.0)
    }
    isFinished = true;
  }
}

// Called once the command ends or is interrupted.
void AlignStrafferCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool AlignStrafferCmd::IsFinished() {
  if(m_pStraffer->GetControlMode() == ControlMode::OPEN_LOOP)
    return true;
  else 
    return isFinished;
}