// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCoralCmd.h"

IntakeCoralCmd::IntakeCoralCmd(Gripper *pGripper, Straffer *pStraffer, Elevator *pElevator) 
                                                                                            : m_pGripper(pGripper), 
                                                                                            m_pStraffer(pStraffer), 
                                                                                            m_pElevator(pElevator){
  AddRequirements(m_pGripper);
} 
// Called when the command is initially scheduled.
void IntakeCoralCmd::Initialize() {
  if(m_pGripper->GetControlMode() == ControlMode::CLOSED_LOOP) 
  {
    switch (m_pGripper->m_state)
    {
    case Gripper::State::REST_EMPTY :
      if(m_pStraffer->m_state == Straffer::State::AT_STATION && m_pElevator->IsAtCoralStation())
      {
        m_pGripper->m_state = Gripper::State::INTAKE_EMPTY;
        m_pGripper->SetSpeedOuttake(gripperConstants::Speed::OUTTAKE_EMPTY);
        m_pGripper->SetSpeedIntake(gripperConstants::Speed::INTAKE_EMPTY);
      }
      break;
    
    case Gripper::State::REST_LOADED :
      m_pGripper->m_state = Gripper::State::INTAKE_FEEDING_BACKWARD;
      m_pGripper->SetSpeedOuttake(gripperConstants::Speed::FEEDING_BACKWARD);
      break;
    
    default:
      break;
    }
  }
}
//TODO : elevateur at positon

// Called repeatedly when this Command is scheduled to run
void IntakeCoralCmd::Execute() {}

// Called once the command ends or is interrupted.
void IntakeCoralCmd::End(bool interrupted) {
  if(m_pGripper->m_state == Gripper::State::INTAKE_EMPTY)
  {
    m_pGripper->m_state = Gripper::State::REST_EMPTY;
    m_pGripper->SetSpeedOuttake(gripperConstants::Speed::REST);
    m_pGripper->SetSpeedIntake(gripperConstants::Speed::REST);
  }
}

// Returns true when the command should end.
bool IntakeCoralCmd::IsFinished() 
{ 
  if(m_pGripper->GetControlMode() == ControlMode::OPEN_LOOP)
  {
  return true;
  }
  else if(m_pGripper->m_state == Gripper::State::REST_LOADED)
  {
    return true;
  }
  else 
    return false;
}