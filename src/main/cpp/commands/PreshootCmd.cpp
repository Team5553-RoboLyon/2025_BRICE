// / Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PreshootCmd.h"

PreshootCmd::PreshootCmd(Gripper *pGripper, Straffer *pStraffer, Elevator *pElevator) 
                                                                                            : m_pGripper(pGripper), 
                                                                                            m_pStraffer(pStraffer), 
                                                                                            m_pElevator(pElevator){
  AddRequirements(m_pGripper);
} //TODO : add expl
// Called when the command is initially scheduled.
void PreshootCmd::Initialize() {
    // std::cout << "cc" << std::endl;
  switch (m_pGripper->m_state)
  {
  case Gripper::State::REST_LOADED :
    m_pGripper->m_state = Gripper::State::PRESHOOT;
    m_pGripper->SetSpeedOuttake(gripperConstants::Speed::PRESHOOT);
    m_pGripper->m_counter = 6; //  counter preshoot
    break;
  
  default:
    break;
  }
}

// Called repeatedly when this Command is scheduled to run
void PreshootCmd::Execute() {}

// Called once the command ends or is interrupted.
void PreshootCmd::End(bool interrupted) {
}

// Returns true when the command should end.
bool PreshootCmd::IsFinished() 
{
    return false;
}