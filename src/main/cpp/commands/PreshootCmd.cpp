// / Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PreshootCmd.h"

static double SpeedPerStageLUT[5] = { gripperConstants::Speed::SHOOTTTT,
                                       0.3, //L1
                                        0.4,//L2
                                         0.45,//L3
                                          0.4}; //L4


PreshootCmd::PreshootCmd(Gripper *pGripper, Straffer *pStraffer, Elevator *pElevator) 
                                                                                            : m_pGripper(pGripper), 
                                                                                            m_pStraffer(pStraffer), 
                                                                                            m_pElevator(pElevator){
  AddRequirements(m_pGripper);
}
// Called when the command is initially scheduled.
void PreshootCmd::Initialize() {
  isFinished = false;

  if(m_pGripper->GetControlMode() == ControlMode::OPEN_LOOP)
  {
    isFinished = true;
  }
}

// Called repeatedly when this Command is scheduled to run
void PreshootCmd::Execute() {
  switch (m_pGripper->m_state)
  {
  case Gripper::State::REST_LOADED :
    if(m_pStraffer->m_state == Straffer::State::AT_REEF || m_pStraffer->m_state == Straffer::State::AT_STATION || m_pStraffer->GetControlMode()== ControlMode::OPEN_LOOP)
    {
      m_pGripper->m_state = Gripper::State::PRESHOOT;
      m_pGripper->SetSpeedOuttake(gripperConstants::Speed::PRESHOOT);
      m_pGripper->m_ShoooootttttSpeed = SpeedPerStageLUT[(int)m_pElevator->GetStage()];
      m_pGripper->m_counter = gripperConstants::Counter::PRESHOOT;
      isFinished = true;
    }
    break;
  default:
    break;
  }
}

// Called once the command ends or is interrupted.
void PreshootCmd::End(bool interrupted) {
  // m_pStraffer->m_state = Straffer::State::IDLE;
}

// Returns true when the command should end.
bool PreshootCmd::IsFinished() 
{
    return isFinished;
}