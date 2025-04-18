// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunDefaultGripper.h"


RunDefaultGripper::RunDefaultGripper(Gripper *pGripper, frc::PS4Controller *pGamepad) : m_pGripper(pGripper), m_pGamepad(pGamepad) {
  AddRequirements(m_pGripper);
}

// Called when the command is initially scheduled.
void RunDefaultGripper::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunDefaultGripper::Execute() {
  if(m_pGripper->GetControlMode() == ControlMode::OPEN_LOOP) 
  {
    if(m_pGamepad->GetRawAxis(3) > 0.1)
    {
      m_pGripper->SetSpeedIntake(m_pGamepad->GetRawAxis(3)/3.0);
      m_pGripper->SetSpeedOuttake(m_pGamepad->GetRawAxis(3)/3.0);
    }
    else if(m_pGamepad->GetRawAxis(2) >0.1)
    {
      m_pGripper->SetSpeedIntake(-m_pGamepad->GetRawAxis(2)/3.0);
      m_pGripper->SetSpeedOuttake(-m_pGamepad->GetRawAxis(2)/3.0);
    }
    else 
    {
      m_pGripper->SetSpeedIntake(gripperConstants::Speed::REST);
      m_pGripper->SetSpeedOuttake(gripperConstants::Speed::REST);
    }
  }
}

// Called once the command ends or is interrupted.
void RunDefaultGripper::End(bool interrupted) {
 m_pGripper->SetSpeedIntake(gripperConstants::Speed::REST);
 m_pGripper->SetSpeedOuttake(gripperConstants::Speed::REST);
}

// Returns true when the command should end.
bool RunDefaultGripper::IsFinished() {
  return false;
}