// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveDistanceCmd.h"

DriveDistanceCmd::DriveDistanceCmd(Camera *pCamera, Drivetrain *pDrivetrain) : m_pCamera(pCamera), m_pDrivetrain(pDrivetrain) {
  AddRequirements(m_pDrivetrain);
}

// Called when the command is initially scheduled.
void DriveDistanceCmd::Initialize() {
  m_pDrivetrain->SetPower(0.2);
}

// Called repeatedly when this Command is scheduled to run
void DriveDistanceCmd::Execute() {
  m_pCamera->Update();
}

// Called once the command ends or is interrupted.
void DriveDistanceCmd::End(bool interrupted) {
  m_pDrivetrain->SetPower(0.0);
}

// Returns true when the command should end.
bool DriveDistanceCmd::IsFinished() {
  if(m_pCamera->HasTargets() && (m_pCamera->GetDistance(m_pCamera->GetBestTarget()) < 60))
    return true;
  else 
    return false;
}
