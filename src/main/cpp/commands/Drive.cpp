// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"
#include "frc/smartdashboard/SmartDashboard.h"

Drive::Drive(std::function<double()> forward, std::function<double()> turn, Drivetrain *pDrivetrain, Elevator *pElevator)
    : m_Forward(forward), m_Turn(turn), m_pDrivetrain(pDrivetrain), m_pElevator(pElevator)
{
  AddRequirements({pDrivetrain});
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {
  if(!m_pDrivetrain->isAuto) {
    double forward = -m_Forward();
    double turn = m_Turn();
    double heightFactor = NORMALIZE_HEIGHT(m_pElevator->GetHeight());
    m_pDrivetrain->Drive(forward, turn, heightFactor);
  }
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished() {
  return false;
}