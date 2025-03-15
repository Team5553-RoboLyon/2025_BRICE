// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"
#include "frc/smartdashboard/SmartDashboard.h"

Drive::Drive(std::function<double()> forward, std::function<double()> turn, Drivetrain *pDrivetrain)
    : m_Forward(forward), m_Turn(turn), m_pDrivetrain(pDrivetrain)
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({pDrivetrain});
}

// Called when the command is initially scheduled.
void Drive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() {
  double forward = -m_Forward();
  frc::SmartDashboard::PutNumber("forward", forward);
  double turn = m_Turn();
  frc::SmartDashboard::PutNumber("turn", turn);
  m_pDrivetrain->Drive(forward, turn);
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished() {
  return false;
}
