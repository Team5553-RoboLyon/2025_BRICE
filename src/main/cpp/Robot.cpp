// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h> 
#include <iostream>

Robot::Robot() {
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_container.m_drivetrain.isAuto = true;
  initialPosition = m_container.m_drivetrain.DriveAuto();
  m_container.m_drivetrain.SetPower(0.5);
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  if((m_container.m_drivetrain.DriveAuto() - initialPosition) > target)
  {
    m_container.m_drivetrain.SetPower(0.0);
  }
  else {
      m_container.m_drivetrain.SetPower(0.5);
  }
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
  m_container.m_drivetrain.isAuto = false;
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
  // frc::SmartDashboard::PutNumber("hall effect sensor", m_hallEffectSensor.GetVoltage());
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
