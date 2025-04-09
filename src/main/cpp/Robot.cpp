// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <iostream>

Robot::Robot() {
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
  m_container.m_straffer.isInitialized = false;
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  m_container.m_straffer.SetControlMode(ControlMode::AUTO_LOOP);
  m_container.m_elevator.SetControlMode(ControlMode::AUTO_LOOP);
  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
  m_container.m_straffer.SetControlMode(strafferConstants::defaultMode);
  m_container.m_elevator.SetControlMode(elevatorConstants::defaultMode);
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }

  if(m_container.m_outtake.isRumbled) 
  {
    if(m_rumbleCounter < m_container.m_outtake.rumbleTime)
    {
      m_container.m_controllerCopilot.SetRumble(frc::GenericHID::kBothRumble, 0.5);
      m_rumbleCounter++;
    }
    else 
    {
      m_container.m_controllerCopilot.SetRumble(frc::GenericHID::kBothRumble, 0.0);
      m_container.m_outtake.isRumbled = false;
      m_rumbleCounter = 0;
    }
  }
}

void Robot::TeleopPeriodic() {
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