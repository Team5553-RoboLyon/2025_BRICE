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
  // m_camera.Update();
  // frc::SmartDashboard::PutNumber("id", m_camera.GetAprilTagID(m_camera.GetBestTarget()));
  // frc::SmartDashboard::PutNumber("amb", m_camera.GetAmbiguity(m_camera.GetBestTarget()));
  // frc::SmartDashboard::PutNumber("y", m_camera.GetHorizontalDistance(m_camera.GetBestTarget()));

  // std::cout << m_container.m_controllerCopilot.GetRawAxis(2) << m_container.m_controllerCopilot.GetRawAxis(3) << std::endl;
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::DisabledExit() {
  m_container.m_elevator.isInitialized = false;
  m_container.m_straffer.isInitialized = false;
}

void Robot::AutonomousInit() {
  m_container.m_drivetrain.isAuto = true;
  m_container.m_gripper.SetControlMode(ControlMode::AUTO_LOOP);
  m_container.m_straffer.SetControlMode(ControlMode::AUTO_LOOP);
  m_container.m_elevator.SetControlMode(ControlMode::AUTO_LOOP);
  // initialPosition = m_container.m_drivetrain.DriveAuto();
  // m_container.m_drivetrain.SetPower(0.5);
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  // if((m_container.m_drivetrain.DriveAuto() - initialPosition) > target)
  // {
  //   m_container.m_drivetrain.SetPower(0.0);
  // }
  // else {
  //     m_container.m_drivetrain.SetPower(0.5);
  // }
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
  m_container.m_straffer.SetControlMode(strafferConstants::defaultMode);
  m_container.m_elevator.SetControlMode(elevatorConstants::defaultMode);
  m_container.m_gripper.SetControlMode(gripperConstants::defaultMode);
  m_container.m_drivetrain.isAuto = false;
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
  if(canRumble)
  {
    if(m_container.m_gripper.m_rumble || m_container.m_straffer.m_rumble)
    {
      canRumble = false;
      m_rumbleCounter = 11;
    }
  }
  else
  {
    m_rumbleCounter--;
    if(m_rumbleCounter == 0)
    {
      canRumble = true;
      m_container.m_straffer.m_rumble = false;
      m_container.m_gripper.m_rumble = false;
      m_container.m_controllerCopilot.SetRumble(frc::PS4Controller::RumbleType::kBothRumble, 0.0);
    }
    else 
    {
      m_container.m_controllerCopilot.SetRumble(frc::PS4Controller::RumbleType::kBothRumble, 0.5553);
    }
  }
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
