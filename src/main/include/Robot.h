// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
// #include "subsystems/Camera.h"
// #include <frc/smartdashboard/SmartDashboard.h>


#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;  
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

  void Leave();

 private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  // int count = 0;
  RobotContainer m_container;
  // int *m_currentRumble[3];
  int m_rumbleCounter;
  bool canRumble = true;
  double initialPosition;
  double target = 3.0; // in meters

  // int m_rumbleTable[5][3] //{intensity, side, number of cycles, time on, time off}}
  // {
  //     {75, frc::PS4Controller::RumbleType::kBothRumble, 8}, // Caught
  //     {75, frc::PS4Controller::RumbleType::kBothRumble, 16}, // Dropped
  //     {50, frc::PS4Controller::RumbleType::kLeftRumble, 10}, // LEFT_OUT_OF_RANGE,
  //     {50, frc::PS4Controller::RumbleType::kRightRumble, 10}, // RIGHT_OUT_OF_RANGE
  //     {0, frc::PS4Controller::RumbleType::kBothRumble, 1} // Nothing
  // };
};