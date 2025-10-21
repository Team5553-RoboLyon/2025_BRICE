// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/TimedRobot.h>
#include <frc/PWM.h>

#include "RobotContainer.h"
#include "lib/DebugUtils.h"

#include "lib/Alert.h"

#include "choreo/Choreo.h"
#include <optional>


class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotInit() override;
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

 private:
  RobotContainer m_container;
  int m_rumbleCounter;
  bool CanRumble = true;
  frc::PWM m_led{9};
  Camera m_camera;

  Alert m_isNotCompetitionRobot{"Not CompBot used", Alert::AlertType::WARNING};

  Alert m_pilot{"Pilot currently driving :", Alert::AlertType::INFO};
  Alert m_operator{"Operator currently operatoring :", Alert::AlertType::INFO};
  Alert m_robot{"Operator currently used :", Alert::AlertType::INFO};

  std::optional<choreo::Trajectory<choreo::DifferentialSample>> Traj = choreo::Choreo::LoadTrajectory<choreo::DifferentialSample>("Test2");
};