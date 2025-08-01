// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "frc/kinematics/ChassisSpeeds.h"

#include <functional>
#include <optional>

#include "lib/RateLimiter.h"
#include "lib/Alert.h"
#include "lib/PidRBL.h"
#include "lib/TimerRBL.h"

#include "DrivetrainIO.h"
#include "DrivetrainIOLogger.h"
#include "DrivetrainConstants.h"

#include "choreo/trajectory/DifferentialSample.h"
#include "choreo/trajectory/Trajectory.h"

class DrivetrainSubsystem : public frc2::SubsystemBase 
{
 public:
  enum class SystemDrive
  {
    ARCADE_DRIVE,
    CURVE_DRIVE,
    AUTO_PATH_FOLLOWER,
    DISABLE  
  };

  DrivetrainSubsystem(DrivetrainIO *pIO);
  DrivetrainSubsystem(DrivetrainIO *pIO, 
                     std::function<double()> fxForwardAxis,
                     std::function<double()> fxRotationAxis,
                     std::function<bool()> fxSlowDriveButton,
                     std::function<bool()> fxDriveActionButton,
                     std::function<double()> fxHeightFactor);

  void SetWantedDrive(const DriveMode wantedDrive);
  SystemDrive GetSystemDrive() const;
  void ConfigureManualAxis(const std::function<double()> fxForwardAxis,
                          const std::function<double()> fxRotationAxis,
                          const std::function<bool()> fxSlowDriveButton,
                          const std::function<bool()> fxDriveActionButton,
                          const std::function<double()> fxHeightFactor);
  
  void SetAlliance(frc::DriverStation::Alliance alliance);

  void SetDesiredAutoTrajectory(choreo::Trajectory<choreo::DifferentialSample> trajectory);
  
  void Periodic() override;

 private:
  DrivetrainIO *m_pTankDriveIO;
  DrivetrainIOInputs inputs;
  DrivetrainIOLogger m_logger{frc::DataLogManager::GetLog(), "/Drivetrain"};

  DriveMode m_wantedDrive = DriveMode::DISABLE;
  SystemDrive m_systemDrive = SystemDrive::DISABLE;

  frc::ChassisSpeeds m_output;

  Alert m_frontLeftMotorDisconnected{"Drivetrain Front Left Motor: Disconnected", Alert::AlertType::ERROR};
  Alert m_frontRightMotorDisconnected{"Drivetrain Front Right Motor: Disconnected", Alert::AlertType::ERROR};
  Alert m_backLeftMotorDisconnected{"Drivetrain Back Left Motor: Disconnected", Alert::AlertType::ERROR};
  Alert m_backRightMotorDisconnected{"Drivetrain Back Right Motor: Disconnected", Alert::AlertType::ERROR};

  Alert m_frontLeftMotorHot{"Drivetrain Front Left Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
  Alert m_frontRightMotorHot{"Drivetrain Front Right Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
  Alert m_backLeftMotorHot{"Drivetrain Back Left Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
  Alert m_backRightMotorHot{"Drivetrain Back Right Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};

  Alert m_frontLeftMotorOverheating{"Drivetrain Front Left Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
  Alert m_frontRightMotorOverheating{"Drivetrain Front Right Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
  Alert m_backLeftMotorOverheating{"Drivetrain Back Left Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
  Alert m_backRightMotorOverheating{"Drivetrain Back Right Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};

  std::function<double()> m_fxForwardAxis;
  std::function<double()> m_fxRotationAxis;
  std::function<bool()> m_fxSlowDriveButton;
  std::function<bool()> m_fxDriveActionButton; //Reverse in Arcade | QuickTurn in Curve
  std::function<double()> m_fxHeightFactor; //temporary
  bool m_axisAreActive;

  RateLimiter m_forwardLimitedAxis;
  RateLimiter m_rotationLimitedAxis;


  //ARCADE
  double m_rotationSigma{0.0}; // Weight for rotation in arcade drive

  //CURVE //TODO
  double m_previousRotation{0.0};
  double m_negInertiaAccumulator{0.0};
  double m_quickStopAccumulator{0.0};
  const frc::ChassisSpeeds restSpeeds;

  //AUTO
  choreo::Trajectory<choreo::DifferentialSample> m_desiredAutoTrajectory;
  TimerRBL m_autoTimer;
  std::optional<choreo::DifferentialSample> m_autoSampleToBeApplied;


  frc::DriverStation::Alliance m_alliance;

  frc::ChassisSpeeds ArcadeDrive(const std::pair<double, double> percentage);
  frc::ChassisSpeeds CurveDrive(const std::pair<double, double> percentage, const bool quickTurnEnabled);
  frc::ChassisSpeeds FollowPath();

  std::pair<double, double> GetSafetyPercentages(); // first = Fwd, Second = rotation
};