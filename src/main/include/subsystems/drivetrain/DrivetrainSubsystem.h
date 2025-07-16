// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <functional>
#include "DrivetrainIO.h"
#include "lib/RateLimiter.h"
#include "DrivetrainConstants.h"
#include "lib/Alert.h"

class DrivetrainSubsystem : public frc2::SubsystemBase 
{
 public:
  DrivetrainSubsystem(DrivetrainIO *pIO);
  DrivetrainSubsystem(DrivetrainIO *pIO, 
                     std::function<double()> fxForwardAxis,
                     std::function<double()> fxRotationAxis,
                     std::function<bool()> fxSlowDriveButton,
                     std::function<double()> fxHeightFactor);
  
  enum class WantedDrive
  {
    STAND_BY,
    ARCADE_DRIVE,
    REVERSE_DRIVE,
    AUTO_PATH_FOLLOWER
  };
  enum class SystemDrive
  {
    ARCADE_DRIVE,
    REVERSE_ARCADE_DRIVE,
    // CURVE_DRIVE,
    // REVERSED_CURVE_DRIVE,
    // TANK_DRIVE,
    // REVERSED_TANK_DRIVE,
    AUTO_PATH_FOLLOWER
  };

  void SetWantedDrive(const WantedDrive wantedDrive);
  void ConfigureManualAxis(const std::function<double()> fxForwardAxis,
                          const std::function<double()> fxRotationAxis,
                          const std::function<bool()> fxSlowDriveButton,
                          const std::function<double()> fxHeightFactor);
  void Periodic() override;

 private:
  DrivetrainIO *m_pTankDriveIO;
  DrivetrainIOInputs inputs;

  WantedDrive m_wantedDrive = WantedDrive::STAND_BY;
  SystemDrive m_systemDrive = SystemDrive::ARCADE_DRIVE;

  std::pair<double, double> m_output{0.0, 0.0};

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
  std::function<double()> m_fxHeightFactor; //temporary
  bool m_axisAreActive;

  //ARCADE
  double m_rotationSigma{0.0}; // Weight for rotation in arcade drive
  RateLimiter m_forwardLimitedAxis{Settings::TIME_TO_REACH_FULL_FORWARD};
  RateLimiter m_rotationLimitedAxis{Settings::TIME_TO_REACH_FULL_ROTATION};

  //CURVE //TODO

  std::pair<double, double> ArcadeDrive(const double forward, const double rotation);
  //TODO
  // std::pair<double, double> TankDrive();
  // std::pair<double, double> CurveDrive();
  // std::pair<double, double> FollowPath();
};
