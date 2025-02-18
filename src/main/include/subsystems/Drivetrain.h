// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include "rev/SparkFlex.h"
#include "Constants.h"
#include "lib/UtilsRBL.h"
#include "lib/rate_limiter.h"
#include "lib/NRollingAverage.h"
#include "lib/Dynamic.h"
#include "lib/utils.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  void SetPower(double v_motor);
  void SetVoltage(double voltageLeft, double voltageRight);
  void DriveAuto(double speed, double rotation);

  void Drive(double FwdJoystick, double RotateJoystick);
  double Calcul_De_Notre_Cher_JM(double forward, double turn, bool wheelSide);
  void Stop();
  void Reset();

  void Periodic() override;

 private:
  rev::spark::SparkFlex m_MotorFrontLeft{DriveConstants::LeftGearbox::Motor::FRONT_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex m_MotorBackLeft{DriveConstants::LeftGearbox::Motor::BACK_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex m_MotorFrontRight{DriveConstants::RightGearbox::Motor::FRONT_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkFlex m_MotorBackRight{DriveConstants::RightGearbox::Motor::BACK_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkBaseConfig m_MotorFrontLeftConfig{};
  rev::spark::SparkBaseConfig m_MotorBackLeftConfig{};
  rev::spark::SparkBaseConfig m_MotorFrontRightConfig{};
  rev::spark::SparkBaseConfig m_MotorBackRightConfig{};

  frc::Encoder m_EncoderLeft{DriveConstants::LeftGearbox::Encoder::ID_ENCODER_A, DriveConstants::LeftGearbox::Encoder::ID_ENCODER_B, DriveConstants::LeftGearbox::Encoder::REVERSE_ENCODER, frc::Encoder::k4X};
  frc::Encoder m_EncoderRight{DriveConstants::RightGearbox::Encoder::ID_ENCODER_A, DriveConstants::RightGearbox::Encoder::ID_ENCODER_B, DriveConstants::RightGearbox::Encoder::REVERSE_ENCODER, frc::Encoder::k4X};

  double m_sigma = 0.0;

  RateLimiter m_JoystickPrelimited_V; // joystick V rate limiter 1
  RateLimiter m_JoystickLimited_V;    // joystick V rate limiter 2

  RateLimiter m_JoystickPrelimited_W; // joystick W rate limiter 1
  RateLimiter m_JoystickLimited_W;    // joystick W rate limiter 2

};
