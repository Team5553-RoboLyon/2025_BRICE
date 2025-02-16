// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/SparkMax.h"
#include <frc/Encoder.h>
#include "Constants.h"


class DeepClimb : public frc2::SubsystemBase {
 public:
  DeepClimb();

  void SetClimbSpeed(double speed);
  void StopClimb();
  double GetArmPosition();
  void resetClimberPosition();

  void Periodic() override;

  bool isClimbed;
 private:
  rev::spark::SparkMax m_climbFrontMotor{DeepClimbConstants::FRONT_MOTOR_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_climbFrontMotorConfig;
  rev::spark::SparkMax m_climbBackMotor{DeepClimbConstants::BACK_MOTOR_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_climbBackMotorConfig;

  frc::Encoder m_climbEncoder{DeepClimbConstants::ENCODER_A_ID, DeepClimbConstants::ENCODER_B_ID};
};
