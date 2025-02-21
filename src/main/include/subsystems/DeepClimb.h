// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/SparkMax.h"
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include "Constants.h"


class DeepClimb : public frc2::SubsystemBase {
 public:
  DeepClimb();

  void SetClimbSpeed(double speed);
  void GoToPosition(double position);
  void StopClimb();
  double GetArmPosition();
  void ResetClimberPosition();
  bool isClimbed();

  void Periodic() override;
 private:
  rev::spark::SparkMax m_climbFrontMotor{DeepClimbConstants::Motors::Front::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_climbFrontMotorConfig;
  rev::spark::SparkMax m_climbBackMotor{DeepClimbConstants::Motors::Front::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_climbBackMotorConfig;

  frc::Encoder m_climbEncoder{DeepClimbConstants::Encoder::ENCODER_A_ID, DeepClimbConstants::Encoder::ENCODER_B_ID};
  frc::DigitalInput m_hallEffectSensorUp{DeepClimbConstants::HallEffectSensor::UP};
  frc::DigitalInput m_hallEffectSensorDown{DeepClimbConstants::HallEffectSensor::DOWN};

  bool isClimbed;
};
