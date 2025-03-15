// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/SparkMax.h"
#include <frc/Encoder.h>
#include <frc/AnalogInput.h>
#include "Constants.h"


class DeepClimb : public frc2::SubsystemBase {
 public:
  DeepClimb();

  void SetClimbSpeed(double speed);
  void Periodic() override;

 private:

  rev::spark::SparkMax m_climbFrontMotor{DeepClimbConstants::Motors::ID_FRONT, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_climbBackMotor{DeepClimbConstants::Motors::ID_BACK, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkBaseConfig m_climbFrontMotorConfig;
  rev::spark::SparkBaseConfig m_climbBackMotorConfig;

  // frc::Encoder m_climbEncoder;
};
