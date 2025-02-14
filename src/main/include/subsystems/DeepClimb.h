// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/SparkMax.h"
#include "rev/SparkRelativeEncoder.h"
#include "Constants.h"


class DeepClimb : public frc2::SubsystemBase {
 public:
  DeepClimb();

  void SetClimbSpeed(double speed);

  void Periodic() override;

 private:
  rev::spark::SparkMax m_climbFrontMotor{DeepClimbConstants::FRONT_MOTOR_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_climbFrontMotorConfig;
  rev::spark::SparkMax m_climbBackMotor{DeepClimbConstants::BACK_MOTOR_ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_climbBackMotorConfig;

  rev::spark::SparkRelativeEncoder encoder = m_climbFrontMotor.GetEncoder();

  std::function<double()> m_ClimbingSpeed;
};
