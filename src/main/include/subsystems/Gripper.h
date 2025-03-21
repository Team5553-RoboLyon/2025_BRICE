// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
// #include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
// #include <networktables/NetworkTableEntry.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include "rev/SparkMax.h"
#include <frc/DigitalInput.h>

#include "Constants.h"
#include "lib/UtilsRBL.h"
// #include "lib/NRollingAverage.h"

class Gripper : public frc2::SubsystemBase {
 public:
  bool gripperActivated = false;

  rev::spark::SparkMax m_gripperMotorTop{GrippperConstants::Motors::ID_TOP, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_gripperMotorTopConfig;
  rev::spark::SparkMax m_gripperMotorBottom{GrippperConstants::Motors::ID_BOTTOM, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_gripperMotorBottomConfig;
  Gripper();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetIntakeSpeed(double speed);
  void SetDropSpeed(double speed);

 private:

   frc::DigitalInput m_irBreaker{GrippperConstants::Sensor::IR_BREAKER_ID};
  bool m_isIRBreakerTriggered;
    double gripperOutput = 0.0;
  u_int32_t m_gripperCounter;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
