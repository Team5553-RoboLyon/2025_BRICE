// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include "rev/SparkMax.h"
#include "Constants.h"
#include "lib/UtilsRBL.h"
#include "lib/pid_rbl.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  void SetSpeed(double speed);
  void StopMotor();
  void GoToPosition(double position);
  void ResetEncoder();
  double GetCurrentElevatorHeight();
  double GetEncoderVelocity();
  bool IsAtPosition(double targetPosition, double tolerance);
  bool IsAtTopLimit();
  bool IsAtBottomLimit();
  void Periodic() override;

 private:

  rev::spark::SparkMax m_elevatorMotor{elevatorConstants::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};

  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  frc::AnalogInput m_TopHallEffectSensor{elevatorConstants::Sensor::ID_TOP};
  frc::AnalogInput m_MiddleHallEffectSensor{elevatorConstants::Sensor::ID_MIDDLE};
  frc::AnalogInput m_BottomHallEffectSensor{elevatorConstants::Sensor::ID_BOTTOM};

  frc::Encoder m_ElevatorEncoder{elevatorConstants::Encoder::ENCODER_A_ID, elevatorConstants::Encoder::ENCODER_B_ID};

  PidRBL m_pid{0.1, 0.0, 0.0};

  //debug variables

  nt::GenericEntry* ElevatorHeightEntry;
  nt::GenericEntry* encoderVelocityEntry;
  nt::GenericEntry* topLimitEntry;
  nt::GenericEntry* bottomLimitEntry;
  nt::GenericEntry* middleLimitEntry;
};
