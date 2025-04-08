// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/SparkMax.h"
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>

#include "Constants.h"
#include "lib/pid_rbl.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  void SetDesiredHeight(double height);
  void SetDesiredStage(Stage stage);
  double GetHeight();
  void SetJoystickInput(double input);
  void SetControlMode(DriveMode mode);
  DriveMode GetControlMode();
  bool IsAtDesiredStage();

  void Reset();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 void ClosedLoopControl();
  void OpenLoopControl();
  rev::spark::SparkMax m_leftMotor{elevatorConstants::Motors::Left::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_leftMotorConfig;

  rev::spark::SparkMax m_rightMotor{elevatorConstants::Motors::Right::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_rightMotorConfig;

  frc::Encoder m_encoder{elevatorConstants::Sensor::Encoder::A_ID, elevatorConstants::Sensor::Encoder::B_ID, elevatorConstants::Sensor::Encoder::REVERSED, frc::Encoder::EncodingType::k4X};
  frc::DigitalInput m_topLimitSwitch{elevatorConstants::Sensor::LimitSwitch::TOP_ID};
  frc::DigitalInput m_topLimitSwitch2{elevatorConstants::Sensor::LimitSwitch::TOP_2_ID};
  frc::DigitalInput m_bottomLimitSwitch{elevatorConstants::Sensor::LimitSwitch::BOTTOM_ID};

  PidRBL m_elevatorPIDController{elevatorConstants::PID::KP, elevatorConstants::PID::KI, elevatorConstants::PID::KD};

  bool m_isBottomLimitSwitchTriggered;
  bool m_isTopLimitSwitchTriggered;
  double m_height;

  double m_output;
  double m_joystickInput;
  DriveMode m_driveMode = elevatorConstants::defaultMode;
};
