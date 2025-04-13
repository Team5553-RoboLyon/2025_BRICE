// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO Today : calibrate position
//TODO : MORE Trapezoidal Motion
//TODO : put rest at kG
//TODO : calibrate PID
#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/SparkMax.h"
#include <frc/Encoder.h>
#include <frc/DigitalInput.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <iostream>

#include "Constants.h"
#include "lib/pid_rbl.h"
#include "lib/rate_limiter.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();
  void SetControlMode(ControlMode mode);
  ControlMode GetControlMode();

  void SetDesiredHeight(double height);
  void SetDesiredStage(Stage stage);
  double GetHeight();
  bool IsAtDesiredStage();

  void SetJoystickInput(double input);

  void Periodic() override;

  bool isInitialized = false;
 private:
  void ClosedLoopControl();
  void OpenLoopControl();
  void Reset();

  rev::spark::SparkMax m_leftMotor{elevatorConstants::Motors::Left::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_leftMotorConfig;
  rev::spark::SparkMax m_rightMotor{elevatorConstants::Motors::Right::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_rightMotorConfig;

  frc::Encoder m_encoder{elevatorConstants::Sensor::Encoder::A_ID, elevatorConstants::Sensor::Encoder::B_ID, elevatorConstants::Sensor::Encoder::REVERSED};
  frc::DigitalInput m_bottomLimitSwitch{elevatorConstants::Sensor::LimitSwitch::BOTTOM_ID};
  frc::DigitalInput m_bottomLimitSwitch2{elevatorConstants::Sensor::LimitSwitch::BOTTOM_2_ID};
  PidRBL m_elevatorPIDController{elevatorConstants::PID::KP, elevatorConstants::PID::KI, elevatorConstants::PID::KD};
  RateLimiter m_rateLimiter;
  

  bool m_isBottomLimitSwitchTriggered;
  double m_height;

  double m_output;
  double m_joystickInput;
  ControlMode m_controlMode = elevatorConstants::defaultMode;
};