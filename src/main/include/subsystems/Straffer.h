// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/Encoder.h>
#include "rev/SparkMax.h"
#include <iostream>

#include "Constants.h"
#include "lib/UtilsRBL.h"
#include "lib/rate_limiter.h"
#include "lib/pid_rbl.h"

class Straffer : public frc2::SubsystemBase {
 public:
  Straffer();
  void SetControlMode(ControlMode mode);
  ControlMode GetControlMode();

  void SetDesiredPosition(double Position);
  double GetPosition();
  bool IsAtDesiredPosition();
  void SetDesiredSide(Side side);

  void SetJoystickInput(double input);

  void Periodic() override;

  bool isInitialized = false;
 private:
  void OpenLoopControl();
  void ClosedLoopControl();
  void Reset();

  rev::spark::SparkMax m_motor{strafferConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_motorConfig;
  frc::DigitalInput m_leftLimitSwitch{strafferConstants::Sensor::LimitSwitch::LEFT_ID};
  frc::DigitalInput m_rightLimitSwitch{strafferConstants::Sensor::LimitSwitch::RIGHT_ID};
  frc::Encoder m_encoder{strafferConstants::Sensor::Encoder::A_ID, strafferConstants::Sensor::Encoder::B_ID, strafferConstants::Sensor::Encoder::REVERSED};

  bool m_isLeftLimitSwitchTriggered;
  bool m_isRightLimitSwitchTriggered;
  double m_width;

  double m_output;
  double m_joystickInput;
  ControlMode m_controlMode = strafferConstants::defaultMode;

  RateLimiter m_rateLimiter;
  PidRBL m_strafferPIDController{strafferConstants::PID::KP, strafferConstants::PID::KI, strafferConstants::PID::KD};
};