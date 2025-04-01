// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include "rev/SparkMax.h"
#include "Constants.h"

class Outtake : public frc2::SubsystemBase {
 public:
  Outtake();

  void SetSpeed(double speed);
  void SetControlMode(ControlMode mode);
  ControlMode GetControlMode();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  bool isRumbled = false;
  bool canCatch = false;
  bool canDrop = false;
 private:
  enum class State {
    REST,
    STARTING,
    CATCHING,
    CAUGHT,
    DROPPING,
    DROPPED,
  };
  State m_state = State::REST;
  rev::spark::SparkMax m_motor{outtakeConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_motorConfig;
  frc::DigitalInput m_IRBreakerDown{outtakeConstants::Sensor::IRbreaker::DOWN_ID};
  frc::DigitalInput m_IRBreakerUp{outtakeConstants::Sensor::IRbreaker::UP_ID};

  bool m_isIRBreakerDownTriggered;
  bool m_isIRBreakerUpTriggered;
  double m_output;
  double m_counter{0.0};
  ControlMode m_controlMode = outtakeConstants::defaultMode;

  void openLoop();
  void closedLoop();
};
