// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "rev/SparkMax.h"
#include "Constants.h"
#include <iostream>

class Gripper : public frc2::SubsystemBase {
 public:
  Gripper();

  void SetSpeedOuttake(double outtakeSpeed);
  void SetSpeedIntake(double intakeSpeed);
  void SetControlMode(ControlMode mode);
  void ToggleControlMode();
  ControlMode GetControlMode();
  
  bool IsMoving();

  void Periodic() override;

  bool isRumbled = false;
  double rumbleTime = 0.0;
    enum class State {
    IDLE,
    REST_EMPTY,
    REST_LOADED,
    INTAKE_EMPTY,
    INTAKE_FEEDING_FORWARD,
    INTAKE_FEEDING_BACKWARD,
    INTAKE_FEEDING_FORWARD_SHY,
    PRESHOOT,
    SHOOT
  };
  int m_counter{0};
  double m_ShoooootttttSpeed;

  State m_state = State::IDLE;
  
 private:
   void OpenLoopControl();
  void ClosedLoopControl();

  rev::spark::SparkMax m_outtakeMotor{gripperConstants::Motors::Outtake::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_outtakeMotorConfig;
  rev::spark::SparkMax m_intakeMotor{gripperConstants::Motors::Intake::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_intakeMotorConfig;
  frc::DigitalInput m_IRBreakerDown{gripperConstants::Sensor::IRbreaker::DOWN_ID};
  frc::DigitalInput m_IRBreakerUp{gripperConstants::Sensor::IRbreaker::UP_ID};
  frc::DigitalInput m_IRBreakerUp2{gripperConstants::Sensor::IRbreaker::UP2_ID};

  bool m_isIRBreakerDownTriggered;
  bool m_isIRBreakerUpTriggered;
  ControlMode m_controlMode = gripperConstants::defaultMode;
};