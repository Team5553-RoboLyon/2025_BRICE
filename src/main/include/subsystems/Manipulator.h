// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <bitset>
#include <string>
#include <cassert>
#include <array>
#include <iostream>

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
#include "lib/pid_rbl.h"
// #include "lib/NRollingAverage.h"
#include "lib/NSensorThreshold.h"

class Manipulator : public frc2::SubsystemBase {
 public:
 bool isInitialized = false;
  Manipulator();
  void Periodic() override;
  void Move(double ElevatorSpeed, double PlanetarySpeed);


  private:
  void Reset();
  void Dashboard();
  
  rev::spark::SparkMax m_elevatorMotor{ManipulatorConstants::Elevator::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  rev::spark::SparkMax m_planetaryMotor{ManipulatorConstants::Planetary::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_planetaryMotorConfig;


  frc::Encoder m_planetaryEncoder{ManipulatorConstants::Planetary::Sensor::Encoder::A_ID, ManipulatorConstants::Planetary::Sensor::Encoder::B_ID};
  frc::DigitalInput m_elevatorBottomLimitSwitch{ManipulatorConstants::Elevator::Sensor::LimitSwitch::BOTTOM_ID};
  frc::DigitalInput m_elevatorTopLimitSwitch{ManipulatorConstants::Elevator::Sensor::LimitSwitch::TOP_ID};
  
  bool m_isTopLimitSwitchTriggered;
  bool m_isBottomLimitSwitchTriggered;
  double m_planetaryAngle;

  double elevatorOutput = 0.0;
  double planetaryOutput = 0.0;
};