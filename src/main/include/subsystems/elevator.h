// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <bitset>
#include <string>
#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
  void SelectWantedStage(int8_t stage);
  void SelectCurrentPosition(int8_t position);
  void SelectMoving(int8_t movingType);
  void Periodic() override;

 private:

  rev::spark::SparkMax m_elevatorMotor{elevatorConstants::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  frc::AnalogInput m_TopHallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_TOP};
  frc::AnalogInput m_MiddleHallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_MIDDLE};
  frc::AnalogInput m_BottomHallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_BOTTOM};
  frc::Encoder m_ElevatorEncoder{elevatorConstants::Sensor::Encoder::A_ID, elevatorConstants::Sensor::Encoder::B_ID};

  PidRBL m_pid{elevatorConstants::PID::KD, elevatorConstants::PID::KI, elevatorConstants::PID::KD};
  int32_t m_state;
};
// 3 bits par partie sur un total de 32 bits
// 0000 0000 0000 0000 0000 0000 0000 0111 : Desired Position
// 0000 0000 0000 0000 0000 0000 0011 1000 : Current Position
// 0000 0000 0000 0000 0000 0001 1100 0000 : Moving Type
//
// DESIRED POSITION
// 000 : L1
// 001 : L2
// 010 : L3
// 011 : L4
//
// CURRENT POSITION
// 000 : L1
// 001 : L1/L2
// 010 : L2
// 011 : L2/L3
// 100 : L3
// 101 : L3/L4
// 110 : L4
//
// MOVING TYPE
// 000 : Rest
// 001 : Up
// 010 : Down
//
// Masks
// 0000 0000 0000 0000 0000 0001 1111 1000 : Desired Position
// 0000 0000 0000 0000 0000 0001 1100 0111 : Current Position
// 0000 0000 0000 0000 0000 0000 0011 1111 : Moving Type
