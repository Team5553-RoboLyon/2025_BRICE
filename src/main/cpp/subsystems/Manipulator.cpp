// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Manipulator.h"
#include <iostream>

Manipulator::Manipulator() { 
    m_elevatorMotorConfig
                .VoltageCompensation(ManipulatorConstants::Elevator::Motors::VOLTAGE_COMPENSATION)
                .Inverted(ManipulatorConstants::Elevator::Motors::INVERTED)
                .SmartCurrentLimit(ManipulatorConstants::Elevator::Motors::CURRENT_LIMIT)
                .ClosedLoopRampRate(ManipulatorConstants::Elevator::Motors::RAMP_RATE)
                .SetIdleMode(ManipulatorConstants::Elevator::Motors::IDLE_MODE);

    m_elevatorMotor.Configure(
                m_elevatorMotorConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_planetaryMotorConfig
                .VoltageCompensation(ManipulatorConstants::Planetary::Motors::VOLTAGE_COMPENSATION)
                .Inverted(ManipulatorConstants::Planetary::Motors::INVERTED)
                .SmartCurrentLimit(ManipulatorConstants::Planetary::Motors::CURRENT_LIMIT)
                .ClosedLoopRampRate(ManipulatorConstants::Planetary::Motors::RAMP_RATE)
                .SetIdleMode(ManipulatorConstants::Planetary::Motors::IDLE_MODE);

    m_planetaryMotor.Configure(
                m_planetaryMotorConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_planetaryEncoder.Reset();
    m_planetaryEncoder.SetDistancePerPulse(ManipulatorConstants::Planetary::Sensor::Encoder::DISTANCE_PER_PULSE);
    m_planetaryEncoder.SetReverseDirection(ManipulatorConstants::Planetary::Sensor::Encoder::REVERSED); // add Hall Fx sensor
}
void Manipulator::Reset() {
  if(m_elevatorBottomLimitSwitch.Get() == ManipulatorConstants::Elevator::Sensor::LimitSwitch::IS_TRIGGERED) {
    m_elevatorMotor.Set(ManipulatorConstants::Elevator::Speed::REST);
    isInitialized = true;
  }
  else {
      m_elevatorMotor.Set(ManipulatorConstants::Elevator::Speed::CALIBRATION);
  }
}
void Manipulator::Move(double ElevatorSpeed, double PlanetarySpeed) {
  elevatorOutput = ElevatorSpeed / 1.0;
  planetaryOutput = PlanetarySpeed / 4.5;
}

void Manipulator::Periodic() {
  // if(!isInitialized) {
  //     Reset();
  //     return;
  // }
  // ----------------- Save sensors value -----------------
      // ELEVATOR
  m_isBottomLimitSwitchTriggered = m_elevatorBottomLimitSwitch.Get() == ManipulatorConstants::Elevator::Sensor::LimitSwitch::IS_TRIGGERED;
  m_isTopLimitSwitchTriggered = m_elevatorTopLimitSwitch.Get() == ManipulatorConstants::Elevator::Sensor::LimitSwitch::IS_TRIGGERED; 
  //     // PLANETARY
  m_planetaryAngle = std::fmod(m_planetaryEncoder.GetDistance(), 2*M_PI);

  // ----------------- Dynamic -----------------
  // Executes the dynamic task for controlling the elevator motor.
  if(m_isTopLimitSwitchTriggered && elevatorOutput > 0.0) {
    elevatorOutput = 0.0;
  }
  else if(m_isBottomLimitSwitchTriggered && elevatorOutput < 0.0) {
    elevatorOutput = 0.0;
  }
  // if(planetaryOutput < 0.0 & m_planetaryAngle < 0.0)
  // { 
  //   planetaryOutput = 0.0;
  // }
  // if(planetaryOutput < 0.0)
  m_elevatorMotor.Set(elevatorOutput);
  m_planetaryMotor.Set(planetaryOutput);
  Dashboard();
}
void Manipulator::Dashboard() {
  // ----------------- Elevator -----------------
  frc::SmartDashboard::PutNumber("Elevator Current", m_elevatorMotor.GetOutputCurrent());
  frc::SmartDashboard::PutBoolean("Elevator Bottom Limit Switch", m_elevatorBottomLimitSwitch.Get());
  frc::SmartDashboard::PutBoolean("Elevator Top Limit Switch", m_elevatorTopLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Elevator motor Output",m_elevatorMotor.Get());
  // // ----------------- Planetary -----------------
  frc::SmartDashboard::PutNumber("Planetary Encoder", m_planetaryAngle);
  frc::SmartDashboard::PutNumber("Planetary Current", m_planetaryMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Planetary motor Output",m_planetaryMotor.Get());
  // ----------------- Gripper -----------------
}

