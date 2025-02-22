// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/elevator.h"

Elevator::Elevator(){
    m_elevatorMotorConfig
                .VoltageCompensation(elevatorConstants::Motors::VOLTAGE_COMP)
                .Inverted(elevatorConstants::Motors::INVERTED)
                .SmartCurrentLimit(elevatorConstants::Motors::CURRENT_LIMIT)
                .ClosedLoopRampRate(elevatorConstants::Motors::RAMP_RATE)
                .SetIdleMode(elevatorConstants::Motors::IDLE_MODE);

    m_elevatorMotor.Configure(
                m_elevatorMotorConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_ElevatorEncoder.Reset();
    m_ElevatorEncoder.SetDistancePerPulse(elevatorConstants::Encoder::DISTANCE_PER_PULSE);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {}

void Elevator::SetSpeed(double speed) {
  m_elevatorMotor.Set(speed);
}

void Elevator::StopMotor() {
  m_elevatorMotor.StopMotor();
}

void Elevator::GoToPosition(double position) {
    if (!IsAtTopLimit()){
        SetSpeed(m_pid.Calculate(m_ElevatorEncoder.GetDistance(), position));
    }
    else{
        StopMotor();
    }
}

void Elevator::ResetEncoder() {
  m_ElevatorEncoder.Reset();
}

double Elevator::GetCurrentElevatorHeight() {
  return m_ElevatorEncoder.GetDistance();
}

double Elevator::GetEncoderVelocity() {
  return m_ElevatorEncoder.GetRate();
}

bool Elevator::IsAtPosition(double targetPosition, double tolerance) {
  return NABS(targetPosition - GetCurrentElevatorHeight()) < tolerance;
}

bool Elevator::IsAtTopLimit() {
  return m_TopHallEffectSensor.GetVoltage() > elevatorConstants::Sensor::TOLERANCE;
}

bool Elevator::IsAtBottomLimit() {
  return m_BottomHallEffectSensor.GetVoltage() > elevatorConstants::Sensor::TOLERANCE;
}

