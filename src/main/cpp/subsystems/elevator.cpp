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

    //debug 

    frc::ShuffleboardTab& ElevatorTab = frc::Shuffleboard::GetTab("elevator");
    ElevatorHeightEntry = ElevatorTab.Add("Elevator Height", 0.0)
                        .WithWidget(frc::BuiltInWidgets::kNumberBar)
                        .GetEntry();

    encoderVelocityEntry = ElevatorTab.Add("Encoder Velocity", 0.0)
                        .WithWidget(frc::BuiltInWidgets::kGraph)
                        .GetEntry();

    topLimitEntry = ElevatorTab.Add("Top Limit", false)
                        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
                        .GetEntry();
    
    bottomLimitEntry = ElevatorTab.Add("Bottom Limit", false)
                        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
                        .GetEntry();
    
    middleLimitEntry = ElevatorTab.Add("Middle Limit", false) 
                        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
                        .GetEntry();
}

// This method will be called once per scheduler run
void Elevator::Periodic() {

    //debug
    ElevatorHeightEntry->SetDouble(GetCurrentElevatorHeight());
    encoderVelocityEntry->SetDouble(GetEncoderVelocity());
    topLimitEntry->SetBoolean(IsAtTopLimit());
    bottomLimitEntry->SetBoolean(IsAtBottomLimit());
    middleLimitEntry->SetBoolean(m_MiddleHallEffectSensor.GetVoltage()>elevatorConstants::Sensor::TOLERANCE);
}

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

