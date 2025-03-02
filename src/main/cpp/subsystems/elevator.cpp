// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/elevator.h"

Elevator::Elevator(){ //ok
    m_elevatorMotorConfig
                .VoltageCompensation(elevatorConstants::Motors::VOLTAGE_COMPENSATION)
                .Inverted(elevatorConstants::Motors::INVERTED)
                .SmartCurrentLimit(elevatorConstants::Motors::CURRENT_LIMIT)
                .ClosedLoopRampRate(elevatorConstants::Motors::RAMP_RATE)
                .SetIdleMode(elevatorConstants::Motors::IDLE_MODE);

    m_elevatorMotor.Configure(
                m_elevatorMotorConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_ElevatorEncoder.Reset();
    m_ElevatorEncoder.SetDistancePerPulse(elevatorConstants::Sensor::Encoder::DISTANCE_PER_PULSE);

    m_pid.SetTolerance(elevatorConstants::PID::TOLERANCE);
    m_pid.Reset(elevatorConstants::PID::SETPOINT);
    m_pid.SetOutputLimits(elevatorConstants::Speed::MAX_SPEED, elevatorConstants::Speed::MIN_SPEED);

    m_state = 0b000'000'000; // Set the Initial State to rest postion at L1
}


void Elevator::SelectWantedStage(int8_t stage) {
  switch (stage) {
  case 0:
    m_state = GET_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L1Desired);
    break;
  case 1:
    m_state = GET_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L2Desired);
    break;
  case 2:
    m_state = GET_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L3Desired);
    break;
  case 3:
    m_state = GET_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L4Desired);
  default:
    break;
  }
}
void Elevator::SelectCurrentPosition(int8_t position) {
  switch (position) {
  case 0:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL1);
    break;
  case 1:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL1L2);
    break;
  case 2:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL2);
    break;
  case 3:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL2L3);
    break;
  case 4:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL3);
    break;
  case 5:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL3L4);
    break;
  case 6:
    m_state = GET_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL4);
    break;
  default:
    break;
  }
}
void Elevator::SelectMoving(int8_t movingType) {
  switch (movingType) {
  case 0:
    m_state = GET_MOVING_TYPE(m_state);
    m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
    break;
  case 1:
    m_state = GET_MOVING_TYPE(m_state);
    m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Up);
    break;
  case 2:
    m_state = GET_MOVING_TYPE(m_state);
    m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Down);
    break;
  default:
    break;
  }
}
// This method will be called once per scheduler run
void Elevator::Periodic() {
  frc::SmartDashboard::PutString("state", std::bitset<9>(m_state).to_string());
}

