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

    // Set the Initial State to rest postion at L1
    m_state = MAKE_ELEVATOR_STATE(elevatorConstants::State::L1, elevatorConstants::State::L1, elevatorConstants::State::Rest);
}

void Elevator::SelectWantedStage(u_int16_t stage) {
  switch (stage) {
  case elevatorConstants::State::L1:
    if(m_counter > elevatorConstants::TIMEOUT) {
      m_state = SET_ELEVATOR_DESIRED_POSITION(m_state, elevatorConstants::State::L1);
    }
    else {
      m_counter++;
    }
    break;
  case elevatorConstants::State::L2:
    m_counter = 0;
    m_state = SET_ELEVATOR_DESIRED_POSITION(m_state, elevatorConstants::State::L2);
    break;
  case elevatorConstants::State::L3:
    m_counter = 0;
    m_state = SET_ELEVATOR_DESIRED_POSITION(m_state, elevatorConstants::State::L3);
    break;
  case elevatorConstants::State::L4:
    m_counter = 0;
    m_state = SET_ELEVATOR_DESIRED_POSITION(m_state, elevatorConstants::State::L4);
  default:
    m_counter++;
    break;
  }
}

void Elevator::Periodic() {
  ActualizeCurrentPosition();

  switch (GET_ELEVATOR_MOVING_TYPE(m_state)) {
    case elevatorConstants::State::Rest:
      if(GET_ELEVATOR_CURRENT_POSITION(m_state) == GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        SetDesiredSetpoint();
      }
      else if(GET_ELEVATOR_CURRENT_POSITION(m_state) < GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Up);
        SetDesiredSetpoint();
        SetNextCurrentPosition(true);
      }
      else /*if(GET_ELEVATOR_CURRENT_POSITION(m_state) > GET_ELEVATOR_DESIRED_POSITION(m_state))*/ {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Down);
        SetDesiredSetpoint();
        SetNextCurrentPosition(false);
      }
      break;
    case elevatorConstants::State::Up:
      if(GET_ELEVATOR_CURRENT_POSITION(m_state) == GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
        SetDesiredSetpoint();
      }
      break;
    case elevatorConstants::State::Down:
      if(GET_ELEVATOR_CURRENT_POSITION(m_state) == GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
        SetDesiredSetpoint();
      }
      break;
  default:
    assert(false && "void Elevator::Periodic() -> moving type undefined");
    break;
  }

  // ----------------- Movement -----------------
  ExecuteTask();
  Dashboard();
}
void Elevator::ActualizeCurrentPosition() {
  if(m_bottomLimitSwitch.Get())
    m_state = SET_ELEVATOR_REST_AT_POSITION(m_state, elevatorConstants::State::L1);
  else if(m_topLimitSwitch.Get())
    m_state = SET_ELEVATOR_REST_AT_POSITION(m_state, elevatorConstants::State::L4);
  else {
      for (u_int i = 0; i < std::size(m_distanceEncoder); i++) {
        if (GetHight() >= m_distanceEncoder[i][0] && GetHight() <= m_distanceEncoder[i][1]) {
          m_state = SET_ELEVATOR_CURRENT_POSITION(m_state, i);
          break;
    }
  }
  }
}
void Elevator::ExecuteTask() {
  //motion profile
    m_elevatorMotor.Set(m_pid.Calculate(GetHight()));
}
double Elevator::GetHight() {
  double result = m_ElevatorEncoder.GetDistance() - m_EncoderDriftFilter.get();
  assert((result < m_distanceEncoder[0][0] && result > m_distanceEncoder[6][1])&& "distance is out of range");
  return result;
}
void Elevator::SetDesiredSetpoint() {
    int i = GET_ELEVATOR_DESIRED_POSITION(m_state);
    assert(!IS_TRANSITION_POSITION(i) && "void Elevator::SetDesiredSetpoint() -> desired position not found");
    m_pid.SetSetpoint(m_distanceEncoder[i][2]);
  }
void Elevator::SetNextCurrentPosition(bool isUpside) {
  if(isUpside) {
      int i = GET_ELEVATOR_CURRENT_POSITION(m_state) +1;
      assert(i <= elevatorConstants::State::L4 && "void Elevator::SetNextCurrentPosition() -> current position not found or impossible");
      m_state = SET_ELEVATOR_CURRENT_POSITION(m_state, i);
    }
  else {
      int i = GET_ELEVATOR_CURRENT_POSITION(m_state) -1;
      assert(i >= elevatorConstants::State::L1 && "void Elevator::SetNextCurrentPosition() -> current position not found or impossible");
      m_state = SET_ELEVATOR_CURRENT_POSITION(m_state, i);
    } 
}
void Elevator::Dashboard() {
  frc::SmartDashboard::PutString("binary state", std::bitset<12>(m_state).to_string());
  frc::SmartDashboard::PutNumber("Elevator Encoder", GetHight());
  frc::SmartDashboard::PutNumber("Elevator Current", m_elevatorMotor.GetOutputCurrent());
  // frc::SmartDashboard::PutNumber("Elevator L2 Hall Effect", m_stageL2HallEffectSensor.GetVoltage());
  // frc::SmartDashboard::PutNumber("Elevator L3 Hall Effect", m_stageL3HallEffectSensor.GetVoltage());
  switch (GET_ELEVATOR_DESIRED_POSITION(m_state))
  {
  case elevatorConstants::State::L1:
    frc::SmartDashboard::PutString("Desired Position", "L1");
    break;
  case elevatorConstants::State::L2:
    frc::SmartDashboard::PutString("Desired Position", "L2");
    break;
  case elevatorConstants::State::L3:
    frc::SmartDashboard::PutString("Desired Position", "L3");
    break;
  case elevatorConstants::State::L4:
    frc::SmartDashboard::PutString("Desired Position", "L4");
    break;
  default:
    assert(false  && "void Elevator::Dashboard() -> desired position not found ");
    break;
  }
  switch (GET_ELEVATOR_CURRENT_POSITION(m_state))
  {
  case elevatorConstants::State::L1:
    frc::SmartDashboard::PutString("Current Position", "L1");
    break;
  case elevatorConstants::State::L1L2:
    frc::SmartDashboard::PutString("Current Position", "L1/L2");
    break;
  case elevatorConstants::State::L2:  
    frc::SmartDashboard::PutString("Current Position", "L2");
    break;
  case elevatorConstants::State::L2L3:
    frc::SmartDashboard::PutString("Current Position", "L2/L3");
    break;
  case elevatorConstants::State::L3:
    frc::SmartDashboard::PutString("Current Position", "L3");
    break;
  case elevatorConstants::State::L3L4:  
    frc::SmartDashboard::PutString("Current Position", "L3/L4");
    break;
  case elevatorConstants::State::L4:  
    frc::SmartDashboard::PutString("Current Position", "L4");
    break;
  default:
    assert(false && "void Elevator::Dashboard() -> current position not found");
    break;
  }
  switch (GET_ELEVATOR_MOVING_TYPE(m_state))
  {
  case elevatorConstants::State::Rest:
    frc::SmartDashboard::PutString("Moving Type", "Rest");
    break;
  case elevatorConstants::State::Up:
    frc::SmartDashboard::PutString("Moving Type", "Up");
    break;
  case elevatorConstants::State::Down:
    frc::SmartDashboard::PutString("Moving Type", "Down");
    break;
  default:
    assert(false && "void Elevator::Dashboard() -> moving type not found ");
    break;
  }
}

