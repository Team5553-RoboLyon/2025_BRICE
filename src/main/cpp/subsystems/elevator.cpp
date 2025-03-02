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
    m_state = CLEAR_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L1Desired);
    break;
  case 1:
    m_state = CLEAR_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L2Desired);
    break;
  case 2:
    m_state = CLEAR_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L3Desired);
    break;
  case 3:
    m_state = CLEAR_DESIRED_POSITION(m_state);
    m_state = SET_DESIRED_POSITION(m_state, elevatorConstants::State::m_L4Desired);
  default:
    break;
  }
}

void Elevator::Periodic() {
  actualizeCurrentPosition();
  if(GET_CURRENT_POSITION(m_state) < GET_DESIRED_POSITION(m_state)) {
    SetGoingUp();
  }
  else if(GET_CURRENT_POSITION(m_state) > GET_DESIRED_POSITION(m_state)) {
    SetGoingDown();
  }
  else if(GET_CURRENT_POSITION(m_state) == GET_DESIRED_POSITION(m_state)) {
    SetRestPosition();
  }
  ExecuteTask();
  Dashboard();
}
void Elevator::actualizeCurrentPosition() {
  if(NABS(m_ElevatorEncoder.GetDistance() - elevatorConstants::Sensor::Encoder::Distance::L1) < elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL1);
  } 
  else if(NABS(m_ElevatorEncoder.GetDistance() - elevatorConstants::Sensor::Encoder::Distance::L2) < elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL2);
  } 
  else if(NABS(m_ElevatorEncoder.GetDistance() - elevatorConstants::Sensor::Encoder::Distance::L3) < elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL3);
  } 
  else if(NABS(m_ElevatorEncoder.GetDistance() - elevatorConstants::Sensor::Encoder::Distance::L4) < elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL4);
  }
  else if((elevatorConstants::Sensor::Encoder::Distance::L1 + elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE ) < m_ElevatorEncoder.GetDistance() && m_ElevatorEncoder.GetDistance() < (elevatorConstants::Sensor::Encoder::Distance::L2 - elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE)) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL1L2);
  } 
  else if((elevatorConstants::Sensor::Encoder::Distance::L2 + elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE ) < m_ElevatorEncoder.GetDistance() && m_ElevatorEncoder.GetDistance() < (elevatorConstants::Sensor::Encoder::Distance::L3 - elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE)) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL2L3);
  } 
  else if((elevatorConstants::Sensor::Encoder::Distance::L3 + elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE ) < m_ElevatorEncoder.GetDistance() && m_ElevatorEncoder.GetDistance() < (elevatorConstants::Sensor::Encoder::Distance::L4 - elevatorConstants::Sensor::Encoder::Distance::TOLERANCE_PER_STAGE)) {
    m_state = CLEAR_CURRENT_POSITION(m_state);
    m_state = SET_CURRENT_POSITION(m_state, elevatorConstants::State::m_AtL3L4);
  }
}
void Elevator::ExecuteTask() {
  switch (GET_DESIRED_POSITION(m_state))
  {
  case elevatorConstants::State::m_L1Desired:
    switch (GET_MOVING_TYPE(m_state)){
      case elevatorConstants::State::Rest:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L1);
        m_elevatorMotor.StopMotor();
        break;
      case elevatorConstants::State::Up:
        m_state = CLEAR_MOVING_TYPE(m_state);
        m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
        m_elevatorMotor.StopMotor();
        break;
      case elevatorConstants::State::Down:
        //TODO : Check if the hall effect sensor is sensoring
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L1);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
    default:
      break;
    }
    break;
  case elevatorConstants::State::m_L2Desired:
    switch (GET_MOVING_TYPE(m_state)) {
      case elevatorConstants::State::Rest:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L2);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
      case elevatorConstants::State::Up:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L2);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
      case elevatorConstants::State::Down:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L2);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
    }
    break;
  case elevatorConstants::State::m_L3Desired:
    switch (GET_MOVING_TYPE(m_state)) {
      case elevatorConstants::State::Rest:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L3);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
      case elevatorConstants::State::Up:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L3);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
      case elevatorConstants::State::Down:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L3);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
    }
    break;
  case elevatorConstants::State::m_L4Desired:
    switch (GET_MOVING_TYPE(m_state)) {
      case elevatorConstants::State::Rest:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L4);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
      case elevatorConstants::State::Up:
        m_pid.SetSetpoint(elevatorConstants::Sensor::Encoder::Distance::L4);
        m_elevatorMotor.Set(m_pid.Calculate(m_ElevatorEncoder.GetDistance()));
        break;
      case elevatorConstants::State::Down:
        m_state = CLEAR_MOVING_TYPE(m_state);
        m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
        break;
    }
    break;
  default:
    break;
  }
}

void Elevator::SetRestPosition() {
  m_state = CLEAR_MOVING_TYPE(m_state);
  m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
}
void Elevator::SetGoingUp() {
  m_state = CLEAR_MOVING_TYPE(m_state);
  m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Up);
}
void Elevator::SetGoingDown() {
    m_state = CLEAR_MOVING_TYPE(m_state);
    m_state = SET_MOVING_TYPE(m_state, elevatorConstants::State::Down);
}

void Elevator::Dashboard() {
  frc::SmartDashboard::PutString("binary state", std::bitset<9>(m_state).to_string());
  frc::SmartDashboard::PutNumber("Elevator Encoder", m_ElevatorEncoder.GetDistance());
  frc::SmartDashboard::PutNumber("Elevator Current", m_elevatorMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Elevator Top Hall Effect", m_TopHallEffectSensor.GetVoltage());
  frc::SmartDashboard::PutNumber("Elevator Middle Hall Effect", m_MiddleHallEffectSensor.GetVoltage());
  frc::SmartDashboard::PutNumber("Elevator Bottom Hall Effect", m_BottomHallEffectSensor.GetVoltage());
  switch (GET_DESIRED_POSITION(m_state))
  {
  case elevatorConstants::State::m_L1Desired:
    frc::SmartDashboard::PutString("Desired Position", "L1");
    break;
  case elevatorConstants::State::m_L2Desired:
    frc::SmartDashboard::PutString("Desired Position", "L2");
    break;
  case elevatorConstants::State::m_L3Desired:
    frc::SmartDashboard::PutString("Desired Position", "L3");
    break;
  case elevatorConstants::State::m_L4Desired:
    frc::SmartDashboard::PutString("Desired Position", "L4");
    break;
  default:
    frc::SmartDashboard::PutString("Desired Position", "! ERROR, NOT GOOD !");
    break;
  }
  switch (GET_CURRENT_POSITION(m_state))
  {
  case elevatorConstants::State::m_AtL1:
    frc::SmartDashboard::PutString("Current Position", "L1");
    break;
  case elevatorConstants::State::m_AtL1L2:
    frc::SmartDashboard::PutString("Current Position", "L1/L2");
    break;
  case elevatorConstants::State::m_AtL2:  
    frc::SmartDashboard::PutString("Current Position", "L2");
    break;
  case elevatorConstants::State::m_AtL2L3:
    frc::SmartDashboard::PutString("Current Position", "L2/L3");
    break;
  case elevatorConstants::State::m_AtL3:
    frc::SmartDashboard::PutString("Current Position", "L3");
    break;
  case elevatorConstants::State::m_AtL3L4:  
    frc::SmartDashboard::PutString("Current Position", "L3/L4");
    break;
  case elevatorConstants::State::m_AtL4:  
    frc::SmartDashboard::PutString("Current Position", "L4");
    break;
  default:
    frc::SmartDashboard::PutString("Current Position", "! ERROR, NOT GOOD !");
    break;
  }
  switch (GET_MOVING_TYPE(m_state))
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
    frc::SmartDashboard::PutString("Moving Type", "! ERROR, NOT GOOD !");
    break;
  }
}

