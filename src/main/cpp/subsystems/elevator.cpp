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

    m_elevatorEncoder.Reset();
    m_elevatorEncoder.SetDistancePerPulse(elevatorConstants::Sensor::Encoder::DISTANCE_PER_PULSE);
    m_elevatorEncoder.SetReverseDirection(elevatorConstants::Sensor::Encoder::REVERSED);

    m_elevatorPID.SetTolerance(elevatorConstants::PID::TOLERANCE);
    m_elevatorPID.Reset(elevatorConstants::PID::SETPOINT);
    m_elevatorPID.SetOutputLimits(elevatorConstants::Speed::MIN_SPEED, elevatorConstants::Speed::MAX_SPEED);

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
  // ----------------- Save sensors value -----------------
  m_isBottomLimitSwitchTriggered = m_elevatorBottomLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED;
  m_isTopLimitSwitchTriggered = m_elevatorTopLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED; 
  m_elevatorHeight = (m_elevatorEncoder.GetDistance() /*- m_EncoderDriftFilter.get()*/);
    assert((m_elevatorHeight >= elevatorPositionMapping[0][0] && m_elevatorHeight <= elevatorPositionMapping[6][1])&& "distance is out of range");
    
  // ----------------- Actualize current Position -----------------
  // Updates the current position of the elevator based on the state of the limit switches and encoder distance.
  if(m_isBottomLimitSwitchTriggered)
    m_state = SET_ELEVATOR_REST_AT_POSITION(m_state, elevatorConstants::State::L1);
  else if(m_isTopLimitSwitchTriggered)
    m_state = SET_ELEVATOR_REST_AT_POSITION(m_state, elevatorConstants::State::L4);
  else {
      for (u_int i = 0; i < std::size(elevatorPositionMapping); i++) {
        if (m_elevatorHeight >= elevatorPositionMapping[i][0] && m_elevatorHeight <= elevatorPositionMapping[i][1]) {
          m_state = SET_ELEVATOR_CURRENT_POSITION(m_state, i);
          break;
        }
      }
  }

  // ----------------- Updates Machine State -----------------
  switch (GET_ELEVATOR_MOVING_TYPE(m_state)) {
    case elevatorConstants::State::Rest:
      if(GET_ELEVATOR_CURRENT_POSITION(m_state) == GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        SetDesiredSetpoint();
      }
      else if(GET_ELEVATOR_CURRENT_POSITION(m_state) < GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Up);
        SetDesiredSetpoint();
        m_state = SET_UPPER_ELEVATOR_POSITION(m_state);
      }
      else /*if(GET_ELEVATOR_CURRENT_POSITION(m_state) > GET_ELEVATOR_DESIRED_POSITION(m_state))*/ {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Down);
        SetDesiredSetpoint();
        m_state = SET_LOWER_ELEVATOR_POSITION(m_state);
      }
      break;
    case elevatorConstants::State::Up:
      if(GET_ELEVATOR_CURRENT_POSITION(m_state) == GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
        SetDesiredSetpoint();
      }
      else if(GET_ELEVATOR_CURRENT_POSITION(m_state) > GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Down);
        SetDesiredSetpoint();
        m_state = SET_LOWER_ELEVATOR_POSITION(m_state);
      }
      break;
    case elevatorConstants::State::Down:
      if(GET_ELEVATOR_CURRENT_POSITION(m_state) == GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Rest);
        SetDesiredSetpoint();
      }
      else if(GET_ELEVATOR_CURRENT_POSITION(m_state) < GET_ELEVATOR_DESIRED_POSITION(m_state)) {
        m_state = SET_ELEVATOR_MOVING_TYPE(m_state, elevatorConstants::State::Up);
        SetDesiredSetpoint();
        m_state = SET_UPPER_ELEVATOR_POSITION(m_state);
      }
      break;
  default:
    assert(false && "void Elevator::Periodic() -> moving type undefined");
    break;
  }

  // ----------------- Dynamic -----------------
  // Executes the dynamic task for controlling the elevator motor.
  assert(m_elevatorHeight > -0.00025 && "descente non voulue");
  double output = m_elevatorPID.Calculate(m_elevatorHeight);
  if(m_elevatorBottomLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED && output < 0.0)
    output = 0.0;
  else if(m_elevatorTopLimitSwitch.Get() == elevatorConstants::Sensor::LimitSwitch::IS_TRIGGERED && output > 0.0)
    output = 0.0;
  m_elevatorMotor.Set(output);
  Dashboard();
}
void Elevator::SetDesiredSetpoint() {
    int i = GET_ELEVATOR_DESIRED_POSITION(m_state);
    assert(!IS_TRANSITION_POSITION(i) && "void Elevator::SetDesiredSetpoint() -> desired position not found");
    m_elevatorPID.SetSetpoint(elevatorPositionMapping[i][2]);
  }
void Elevator::Dashboard() {
  frc::SmartDashboard::PutString("binary state", std::bitset<12>(m_state).to_string());
  frc::SmartDashboard::PutNumber("Elevator Encoder", m_elevatorHeight);
  frc::SmartDashboard::PutNumber("Elevator Current", m_elevatorMotor.GetOutputCurrent());
  // frc::SmartDashboard::PutNumber("Elevator L2 Hall Effect", m_stageL2HallEffectSensor.GetVoltage());
  // frc::SmartDashboard::PutNumber("Elevator L3 Hall Effect", m_stageL3HallEffectSensor.GetVoltage());
  frc::SmartDashboard::PutBoolean("Elevator Bottom Limit Switch", m_elevatorBottomLimitSwitch.Get());
  frc::SmartDashboard::PutBoolean("Elevator Top Limit Switch", m_elevatorTopLimitSwitch.Get());
  frc::SmartDashboard::PutNumber("sortie moteur",m_elevatorMotor.Get());
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

