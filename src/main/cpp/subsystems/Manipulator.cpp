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

    m_gripperMotorTopConfig
                .VoltageCompensation(ManipulatorConstants::Gripper::Motors::VOLTAGE_COMPENSATION)
                .Inverted(ManipulatorConstants::Gripper::Motors::INVERTED)
                .SmartCurrentLimit(ManipulatorConstants::Gripper::Motors::CURRENT_LIMIT)
                .ClosedLoopRampRate(ManipulatorConstants::Gripper::Motors::RAMP_RATE)
                .SetIdleMode(ManipulatorConstants::Gripper::Motors::IDLE_MODE);

    m_gripperMotorTop.Configure(
                m_gripperMotorTopConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    
    m_gripperMotorBottomConfig.Apply(m_gripperMotorTopConfig)
                              .Follow(m_gripperMotorTop, true);
    
    m_gripperMotorBottom.Configure(
                m_gripperMotorBottomConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_planetaryEncoder.Reset();
    m_planetaryEncoder.SetDistancePerPulse(ManipulatorConstants::Planetary::Sensor::Encoder::DISTANCE_PER_PULSE);
    m_planetaryEncoder.SetReverseDirection(ManipulatorConstants::Planetary::Sensor::Encoder::REVERSED); // add Hall Fx sensor

    m_planetaryPID.SetTolerance(ManipulatorConstants::Planetary::PID::TOLERANCE);
    m_planetaryPID.Reset(ManipulatorConstants::Planetary::PID::SETPOINT);
    m_planetaryPID.SetOutputLimits(ManipulatorConstants::Planetary::Speed::MIN, ManipulatorConstants::Planetary::Speed::MAX);

    m_a.SetThresholds(2.6, 3.0);
    m_b.SetThresholds(2.6, 3.0);
    m_c.SetThresholds(2.6, 3.0);
    m_b.MoveUp();

    u_int16_t ManipulatorInitState = MAKE_STATE( ManipulatorConstants::Gripper::State::REST, 
                                                    ManipulatorConstants::Planetary::State::REST, 
                                                    ManipulatorConstants::Planetary::State::HOME, 
                                                    ManipulatorConstants::Elevator::State::REST, 
                                                    ManipulatorConstants::Elevator::State::L2);
    m_manipulatorState = MAKE_FULL_STATE(ManipulatorInitState, ManipulatorInitState);
}
void Manipulator::Reset() {
  if(m_elevatorBottomLimitSwitch.Get() == ManipulatorConstants::Elevator::Sensor::LimitSwitch::IS_TRIGGERED) {
    m_elevatorMotor.Set(ManipulatorConstants::Elevator::Speed::REST);
    isInitialized = true;
    m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
    m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L2);
    m_b.ResetCounter(1,1, NSensorThreshold::State::kHigh);
  }
  else {
      m_elevatorMotor.Set(ManipulatorConstants::Elevator::Speed::CALIBRATION);
  }
}

void Manipulator::SelectWantedStage(ManipulatorConstants::State target) {
  switch (target) 
  {
    case ManipulatorConstants::State::Home:
      newConsign = true;
        m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::CORALSTATION);
        m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::HOME);
        // m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Gripper::State::DROPPED);
      break;
  case ManipulatorConstants::State::L2:
    newConsign = true;
    m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L2);
    m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::L2);
    // m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Gripper::State::CAUGHT_MIDDLE);
    break;
  case ManipulatorConstants::State::CoralStation:
    newConsign = true;
    m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::CORALSTATION);
    m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::CORALSTATION);
    // m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Gripper::State::CATCHING);
    break;
  case ManipulatorConstants::State::L3:
    newConsign = true;
    m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L3);
    m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::L3);
    // m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Gripper::State::CAUGHT_MIDDLE);
    break;
  case ManipulatorConstants::State::L4:
    newConsign = true;
    m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L4);
    m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::L4);
    //  m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Gripper::State::CAUGHT_BACK);
    break;

  default:
    newConsign = false;
    break;
  }
    frc::SmartDashboard::PutBoolean("new consign", newConsign);
}

void Manipulator::Periodic() {
  if(!isInitialized) {
      Reset();
      return;
  }
  // ----------------- Save sensors value -----------------
      // ELEVATOR
  m_isBottomLimitSwitchTriggered = m_elevatorBottomLimitSwitch.Get() == ManipulatorConstants::Elevator::Sensor::LimitSwitch::IS_TRIGGERED;
  m_isTopLimitSwitchTriggered = m_elevatorTopLimitSwitch.Get() == ManipulatorConstants::Elevator::Sensor::LimitSwitch::IS_TRIGGERED; 
  //     // PLANETARY
  m_planetaryAngle = std::fmod(m_planetaryEncoder.GetDistance(), 2*M_PI);
  //     // GRIPPER
  // m_isIRBreakerTriggered = m_irBreaker.Get() == ManipulatorConstants::Gripper::Sensor::IS_TRIGGERED;





  // ----------------- Updates Current Position -----------------
      // ELEVATOR
  if(m_isTopLimitSwitchTriggered && (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::UP)) {
      elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
      m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
      m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L4);
      m_b.ResetCounter(4,-1, NSensorThreshold::State::kHigh);
  
  }
  else if(m_isBottomLimitSwitchTriggered && (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::DOWN)) {
      elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
      m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
      m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L2);
      m_b.ResetCounter(1,1, NSensorThreshold::State::kHigh);
  }
  switch(m_b.GetPosition()) {
    case 1:
      m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L2);
      break;
    case 2:
     m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::CORALSTATION);
      break;
    case 3:
     m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L3);
      break;
    case 4 :
     m_manipulatorState = SET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::L4);
      break;
    default:
      // assert(false && "Elevator position is out of bounds");
      break; 
  }

      // PLANETARY
  if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Planetary::State::CLOCKWISE) {
    if(m_planetaryAngle > m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING)][1]) {
      m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) + 1);
      // assert((GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) <= ManipulatorConstants::Planetary::State::L2) && "Planetary n+1 mais current position > L2");
    }
    // assert(m_planetaryAngle >= m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING)][0] && "Planetary up mais current position = n-1");
  }
  else if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE) {
    if(m_planetaryAngle < m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING)][0]) {
      m_manipulatorState = SET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING, GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) - 1);
      // assert((GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) >= ManipulatorConstants::Planetary::State::HOME) && "Planetary n-1 mais current position < HOME");
    }
    // assert(m_planetaryAngle <= m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING)][1] && "Planetaru down mais current position = n+1");
  }
  else /*if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Planetary::State::REST || ...::WAITING) */{
    // assert((m_planetaryAngle >= m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING)][0] && m_planetaryAngle <= m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING)][1]) && "planetary rest mais pas au bon endroit");
  }

  // ----------------- Updates Desired Moving Type  -----------------
      // ELEVATOR
  if(newConsign) 
  {
    newConsign = false;

    if(GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) < GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) 
    {
      m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::UP);
      m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::WAITING);
    } 
    else if(GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) > GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) 
    {
      m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::DOWN);
      m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::WAITING);
    }
    else /* if(GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) == GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) */ 
    {
      if(GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::UP)
      {
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::DOWN);
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::WAITING);
      }  
      else if(GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::DOWN) 
      {
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::UP);
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::WAITING);
      }
      else /*if(GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::REST)*/ 
      {
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
        elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
      }
    }
  }
      // PLANETARY
  if(GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) < GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) 
    m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::CLOCKWISE);
  else if(GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) > GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) 
    m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE);
  else /* if(GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) == GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) */
    m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING, ManipulatorConstants::Planetary::State::REST);
  
  // ----------------- Switches Machine State -----------------
    //ELEVATOR
  switch (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING))
  { 
    case ManipulatorConstants::Elevator::State::REST:
      elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
    break;

    case ManipulatorConstants::Elevator::State::UP:
    m_a.Update();
    m_b.Update();
   m_c.Update();
      if(GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) == GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) 
      {
        if(m_b.IsHigh()) 
        {
          elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
          m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
        }
      }
    break;

    case ManipulatorConstants::Elevator::State::DOWN:
    m_a.Update();
    m_b.Update();
    m_c.Update();
      if (GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING) == GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)) 
      {
        if(m_b.IsHigh()) 
        {
          elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
          m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
        }
      }
      break;

    case ManipulatorConstants::Elevator::State::WAITING:
      elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
      if(GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::UP) 
      {
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::UP);
        elevatorOutput = ManipulatorConstants::Elevator::Speed::UP;
        m_a.MoveUp();
        m_b.MoveUp();
        m_c.MoveUp();
      }
      else if(GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::DOWN)
      {
        if(IS_PLANETARY_AT_POSITION(m_manipulatorState, GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING))) 
        {
          m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::DOWN);
          elevatorOutput = ManipulatorConstants::Elevator::Speed::DOWN;
          m_a.MoveDown();
          m_b.MoveDown();
          m_c.MoveDown();
        }
      }
      else /*if(GET_ELERVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Elevator::State::REST)*/ {
        m_manipulatorState = SET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Elevator::State::REST);
        elevatorOutput = ManipulatorConstants::Elevator::Speed::REST;
      }
      break;
    default:
      // assert(false && "void Manipulator::Periodic() -> moving type elevator undefined");
      break;
  } // switch (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING))

  //PLANETARY
  switch (GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING))
  {
    case ManipulatorConstants::Planetary::State::CLOCKWISE:
    case ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE: // Same behavior for these 3 states
      m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Gripper::State::REST);
      planetaryOutput = m_planetaryPID.Calculate(m_planetaryAngle);
      if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) != GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING))
      {
        m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Planetary::State::WAITING);
        planetaryOutput = ManipulatorConstants::Planetary::Speed::REST;
      }
      break;

      case ManipulatorConstants::Planetary::State::REST:
        planetaryOutput = m_planetaryPID.Calculate(m_planetaryAngle);
        if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) != GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING))
        {
          m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Planetary::State::WAITING);
          planetaryOutput = ManipulatorConstants::Planetary::Speed::REST;
        }
        else 
        {
          switch (GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING))
          {
            case ManipulatorConstants::Elevator::State::CORALSTATION:
              m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Gripper::State::CATCHING);
              break;
            case ManipulatorConstants::Elevator::State::L2:
            case ManipulatorConstants::Elevator::State::L3:
            case ManipulatorConstants::Elevator::State::L4:
              m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Gripper::State::DROPPING);
          default:
            break;
          }
        }
        break;
    
    case ManipulatorConstants::Planetary::State::WAITING:
      planetaryOutput = ManipulatorConstants::Planetary::Speed::REST;
      if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Planetary::State::CLOCKWISE)
       {
        if(IS_ELEVATOR_AT_POSITION(m_manipulatorState, GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)))
        {
          m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Planetary::State::CLOCKWISE);
          m_planetaryPID.SetSetpoint(m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)][2]);
        }
      }
      else if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE)
      {
        m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE);
        m_planetaryPID.SetSetpoint(m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)][2]);
      }
      else /*if(GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Planetary::State::REST)*/ 
      {
        m_manipulatorState = SET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Planetary::State::REST);
        m_planetaryPID.SetSetpoint(m_planetaryPositionMapping[GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)][2]);
      }
      break;

    default:
      // assert(false && "void Manipulator::Periodic() -> moving type planetary undefined");
      break;
  } // switch (GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING))

  //GRIPPER
  frc::SmartDashboard::PutBoolean("gripper activated", gripperActivated);
  if(!gripperActivated){
    m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Gripper::State::REST);
  }
    switch(GET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING))
    { 
      #define TIMEOUT 100
        case ManipulatorConstants::Gripper::State::CATCHING:
        if(m_isIRBreakerTriggered) {
          m_gripperMotorTop.Set(ManipulatorConstants::Gripper::Speed::REST);
        }
        else {
          m_gripperCounter++;
          if(m_gripperCounter > TIMEOUT)
            m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Gripper::State::REST);
          else {
            m_gripperMotorTop.Set(ManipulatorConstants::Gripper::Speed::CATCH);
          }
        }
        break;
    
      case ManipulatorConstants::Gripper::State::DROPPING:
        m_gripperCounter++;
        if(m_gripperCounter > TIMEOUT)
          m_manipulatorState = SET_GRIPPER_STATE(m_manipulatorState, CURRENT_STATE_SHIFTING, ManipulatorConstants::Gripper::State::REST);
        else {
          m_gripperMotorTop.Set(ManipulatorConstants::Gripper::Speed::DROP);
        }
        break;
      case ManipulatorConstants::Gripper::State::REST:
          m_gripperCounter = 0;
          m_gripperMotorTop.Set(0.0);
        break;
      default:
        break;
    }

 

  // ----------------- Dynamic -----------------
  // Executes the dynamic task for controlling the elevator motor.

  if(m_isTopLimitSwitchTriggered && (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) && elevatorOutput > 0.0)) {
    // assert(false && "void Manipulator::Periodic() -> elevator top limit switch triggered while going up");
    elevatorOutput = 0.0;
  }
  else if(m_isBottomLimitSwitchTriggered && (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING) && elevatorOutput < 0.0)) {
    // assert(false && "void Manipulator::Periodic() -> elevator bottom limit switch triggered while going down");
    elevatorOutput = 0.0;
  }
  m_elevatorMotor.Set(elevatorOutput);
  m_planetaryMotor.Set(planetaryOutput);
  Dashboard();
}

void Manipulator::Dashboard() {
  frc::SmartDashboard::PutBoolean("macro elevator", IS_ELEVATOR_AT_POSITION(m_manipulatorState, GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING)));
  // ----------------- Hall Fx --------------
  frc::SmartDashboard::PutBoolean("new consign", newConsign);
  
  frc::SmartDashboard::PutNumber("A Sensor Counter", m_a.GetPosition());
  frc::SmartDashboard::PutNumber("B Sensor Counter", m_b.GetPosition());
  frc::SmartDashboard::PutNumber("C Sensor Counter", m_c.GetPosition());

  frc::SmartDashboard::PutNumber("A Sensor", m_a.GetSensorValue());
  frc::SmartDashboard::PutNumber("B Sensor", m_b.GetSensorValue());
  frc::SmartDashboard::PutNumber("C Sensor", m_c.GetSensorValue());

    frc::SmartDashboard::PutNumber("A Move", m_a.GetSens());
  frc::SmartDashboard::PutNumber("B Move", m_b.GetSens());
  frc::SmartDashboard::PutNumber("C Move", m_c.GetSens());

  frc::SmartDashboard::PutBoolean("B is High", m_b.IsHigh());
  // ----------------- Manipulator -----------------
  frc::SmartDashboard::PutString("binary state", std::bitset<32>(m_manipulatorState).to_string());
  // ----------------- Elevator -----------------
  // frc::SmartDashboard::PutNumber("Elevator Encoder", m_elevatorHeight);
  frc::SmartDashboard::PutNumber("Elevator Current", m_elevatorMotor.GetOutputCurrent());
  frc::SmartDashboard::PutBoolean("Elevator Bottom Limit Switch", m_elevatorBottomLimitSwitch.Get());
  frc::SmartDashboard::PutBoolean("Elevator Top Limit Switch", m_elevatorTopLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Elevator motor Output",m_elevatorMotor.Get());
  // // ----------------- Planetary -----------------
  frc::SmartDashboard::PutNumber("Planetary Encoder", m_planetaryAngle);
  frc::SmartDashboard::PutNumber("Planetary Current", m_planetaryMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Planetary motor Output",m_planetaryMotor.Get());
  // ----------------- Gripper -----------------
  // frc::SmartDashboard::PutNumber("gripper output", m_gripperMotorBottom.Get());
  // frc::SmartDashboard::PutBoolean("GET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Gripper::State::CATCHING", GET_GRIPPER_STATE(m_manipulatorState, DESIRED_STATE_SHIFTING) == ManipulatorConstants::Gripper::State::CATCHING);
  // frc::SmartDashboard::PutBoolean("bool", IS_READY_TO_CATCH(m_manipulatorState));
  // frc::SmartDashboard::PutBoolean("IR Breaker",m_isIRBreakerTriggered);
  switch (GET_PLANETARY_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING))
  {
  case ManipulatorConstants::Planetary::State::CORALSTATION:
    frc::SmartDashboard::PutString("Desired P Position", "CORALSTATION");
    break;
  case ManipulatorConstants::Planetary::State::HOME:
    frc::SmartDashboard::PutString("Desired P Position", "HOME");
    break;
  case ManipulatorConstants::Planetary::State::L4:
    frc::SmartDashboard::PutString("Desired P Position", "L4");
    break;
  case ManipulatorConstants::Planetary::State::L3:
    frc::SmartDashboard::PutString("Desired P Position", "L3");
    break;
  case ManipulatorConstants::Planetary::State::L2:
    frc::SmartDashboard::PutString("Desired P Position", "L2");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> desired P position not found ");
    break;
  }
  switch(GET_ELEVATOR_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING))
  { 
  case ManipulatorConstants::Elevator::State::L2:
    frc::SmartDashboard::PutString("Current E Position", "L2");
    break;
  case ManipulatorConstants::Elevator::State::L2_CORALSTATION:
    frc::SmartDashboard::PutString("Current E Position", "L2 CORALSTATION");
    break;
  case ManipulatorConstants::Elevator::State::CORALSTATION:
    frc::SmartDashboard::PutString("Current E Position", "CORALSTATION");
    break;
  case ManipulatorConstants::Elevator::State::CORALSTATION_L3:
    frc::SmartDashboard::PutString("Current E Position", "CORALSTATION L3");
    break;
  case ManipulatorConstants::Elevator::State::L3:
    frc::SmartDashboard::PutString("Current E Position", "L3");
    break;
  case ManipulatorConstants::Elevator::State::L3_L4:
    frc::SmartDashboard::PutString("Current E Position", "L3 L4");
    break;
  case ManipulatorConstants::Elevator::State::L4:
    frc::SmartDashboard::PutString("Current E Position", "L4");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> desired E moving type not found ");
    break;
  }
    switch(GET_ELEVATOR_POSITION(m_manipulatorState, DESIRED_STATE_SHIFTING))
  { 
  case ManipulatorConstants::Elevator::State::L2:
    frc::SmartDashboard::PutString("Desired E Position", "L2");
    break;
  case ManipulatorConstants::Elevator::State::L2_CORALSTATION:
    frc::SmartDashboard::PutString("Desired E Position", "L2 CORALSTATION");
    break;
  case ManipulatorConstants::Elevator::State::CORALSTATION:
    frc::SmartDashboard::PutString("Desired E Position", "CORALSTATION");
    break;
  case ManipulatorConstants::Elevator::State::CORALSTATION_L3:
    frc::SmartDashboard::PutString("Desired E Position", "CORALSTATION L3");
    break;
  case ManipulatorConstants::Elevator::State::L3:
    frc::SmartDashboard::PutString("Desired E Position", "L3");
    break;
  case ManipulatorConstants::Elevator::State::L3_L4:
    frc::SmartDashboard::PutString("Desired E Position", "L3 L4");
    break;
  case ManipulatorConstants::Elevator::State::L4:
    frc::SmartDashboard::PutString("Desired E Position", "L4");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> desired E moving type not found ");
    break;
  }
  switch(GET_PLANETARY_POSITION(m_manipulatorState, CURRENT_STATE_SHIFTING))
  { 
  case ManipulatorConstants::Planetary::State::CORALSTATION:
    frc::SmartDashboard::PutString("Current P Position", "CoralStation");
    break;
  case ManipulatorConstants::Planetary::State::HOME_CORALSTATION:
    frc::SmartDashboard::PutString("Current P Position", "Home CoralStation");
    break;
  case ManipulatorConstants::Planetary::State::HOME:
    frc::SmartDashboard::PutString("Current P Position", "Home");
    break;
  case ManipulatorConstants::Planetary::State::CORALSTATION_L4:
    frc::SmartDashboard::PutString("Current P Position", "CS L4");
    break;
  case ManipulatorConstants::Planetary::State::L4:
    frc::SmartDashboard::PutString("Current P Position", "L4");
    break;
  case ManipulatorConstants::Planetary::State::L4_L3:
    frc::SmartDashboard::PutString("Current P Position", "L4 L3");
    break;
  case ManipulatorConstants::Planetary::State::L3:
    frc::SmartDashboard::PutString("Current P Position", "L3");
    break;
  case ManipulatorConstants::Planetary::State::L3_L2:
    frc::SmartDashboard::PutString("Current P Position", "L3 L2");
    break;
  case ManipulatorConstants::Planetary::State::L2:
    frc::SmartDashboard::PutString("Current P Position", "L2");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> current P position not found ");
    break;
  }
  switch (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING))
  {
  case ManipulatorConstants::Elevator::State::REST:
    frc::SmartDashboard::PutString("Current E Moving Type", "REST");
    break;
  case ManipulatorConstants::Elevator::State::UP:
    frc::SmartDashboard::PutString("Current E Moving Type", "UP");
    break;
  case ManipulatorConstants::Elevator::State::DOWN:
    frc::SmartDashboard::PutString("Current E Moving Type", "DOWN");
    break;
  case ManipulatorConstants::Elevator::State::WAITING:
    frc::SmartDashboard::PutString("Current E Moving Type", "WAITING");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> current E moving type not found ");
    break;
  }
    switch (GET_ELEVATOR_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING))
  {
  case ManipulatorConstants::Elevator::State::REST:
    frc::SmartDashboard::PutString("Desired E Moving Type", "REST");
    break;
  case ManipulatorConstants::Elevator::State::UP:
    frc::SmartDashboard::PutString("Desired E Moving Type", "UP");
    break;
  case ManipulatorConstants::Elevator::State::DOWN:
    frc::SmartDashboard::PutString("Desired E Moving Type", "DOWN");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> current E moving type not found ");
    break;
  }
  switch (GET_PLANETARY_MOVING_TYPE(m_manipulatorState, CURRENT_STATE_SHIFTING))
  {
  case ManipulatorConstants::Planetary::State::REST:
    frc::SmartDashboard::PutString("Current P Moving Type", "REST");
    break;
  case ManipulatorConstants::Planetary::State::CLOCKWISE:
    frc::SmartDashboard::PutString("Current P Moving Type", "CLOCKWISE");
    break;
  case ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE:
    frc::SmartDashboard::PutString("Current P Moving Type", "COUNTER_CLOCKWISE");
    break;
  case ManipulatorConstants::Planetary::State::WAITING:
    frc::SmartDashboard::PutString("Current P Moving Type", "WAITING");
    break;
  default:
    // assert(false  && "void Manipulator::Dashboard() -> current P moving type not found ");
    break;
  }
  switch (GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING))
  {
  case ManipulatorConstants::Planetary::State::REST:
    frc::SmartDashboard::PutString("desired P Moving Type", "REST");
    break;
  case ManipulatorConstants::Planetary::State::CLOCKWISE:
    frc::SmartDashboard::PutString("desired P Moving Type", "CLOCKWISE");
    break;
  case ManipulatorConstants::Planetary::State::COUNTER_CLOCKWISE:
    frc::SmartDashboard::PutString("desired P Moving Type", "COUNTER_CLOCKWISE");
    break;
  default:
    std::cout << GET_PLANETARY_MOVING_TYPE(m_manipulatorState, DESIRED_STATE_SHIFTING) << std::endl;
    // assert(false  && "void Manipulator::Dashboard() -> desired P moving type not found ");
    break;
  }
}

