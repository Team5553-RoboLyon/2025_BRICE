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
 bool gripperActivated = false;
 bool isInitialized = false;
  Manipulator();
  void SelectWantedStage(ManipulatorConstants::State target);
  void Periodic() override;


  private:
  void Reset();
  void Dashboard();
  
  rev::spark::SparkMax m_elevatorMotor{ManipulatorConstants::Elevator::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  rev::spark::SparkMax m_planetaryMotor{ManipulatorConstants::Planetary::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_planetaryMotorConfig;

  rev::spark::SparkMax m_gripperMotorTop{ManipulatorConstants::Gripper::Motors::ID_TOP, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_gripperMotorTopConfig;
  rev::spark::SparkMax m_gripperMotorBottom{ManipulatorConstants::Gripper::Motors::ID_BOTTOM, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_gripperMotorBottomConfig;

  frc::Encoder m_planetaryEncoder{ManipulatorConstants::Planetary::Sensor::Encoder::A_ID, ManipulatorConstants::Planetary::Sensor::Encoder::B_ID};
  frc::DigitalInput m_elevatorBottomLimitSwitch{ManipulatorConstants::Elevator::Sensor::LimitSwitch::BOTTOM_ID};
  frc::DigitalInput m_elevatorTopLimitSwitch{ManipulatorConstants::Elevator::Sensor::LimitSwitch::TOP_ID};
  frc::DigitalInput m_irBreaker{ManipulatorConstants::Gripper::Sensor::IR_BREAKER_ID};

  PidRBL m_planetaryPID{ManipulatorConstants::Planetary::PID::KP, ManipulatorConstants::Planetary::PID::KI, ManipulatorConstants::Planetary::PID::KD};
  u_int32_t m_manipulatorState;

  u_int32_t m_gripperCounter;
  bool newConsign;

  NSensorThreshold m_a{0, 0, 4};
  NSensorThreshold m_b{1, 0, 4};
  NSensorThreshold m_c{2, 1, 4};

  bool m_isTopLimitSwitchTriggered;
  bool m_isBottomLimitSwitchTriggered;
  double m_planetaryAngle;
  bool m_isIRBreakerTriggered;

  double elevatorOutput = 0.0;
  double planetaryOutput = 0.0;
  


  double m_planetaryPositionMapping[9][3]   // {min, max, setpoint}
  {     
    {-0.030, 0.030, 0.000},        // HOME
    {0.015, 0.770, 0.000},         // HOME / CORAL STATION
    {0.755, 0.815, 0.785},         // CORAL STATION
    {0.800, 3.127, 0.000},         // CORAL STATION / L4
    {3.112, 3.172, 3.142},         // L4
    {3.157, 3.650, 0.000},         // L4/L3
    {3.635, 3.695, 3.665},         // L3
    {3.680, 3.999, 0.000},         // L3/L2
    {3.984, 4.044, 4.014}          // L2
  };	
  
};

  #define DESIRED_STATE_SHIFTING 16
  #define CURRENT_STATE_SHIFTING 0

  #define MAKE_STATE(gripSt, movPla, posPla, movEl, posEl) (((gripSt) <<12) | ((movPla) <<10) | ((posPla) <<6) | ((movEl) <<4) | (posEl))
  #define MAKE_FULL_STATE(desired, current) (((desired) << 16) | (current))

  #define GET_ELEVATOR_STATE(bits, stateID) (((bits) >> (stateID)) & (ManipulatorConstants::mask::maskElevatorState))
  #define GET_ELEVATOR_POSITION(bits, stateID) (((bits) >> (stateID)) & (ManipulatorConstants::mask::maskElevatorPosition))
  #define GET_ELEVATOR_MOVING_TYPE(bits, stateID) ((((bits) >> (stateID)) & (ManipulatorConstants::mask::maskElevatorMovingType)) >> 4)
  #define GET_PLANETARY_STATE(bits, stateID) (((bits) >> (stateID)) & (ManipulatorConstants::mask::maskPlanetaryState) >> 6)
  #define GET_PLANETARY_POSITION(bits, stateID) ((((bits) >> (stateID)) & (ManipulatorConstants::mask::maskPlanetaryPosition)) >> 6)
  #define GET_PLANETARY_MOVING_TYPE(bits, stateID) ((((bits) >> (stateID)) & (ManipulatorConstants::mask::maskPlanetaryMovingType)) >> 10)
  #define GET_GRIPPER_STATE(bits, stateID) ((((bits) >> (stateID)) & (ManipulatorConstants::mask::maskGripperState)) >> 12)
  #define GET_STATE(bits, stateID) (((bits) >> (stateID)) & (ManipulatorConstants::mask::maskState))

  #define SET_ELEVATOR_STATE(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskElevatorState) << (stateID)))) | ((setValue) << (stateID))))
  #define SET_ELEVATOR_POSITION(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskElevatorPosition) << (stateID)))) | ((setValue) << (stateID))))
  #define SET_ELEVATOR_MOVING_TYPE(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskElevatorMovingType) << (stateID)))) | (((setValue) << 4) <<(stateID))))
  #define SET_PLANETARY_STATE(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskPlanetaryState) << (stateID)))) | (((setValue) << 6) << (stateID))))
  #define SET_PLANETARY_POSITION(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskPlanetaryPosition) << (stateID)))) | (((setValue) << 6) << (stateID))))
  #define SET_PLANETARY_MOVING_TYPE(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskPlanetaryMovingType) << (stateID)))) | (((setValue) << 10) <<(stateID))))
  #define SET_GRIPPER_STATE(bits, stateID, setValue) ((((bits) & (~((ManipulatorConstants::mask::maskGripperState) << (stateID)))) | (((setValue) << 12) <<(stateID))))

  #define IS_AT_DESIRED_STATE(bits) (((bits) >> 16) == ((bits) & (ManipulatorConstants::mask::maskState)))
  #define IS_ELEVATOR_AT_POSITION(bits, posValue) (((((bits) >> (CURRENT_STATE_SHIFTING)) & (ManipulatorConstants::mask::maskElevatorPosition)) == (posValue)) && ((((bits) >> (DESIRED_STATE_SHIFTING)) & (ManipulatorConstants::mask::maskElevatorPosition)) == (posValue)) && (((((bits) >> (CURRENT_STATE_SHIFTING))) & (ManipulatorConstants::mask::maskElevatorMovingType)) == (ManipulatorConstants::Elevator::State::REST)))
  #define IS_PLANETARY_AT_POSITION(bits, posValue) ((((((bits) >> (CURRENT_STATE_SHIFTING)) & (ManipulatorConstants::mask::maskPlanetaryPosition)) >> 6) == (posValue)) && (((((bits) >> (DESIRED_STATE_SHIFTING)) & (ManipulatorConstants::mask::maskPlanetaryPosition)) >> 6) == (posValue)))
  #define IS_READY_TO_CATCH(bits) ((IS_ELEVATOR_AT_POSITION((bits), (ManipulatorConstants::Elevator::State::CORALSTATION))) && (IS_PLANETARY_AT_POSITION((bits), (ManipulatorConstants::Planetary::State::CORALSTATION))))
  

// ***************************************LE PHARE DANS LA NUIT*******************************************²
// ELEVATOR POSITION
// 0000 : L2
// 0001 : L2/CORAL STATION
// 0010 : CORAL STATION
// 0011 : CORAL STATION/L3
// 0100 : L3
// 0101 : L3/L4
// 0110 : L4

// MOVING TYPE ELEVATOR
// 00 : REST
// 01 : UP
// 10 : DOWN
// 11 : WAITING
//
// PLANETARY POSITION 
// 0000 : HOME
// 0001 : HOME/CORAL STATION
// 0010 : CORAL STATION
// 0011 : CORAL STATION/L4
// 0100 : L4
// 0101 : L4/L3
// 0110 : L3
// 0111 : L3/L2
// 1000 : L2
//
// MOVING TYPE PLANETARY
// 00 : REST
// 01 : Clockwise
// 10 : Counter Clockwise
// 11 : WAITING
//
// GRIPPER STATE
// 0000 : CATCHING ->
// 0001 : CAUGHT MIDDLE -
// 0010 : MOVE TO BACK ->
// 0011 : CAUGHT BACK -
// 0100 : MOVE TO MIDDLE <-
// 0101 : DROPPING <-
// 0110 : DROPPED -

// 0000 : REST
// 0001 : CATCHING
// 0010 : DROPPING
// ***************************************************************************************
// **					Manipulator state													**
// ***************************************************************************************
//
//     State (16 bits): 
//		- Position Elevator     (4 bits)
//		- Moving Type Elevator  (2 bits)
//		- Position Planetary    (4 bits)
//		- Moving Type Planetary (2 bits)
//		- Gripper state         (4 bits)
//
//    Full State (32 bits) :
//    - Current state (16 bits)
//    - Desired state (16 bits)
//    
//
// |___________|___________|___________|___________|___________|___________|___________|___________|
// |31|30|29|28|27|26|25|24|23|22|21|20|19|18|17|16|15|14|13|12|11|10|09|08|07|06|05|04|03|02|01|00|
// | Gripper   | Mov.| Position  | Mov.| Position  | Gripper   | Mov.| Postion   | Mov.| Position  |
// | State     | Pla.| Planetary | El. | Elevator  | State     | Pla.| Planetary | El. | Elevator  |
// |-----------|-----|-----------|-----|-----------|-----------|-----|-----------|-----|-----------|
// | 4 bits    | 2b. | 4 bits    | 2b. | 4 bits    | 4 bits    | 2b. | 4 bits    | 2b. | 4 bits    |
// |-----------|-----|-----------|-----|-----------|-----------|-----|-----------|-----|-----------|
// | Gripper   | Planetary       | Elevator        | Gripper   | Planetary       | Elevator        |
// |-----------------------------------------------|-----------------------------------------------|
// |         DESIRED STATE                         |              CURRENT STATE                    |
// |-----------------------------------------------------------------------------------------------|
// |                                           FULL STATE                                          |
// -------------------------------------------------------------------------------------------------