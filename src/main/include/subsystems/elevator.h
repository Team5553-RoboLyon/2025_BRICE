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
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include "rev/SparkMax.h"
#include <frc/DigitalInput.h>

#include "Constants.h"
#include "lib/UtilsRBL.h"
#include "lib/pid_rbl.h"
#include "lib/NRollingAverage.h"

class Elevator : public frc2::SubsystemBase {
 public:
  void Test(double value);
  Elevator();
  /**
 * @brief Selects the desired stage for the elevator and updates the state.
 *
 * This function takes an integer defined un "Constants.h" representing the desired stage and updates the
 * elevator's state to reflect the desired position for that stage. The stages are
 * defined as follows:
 * - Stage 0: Sets the desired position to m_L1Desired.
 * - Stage 1: Sets the desired position to m_L2Desired.
 * - Stage 2: Sets the desired position to m_L3Desired.
 * - Stage 3: Sets the desired position to m_L4Desired.
 *
 * @param stage The desired stage (0 to 3).
 */
  void SelectWantedStage(u_int16_t stage);
  /**
 * @brief Periodically updates the state of the elevator subsystem.
 * 
 * This function is called periodically and performs the following tasks:
 * - Updates the current position of the elevator.
 * - Update the moving type.
 * - Executes the movement task based on the current state and the desired state.
 * - Updates the dashboard.
 */
  void Periodic() override;

  void setSpeed(double speed);
 private:
  /**
 * @brief Updates the current position of the elevator based on the state of the limit switches and encoder distance.
 * 
 * This function checks the state of the bottom and top limit switches to determine if the elevator is at its lowest
 * or highest position, respectively. If neither limit switch is triggered, it iterates through predefined encoder
 * distance ranges to set the current position of the elevator based on the encoder's distance reading.
 */
  void ActualizeCurrentPosition();
  /**
 * @brief Executes the dynamic task for controlling the elevator motor.
 */
  void ExecuteTask();

  /**
   * @brief Retrieves the current height of the elevator.
   * 
   * @return The current height of the elevator based on the encoder input as a double.
   */
  double GetHeight();
  /**
 * @brief Sets the desired setpoint for the elevator based on the current state.
 *
 * This function retrieves the desired position for the elevator from the current state,
 * checks that the position is not a transition position, and then sets the PID controller's
 * setpoint to the corresponding distance encoder value.
 *
 * @throws std::runtime_error if the desired position is a transition position.
 */
  void SetDesiredSetpoint();
  /**
 * @brief Sets the next current position of the elevator.
 *
 * This function updates the current position of the elevator based on the 
 * direction specified by the parameter `isUpside`. If `isUpside` is true, 
 * the position is incremented. If `isUpside` is false, the position is 
 * decremented. The function ensures that the new position is within the 
 * valid range defined by `elevatorConstants::State::L1` and 
 * `elevatorConstants::State::L4`.
 *
 * @param isUpside A boolean indicating the direction to move the elevator.
 *                 - true: Move the elevator up (increment position).
 *                 - false: Move the elevator down (decrement position).
 *
 * @throws Assertion failure if the new position is out of the valid range.
 */
  void SetNextCurrentPosition(bool isUpside);
  /**
  * @brief Updates the SmartDashboard with all the relevant information about the elevator (eg. encoder position, state, etc.)
  */
  void Dashboard();
  rev::spark::SparkMax m_elevatorMotor{elevatorConstants::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  //TODO : add verif Hall Fx later
  // frc::AnalogInput m_stageL2HallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_L2};
  // frc::AnalogInput m_stageL3HallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_L3};
  frc::Encoder m_ElevatorEncoder{elevatorConstants::Sensor::Encoder::A_ID, elevatorConstants::Sensor::Encoder::B_ID};
  // frc::DigitalInput m_bottomLimitSwitch{elevatorConstants::Sensor::LimitSwitch::BOTTOM_ID};
  // frc::DigitalInput m_topLimitSwitch{elevatorConstants::Sensor::LimitSwitch::TOP_ID};

  PidRBL m_pid{elevatorConstants::PID::KP, elevatorConstants::PID::KI, elevatorConstants::PID::KD};
  NdoubleRollingAverage m_EncoderDriftFilter{5};

  u_int32_t m_state;
  u_int8_t m_counter;

  double m_distanceEncoder[7][3]   // {min, max, setpoint}
  {
    {0.0, 0.01, 0.006},      // L1 
    {0.01, 0.08, 0.045},     // L1/L2
    {0.08, 0.09, 0.085},     // L2
    {0.09, 0.17, 0.13},      // L2/L3
    {0.17, 0.18, 0.175},     // L3
    {0.18, 0.24, 0.21},      // L3/L4
    {0.24, 0.25, 0.243}      // L4 //max de 0.475
  };
};
// 1 nibble par partie sur un total de 32 bits
// 0000 0000 0000 0000 0000 0000 0000 1111 : Desired Position
// 0000 0000 0000 0000 0000 0000 1111 0000 : Current Position
// 0000 0000 0000 0000 0000 1111 0000 0000 : Moving Type
//
// DESIRED POSITION
// 0000 : L1
// 0010 : L2
// 0100 : L3
// 0110 : L4
//
// CURRENT POSITION
// 0000 : L1
// 0001 : L1/L2
// 0010 : L2
// 0011 : L2/L3
// 0100 : L3
// 0101 : L3/L4
// 0110 : L4
//
// MOVING TYPE
// 0000 : Rest
// 0001 : Up
// 0010 : Down
//
// Masks
// 0000 0000 0000 0000 0000 0000 0000 1111 : Desired Position (0xF)
// 0000 0000 0000 0000 0000 0000 1111 0000 : Current Position (0xF0)
// 0000 0000 0000 0000 0000 1111 0000 0000 : Moving Type (0xF00)
