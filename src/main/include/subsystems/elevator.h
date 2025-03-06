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

#define SET_UPPER_ELEVATOR_POSITION(state) \
    ([&]() { \
        int i = GET_ELEVATOR_CURRENT_POSITION(state) + 1; \
        assert(i <= elevatorConstants::State::L4 && "void Elevator::SET_UPPER_ELEVATOR_POSITION -> current position not found or impossible"); \
        return SET_ELEVATOR_CURRENT_POSITION(state, i); \
    })()
#define SET_LOWER_ELEVATOR_POSITION(state) \
    ([&]() { \
        int i = GET_ELEVATOR_CURRENT_POSITION(state) - 1; \
        assert(i >= elevatorConstants::State::L1 && "void Elevator::SET_LOWER_ELEVATOR_POSITION -> current position not found or impossible"); \
        return SET_ELEVATOR_CURRENT_POSITION(state, i); \
    })()

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
 * - Saves the sensor values.
 * - Updates the current position of the elevator.
 * - Updates Machine State.
 * - Executes the movement task.
 * - Updates the dashboard.
 */
  void Periodic() override;
  private:
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
  * @brief Updates the SmartDashboard with all the relevant information about the elevator (eg. encoder position, state, etc.)
  */
  void Dashboard();
  rev::spark::SparkMax m_elevatorMotor{elevatorConstants::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  frc::Encoder m_elevatorEncoder{elevatorConstants::Sensor::Encoder::A_ID, elevatorConstants::Sensor::Encoder::B_ID};
  frc::DigitalInput m_elevatorBottomLimitSwitch{elevatorConstants::Sensor::LimitSwitch::BOTTOM_ID};
  frc::DigitalInput m_elevatorTopLimitSwitch{elevatorConstants::Sensor::LimitSwitch::TOP_ID};

  PidRBL m_elevatorPID{elevatorConstants::PID::KP, elevatorConstants::PID::KI, elevatorConstants::PID::KD};
  // NdoubleRollingAverage m_EncoderDriftFilter{5};

  u_int32_t m_state;
  u_int8_t m_counter;
  bool m_isTopLimitSwitchTriggered;
  bool m_isBottomLimitSwitchTriggered;
  double m_elevatorHeight;

  double elevatorPositionMapping[7][3]   // {min, max, setpoint}
  {
    {0.0, 0.01, 0.006},      // L1 
    {0.01, 0.08, 0.045},     // L1/L2
    {0.08, 0.09, 0.085},     // L2
    {0.09, 0.17, 0.13},      // L2/L3
    {0.17, 0.18, 0.175},     // L3
    {0.18, 0.46, 0.21},      // L3/L4
    {0.46, 0.47, 0.465}      // L4 //max de 0.475

    // {0.0, 0.01, 0.},      // L1 
    // {0.01, 0.08, 0.},     // L1/L2
    // {0.08, 0.09, 0.},     // L2
    // {0.09, 0.17, 0.},      // L2/L3
    // {0.17, 0.18, 0.},     // L3
    // {0.18, 0.24, 0.},      // L3/L4
    // {0.24, 0.25, 0.}      // L4
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