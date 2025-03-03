// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <bitset>
#include <string>
#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableEntry.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include "rev/SparkMax.h"
#include "Constants.h"
#include "lib/UtilsRBL.h"
#include "lib/pid_rbl.h"

class Elevator : public frc2::SubsystemBase {
 public:
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
  void SelectWantedStage(int8_t stage);
  /**
 * @brief Periodically updates the state of the elevator subsystem.
 * 
 * This function is called periodically and performs the following tasks:
 * - Updates the current position of the elevator.
 * - Checks if the current position is less than the desired position and sets the elevator to move up if true.
 * - Checks if the current position is greater than the desired position and sets the elevator to move down if true.
 * - Checks if the current position is equal to the desired position and sets the elevator to the rest position if true.
 * - Executes the movement task based on the current state and the desired state.
 * - Updates the dashboard.
 */
  void Periodic() override;

 private:
  /**
 * @brief Updates the current position state of the elevator based on the encoder distance.
 * 
 * This function checks the current distance measured by the elevator encoder and updates
 * the elevator's state to reflect its current position. The position is determined by 
 * comparing the encoder distance to predefined levels (L1, L2, L3, L4). 
 * If the encoder distance falls within the tolerance range of a level, the 
 * elevator's state is updated to that level. If the distance falls between two levels, 
 * the state is updated to reflect the intermediate position.
 * 
 * The function performs the following checks in order:
 * @return - If the distance is within the tolerance range of L1, set state to m_AtL1.
 * @return - If the distance is within the tolerance range of L2, set state to m_AtL2.
 * @return - If the distance is within the tolerance range of L3, set state to m_AtL3.
 * @return - If the distance is within the tolerance range of L4, set state to m_AtL4.
 * @return - If the distance is between L1 and L2 (excluding tolerance ranges), set state to m_AtL1L2.
 * @return - If the distance is between L2 and L3 (excluding tolerance ranges), set state to m_AtL2L3.
 * @return - If the distance is between L3 and L4 (excluding tolerance ranges), set state to m_AtL3L4.
 */
  void actualizeCurrentPosition();
  /**
 * @brief Executes the task for the elevator subsystem based on the current state.
 * 
 * This function handles the elevator's movement by setting the appropriate setpoints
 * and controlling the motor based on the desired position and movement type.
 * 
 * The function uses a state machine to determine the desired position and movement type,
 * and then performs the necessary actions to move the elevator to the desired position.
 * 
 * The states and movement types are defined in the elevatorConstants::State namespace.
 * 
 * The function handles the following desired positions:
 * - m_L1Desired: Level 1
 * - m_L2Desired: Level 2
 * - m_L3Desired: Level 3
 * - m_L4Desired: Level 4
 * 
 * The function handles the following movement types:
 * - Rest: The elevator is at rest.
 * - Up: The elevator is moving up.
 * - Down: The elevator is moving down.
 * 
 * The function uses a PID controller to calculate the motor output based on the encoder distance.
 */
  void ExecuteTask();
  /**
   * @brief Sets the elevator state to resting.
   *
   * This function clears the current moving type state of the elevator
   * and sets it to the "Rest" state as defined in the elevatorConstants::State.
   */
  void SetRestPosition();
  /**
 * @brief Sets the elevator state to moving up.
 *
 * This function clears the current moving type state of the elevator
 * and sets it to the "Up" state as defined in the elevatorConstants::State.
 */
  void SetGoingUp();
  /**
 * @brief Sets the elevator state to moving down.
 *
 * This function clears the current moving type state of the elevator
 * and sets it to the "Down" state as defined in the elevatorConstants::State.
 */
  void SetGoingDown();
  /**
  * @brief Updates the SmartDashboard with all the relevant information about the elevator (eg. encoder position, state, etc.)
  */
  void Dashboard();
  rev::spark::SparkMax m_elevatorMotor{elevatorConstants::Motors::ID, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_elevatorMotorConfig;

  frc::AnalogInput m_TopHallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_TOP};
  frc::AnalogInput m_MiddleHallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_MIDDLE};
  frc::AnalogInput m_BottomHallEffectSensor{elevatorConstants::Sensor::HallEffect::ID_BOTTOM};
  frc::Encoder m_ElevatorEncoder{elevatorConstants::Sensor::Encoder::A_ID, elevatorConstants::Sensor::Encoder::B_ID};

  PidRBL m_pid{elevatorConstants::PID::KD, elevatorConstants::PID::KI, elevatorConstants::PID::KD};
  u_int32_t m_state;
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
