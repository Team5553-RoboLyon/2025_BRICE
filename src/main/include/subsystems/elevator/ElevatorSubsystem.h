// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "ElevatorConstants.h"
#include "ElevatorIOLogger.h"
#include "ElevatorIO.h"
#include "Constants.h"

#include "lib/RateLimiter.h"
#include "lib/pidRBL.h"
#include "lib/Alert.h"

class ElevatorSubsystem : public frc2::SubsystemBase {
  public:
    ElevatorSubsystem(ElevatorIO *pIO);

    enum class WantedState 
    {
      STAND_BY, // no wanted state scheduled. (It's all good man, it's all good !)
      L1,
      L2,
      L3,
      L4,
      CORAL_STATION,
      HOME,
      VISION_POSITION,
      INITIALIZATION
    };
    enum class SystemState
    {
      IDLE,
      //Steady states
      AT_L1,
      AT_L2,
      AT_L3,
      AT_L4,
      AT_STATION,
      AT_HOME,
      AT_VISION,
      //Transition state
      MOVING_TO_L1,
      MOVING_TO_L2,
      MOVING_TO_L3,
      MOVING_TO_L4,
      MOVING_TO_STATION,
      MOVING_TO_HOME,
      MOVING_TO_VISION
    };
    void SetWantedState(const WantedState wantedState);
    SystemState GetSystemState();
    void SetControlMode(const ControlMode mode);
    ControlMode GetControlMode();
    void ToggleControlMode();

    bool IsResting();
    bool IsInitialized() { return m_isInitialized; } //COMMENTME
    void SetManualAxis(const double value);

    double GetHeight() const { return inputs.heightPosition; } //COMMENTME

    void Periodic() override;
  private:
    // === Hardware & IO Interfaces ===
      ElevatorIO *m_pElevatorIO;
      ElevatorIOInputs inputs;
      ElevatorIOLogger m_logger{frc::DataLogManager::GetLog(), "/Elevator"};
    // === System States & Control Modes ===
      WantedState m_wantedState = WantedState::STAND_BY;
      WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
      SystemState m_systemState = SystemState::IDLE;
      ControlMode m_controlMode = elevatorConstants::MainControlMode;
    // === Motion Control (PID / Filters) ===
      PidRBL m_elevatorPIDController;
      RateLimiter m_rateLimiter{elevatorConstants::Settings::TIME_TO_REACH_FULL_SPEED};
    // === Control Inputs / Outputs ===
      double m_output{0.0};
      double m_manualControlInput{0.0};
      double m_timestamp{0.0};
    // === Status Flags ===
      bool m_isInitialized = false;
      bool m_isEncoderAlreadyReset = false;
    // === System Alerts ===
      Alert m_leftMotorDisconnected{"Elevator Left Motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_rightMotorDisconnected{"Elevator Right Motor: Disconnected", Alert::AlertType::ERROR};
      Alert m_leftMotorHot{"Elevator Left Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_rightMotorHot{"Elevator RightMotor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
      Alert m_leftMotorOverheating{"Elevator Left Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
      Alert m_rightMotorOverheating{"Elevator Right Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    // === Internal Methods ===
      void RunStateMachine();
};