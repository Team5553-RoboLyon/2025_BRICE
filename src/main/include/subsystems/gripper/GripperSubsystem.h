#pragma once

#include <frc2/command/SubsystemBase.h>

#include "GripperConstants.h"
#include "GripperIOLogger.h"
#include "GripperIO.h"
#include "Constants.h"

#include "lib/Alert.h"

class GripperSubsystem : public frc2::SubsystemBase {
 public:
  GripperSubsystem(GripperIO *pIo);

  enum class WantedState 
  {
    STAND_BY, // no wanted state scheduled. (It's all good man, it's all good !)
    LOAD, 
    SCORE_HIGH, // "ROBO'LYON SCORES ON L-FOUUUUUUUUUUUUUURRRRRRRR !!!" commentator at LA
    SCORE_MIDDLE,
    SCORE_LOW,
    TOGGLE,
    SHIFT_FRONT,
    REJECT_BACKWARD,
    REJECT_FORWARD
  };
  enum class SystemState
  {
    IDLE,
    //Steady states
    REST_EMPTY,
    REST_LOADED,
    REST_SHIFTED,
    //Transition state
    COLLECTING_EMPTY,
    FEEDING_FORWARD,
    FEEDING_FORWARD_SHY,
    FEEDING_BACKWARD,
    PRESCORE,
    HIGH_SCORING,
    MIDDLE_SCORING,
    LOW_SCORING,
    REJECTING_BACKWARD,
    REJECTING_FORWARD,
    SHIFTING_FORWARD
  };

  void SetWantedState(const WantedState wantedState);
  SystemState GetSystemState();
  void SetControlMode(const ControlMode mode);
  ControlMode GetControlMode();
  void ToggleControlMode();

  bool IsResting();
  void SetManualAxis(const double value);
  void Periodic() override;

  bool CanRumble = false;
 private:
  // === Hardware & IO Interfaces ===
    GripperIO *m_pGripperIO;
    GripperIOInputs inputs;
    GripperIOLogger m_logger{frc::DataLogManager::GetLog(), "/Gripper"};
  // === System States & Control Modes ===
    WantedState m_wantedState = WantedState::STAND_BY;
    WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
    SystemState m_systemState = SystemState::IDLE;
    ControlMode m_controlMode = gripperConstants::MainControlMode;
  // === Control Inputs / Outputs ===
    double m_manualControlInput{0.0};
    double m_feederOutput{0.0};
    double m_outtakeOutput{0.0};
  // === Internal Calculations ===
    int m_counter{0};
  // === System Alerts ===
    Alert m_feederMotorDisconnected{"Feeder Motor: Disconnected", Alert::AlertType::ERROR};
    Alert m_outtakeMotorDisconnected{"Outtake Motor: Disconnected", Alert::AlertType::ERROR};
    Alert m_feederHot{"Feeder Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
    Alert m_outtakeHot{"Outtake Motor: Temperature exceeds 60°C", Alert::AlertType::WARNING};
    Alert m_feederOverheating{"Feeder Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
    Alert m_outtakeOverheating{"Outtake Motor: Temperature exceeds 75°C", Alert::AlertType::ERROR};
  // === Internal Methods ===
    void RunStateMachine();
};
