#pragma once
//TODO : change implement Cam with a proper way
#include <frc2/command/SubsystemBase.h>
#include "StrafferIO.h"
#include "StrafferConstants.h"
#include "lib/Alert.h"
#include "lib/pid_rbl.h"
#include "lib/rate_limiter.h"
#include "subsystems/vision/Camera.h"

class StrafferSubsystem : public frc2::SubsystemBase {
  public:
    StrafferSubsystem(StrafferIO *pIo, Camera *pCamera); //cam in param temporary

    enum class WantedState 
    {
      STAND_BY, // no wanted state scheduled. (It's all good man, it's all good !)
      ALIGN_LEFT_REEF,
      ALIGN_RIGHT_REEF,
      AUTO_ALIGN,
      GO_TO_STATION,
      GO_TO_LEFT_SIDE,
      GO_TO_RIGHT_SIDE,
      INITIALIZATION
    };
    enum class SystemState
    {
      IDLE,
      //Steady states
      AT_STATION,
      AT_LEFT_REEF,
      AT_RIGHT_REEF,
      AT_LEFT_SIDE,
      AT_RIGHT_SIDE,
      //Transition state
      SEEKING_APRIL_TAG,
      STRAFFING_TO_LEFT_REEF,
      STRAFFING_TO_RIGHT_REEF,
      STRAFFING_TO_STATION,
      STRAFFING_TO_LEFT_SIDE,
      STRAFFING_TO_RIGHT_SIDE
    };
    void SetWantedState(const WantedState wantedState);
    SystemState GetSystemState();
    void SetControlMode(const ControlMode mode);
    ControlMode GetControlMode();

    bool IsResting();
    bool IsInitialized() { return m_isInitialized; } //COMMENTME
    void SetOutputInOpenLoop(double dutyCycle);
    void Periodic() override;

    bool CanRumble = false; //COMMENTME

  private:
    WantedState m_wantedState = WantedState::STAND_BY;
    WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
    SystemState m_systemState = SystemState::IDLE;
    ControlMode m_controlMode = strafferConstants::DefaultMode;
    StrafferIO *m_pStrafferIO;
    StrafferIOInputs inputs;
    Camera *m_pCamera;

    int m_counter{0};
    double m_output{0.0}; 
    double m_selectedReefWidthPosition{0.0}; //COMMENTME
    double m_lowestAmbiguity{1.0}; //COMMENTME
    double m_bestAprilTagOffset{0.0}; //COMMENTME
    bool m_isInitialized = false; 
    bool m_isEncoderAlreadyReset = false;

    PidRBL m_strafferPIDController{strafferConstants::PID::KP, strafferConstants::PID::KI, strafferConstants::PID::KD};
    RateLimiter m_rateLimiter; 

    Alert m_motorDisconnected{"Straffer Motor: Disconnected", Alert::AlertType::ERROR};
    Alert m_motorHot{"Straffer Motor: Temperature exceeds 55°C", Alert::AlertType::WARNING};
    Alert m_motorOverheating{"Straffer Motor: Temperature exceeds 70°C", Alert::AlertType::ERROR};

    /**
    * @brief Executes the state machine logic for the StrafferSubsystem.
    * 
    * This function handles transitions between different wanted states and system states
    * based on the current inputs and conditions. It ensures the subsystem behaves as expected
    * by updating the system state and performing necessary actions for each state.
    * 
    * The state machine operates in two main parts:
    * 1. Transitioning from the wanted state to the corresponding system state.
    * 2. Handling the logic for each system state and transitioning to the next state if conditions are met.
    * 
    * State Transition Logic:
    * - WantedState::ALIGN_TO_LEFT_REEF or ALIGN_TO_RIGHT_REEF transitions to SystemState::SEEKING_APRIL_TAG.
    * - WantedState::GO_TO_LEFT_SIDE transitions to SystemState::STRAFFING_TO_LEFT_SIDE.
    * - WantedState::GO_TO_RIGHT_SIDE transitions to SystemState::STRAFFING_TO_RIGHT_SIDE.
    * - WantedState::GO_TO_STATION transitions to SystemState::STRAFFING_TO_STATION.
    * - WantedState::INITIALIZATION and STAND_BY do not change the system state.
    * 
    * System State Logic:
    * - SystemState::IDLE determines the position of the subsystem and transitions to the appropriate state.
    * - SystemState::STRAFFING_TO_STATION, STRAFFING_TO_LEFT_SIDE, and STRAFFING_TO_RIGHT_SIDE
    *   transition to their respective "AT" states when the position is within a specified tolerance.
    * - SystemState::STRAFFING_TO_LEFT_REEF and STRAFFING_TO_RIGHT_REEF transition to their respective
    *   "AT" states when the position matches the selected reef position within a tolerance.
    * - SystemState::SEEKING_APRIL_TAG handles the logic for finding the best AprilTag target and
    *   transitions to the appropriate reef state or station state based on the results.
    * - Other "AT" states (e.g., AT_STATION, AT_LEFT_REEF) do not perform any actions.
    */
    void RunStateMachine();
};
