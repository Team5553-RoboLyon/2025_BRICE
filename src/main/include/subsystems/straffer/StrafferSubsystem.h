#pragma once
//TODO : change implement Cam with a proper way
#include <frc2/command/SubsystemBase.h>
#include "StrafferIO.h"
#include "StrafferIOLogger.h"
#include "StrafferConstants.h"
#include "lib/Alert.h"
#include "lib/PidRBL.h"
#include "lib/RateLimiter.h"
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
    void ToggleControlMode();

    bool IsResting();
    bool IsInitialized() { return m_isInitialized; } //COMMENTME
    void SetManualAxis(const double value);
    void Periodic() override;

    bool CanRumble = false; //COMMENTME

  private:
    WantedState m_wantedState = WantedState::STAND_BY;
    WantedState m_currentWantedState = m_wantedState; //Local discrete snapshot of m_wantedState for each cycle
    SystemState m_systemState = SystemState::IDLE;
    ControlMode m_controlMode = strafferConstants::MainControlMode;
    StrafferIO *m_pStrafferIO;
    StrafferIOInputs inputs;
    StrafferIOLogger m_logger{frc::DataLogManager::GetLog(), "/Straffer"};
    Camera *m_pCamera;

    int m_counter{0};
    double m_output{0.0}; 
    double m_timestamp{0.0};
    double m_selectedReefWidthPosition{0.0}; //COMMENTME
    double m_lowestAmbiguity{1.0}; //COMMENTME
    double m_bestAprilTagOffset{0.0}; //COMMENTME
    bool m_isInitialized = false; 
    bool m_isEncoderAlreadyReset = false;

    PidRBL m_strafferPIDController;
    RateLimiter m_rateLimiter{strafferConstants::Settings::TIME_TO_REACH_FULL_SPEED};

    Alert m_motorDisconnected{"Straffer Motor: Disconnected", Alert::AlertType::ERROR};
    Alert m_motorHot{"Straffer Motor: Temperature exceeds 55°C", Alert::AlertType::WARNING};
    Alert m_motorOverheating{"Straffer Motor: Temperature exceeds 70°C", Alert::AlertType::ERROR};

    void RunStateMachine();
};
