#pragma once
#include <frc2/command/SubsystemBase.h>

#include "subsystems/straffer/StrafferSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"

class Superstructure : public frc2::SubsystemBase {
  public :

    Superstructure(StrafferSubsystem *pStrafferSubsystem,
                   ElevatorSubsystem *pElevatorSubsystem,
                   GripperSubsystem *pGripperSubsystem);

    Superstructure(StrafferSubsystem *pStrafferSubsystem,
                   ElevatorSubsystem *pElevatorSubsystem,
                   GripperSubsystem *pGripperSubsystem, 
                   double *pElevatorAxis,
                   double *pStrafferAxis,
                   double *pGripperAxis);
    enum class WantedSuperState 
    {
      STAND_BY, // no wanted state scheduled. (It's all good man, it's all good !)
      SCORE,
      COLLECT,
      TOGGLE,
      MOVE_TO_STATION,
      MOVE_TO_HOME,
      ALIGN_L1,
      ALIGN_L2,
      ALIGN_L3,
      ALIGN_L4,
      ALIGN_L2_A,
      ALIGN_L2_B,
      ALIGN_L3_A,
      ALIGN_L3_B,
      ALIGN_L4_A,
      ALIGN_L4_B,
      INITIALIZATION
      // DEFENSE,
      // GOODBYE_ALGAE,    //TODO
      // LE_CASSE_DE_BRICE //TODO
    };

    enum class SystemSuperState
    {
      IDLE,
      //Steady states
      AT_HOME_EMPTY,  // ELEAVATOR + STRAFFER = HOME & GRIPPER EMPTY
      AT_HOME_COLLECTED, // ELEAVATOR + STRAFFER = HOME & GRIPPER LOADED
      AT_STATION_COLLECTED, // ELEAVATOR + STRAFFER = CS & GRIPPER LOADED
      READY_TO_COLLECT,  // ELEAVATOR + STRAFFER = CS & GRIPPER EMPTY
      READY_TO_SCORE, // ELEAVATOR + STRAFFER = STAGE & GRIPPER LOADED
      //Transition state
      PREPARING_TO_COLLECT, // ELEVAOR + STRAFFER = GO TO CS & GRIPPER EMPTY
      PREPARING_TO_SCORE, // ELEVAOR + STRAFFER = GO TO STAGE & GRIPPER LOADED
      RETURNING_TO_HOME_EMPTY, // ELEVAOR + STRAFFER = GO TO HOME & GRIPPER EMPTY
      RETURNING_TO_HOME_COLLECTED,  // ELEVAOR + STRAFFER = GO TO HOME & GRIPPER LOADED
      COLLECTING, // ELEVAOR + STRAFFER = CS & GRIPPER COLLECTING
      SCORING, // ELEVAOR + STRAFFER = STAGE & GRIPPER SCORING
      TOGGLING // ELEVAOR + STRAFFER = REST & GRIPPER TOGGLING
    };


  void SetAssistMode(bool alignAssist, bool shootAssist);
  void ToggleAssistMode();
  void ToggleAlignAssist();
  void ToggleShootAssist();
  std::function<bool()> HasCoral() const;
  void SetWantedSuperState(const WantedSuperState wantedSuperState);
  SystemSuperState GetSystemSuperState() const;

  void Periodic() override;
  void RunSuperStateMachine();

  private :
    bool m_alignAssistEnabled = false;
    bool m_shootAssistEnabled = false;

    WantedSuperState m_wantedSuperState = WantedSuperState::STAND_BY;
    WantedSuperState m_currentWantedSuperState = m_wantedSuperState; //Local discrete snapshot of m_wantedSuperState for each cycle
    SystemSuperState m_systemSuperState = SystemSuperState::IDLE;

    double *m_pElevatorAxis;
    double *m_pStrafferAxis;
    double *m_pGripperAxis;

    bool m_isInitialized = false;

    StrafferSubsystem *m_pStrafferSubsystem;
    ElevatorSubsystem *m_pElevatorSubsystem;
    GripperSubsystem *m_pGripperSubsystem;
};