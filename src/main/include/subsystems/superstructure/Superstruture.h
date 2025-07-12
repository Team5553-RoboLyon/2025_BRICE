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
      STAND_BY =0, // no wanted state scheduled. (It's all good man, it's all good !)
      SCORE =1,
      COLLECT =2,
      TOGGLE =3, //TEST
      MOVE_TO_STATION =4,
      MOVE_TO_HOME =5,
      ALIGN_L1 =6, // TODO : try to score while straffing
      ALIGN_L2 =7,
      ALIGN_L3 =9,
      ALIGN_L4 =10,
      ALIGN_L2_A =11,
      ALIGN_L2_B =12,
      ALIGN_L3_A =13,
      ALIGN_L3_B =14,
      ALIGN_L4_A =15,
      ALIGN_L4_B =16,
      INITIALIZATION =17
      // DEFENSE,
      // GOODBYE_ALGAE,    //TODO
      // LE_CASSE_DE_BRICE //TODO
    };
    //BUG? : prevent moving to stage after shooting while collecting 

    enum class SystemSuperState
    {
      IDLE =0,
      //Steady states
      AT_HOME_EMPTY =1,  // ELEVATOR + STRAFFER = HOME & GRIPPER EMPTY
      AT_HOME_COLLECTED =2, // ELEVATOR + STRAFFER = HOME & GRIPPER LOADED
      AT_STATION_COLLECTED =3, // ELEVATOR + STRAFFER = CS & GRIPPER LOADED
      READY_TO_COLLECT =4,  // ELEVATOR + STRAFFER = CS & GRIPPER EMPTY
      READY_TO_SCORE_AT_L1 =5, // ELEVATOR + STRAFFER = STAGE L1 & GRIPPER LOADED
      READY_TO_SCORE_AT_L2 =6, // ELEVATOR + STRAFFER = STAGE L2 & GRIPPER LOADED
      READY_TO_SCORE_AT_L3 =7, // ELEVATOR + STRAFFER = STAGE L3 & GRIPPER LOADED
      READY_TO_SCORE_AT_L4 =8, // ELEVATOR + STRAFFER = STAGE L4 & GRIPPER LOADED
      //Transition state
      PREPARING_TO_COLLECT =9, // ELEVATOR + STRAFFER = GO TO CS & GRIPPER EMPTY
      PREPARING_TO_SCORE =10, // ELEVATOR + STRAFFER = GO TO STAGE & GRIPPER LOADED
      RETURNING_TO_HOME_EMPTY =11, // ELEVATOR + STRAFFER = GO TO HOME & GRIPPER EMPTY
      RETURNING_TO_HOME_COLLECTED =12,  // ELEVATOR + STRAFFER = GO TO HOME & GRIPPER LOADED
      COLLECTING =13, // ELEVATOR + STRAFFER = CS & GRIPPER COLLECTING
      SCORING =14, // ELEVATOR + STRAFFER = STAGE & GRIPPER SCORING
      TOGGLING =15 // ELEVATOR + STRAFFER = REST & GRIPPER TOGGLING
    };


  void SetAssistMode(bool alignAssist, bool shootAssist); //TEST
  void ToggleAssistMode(); //TEST
  void ToggleAlignAssist();//TEST
  void ToggleShootAssist();//TEST
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