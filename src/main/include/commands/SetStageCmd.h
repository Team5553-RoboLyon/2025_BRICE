// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <iostream>

#include "subsystems/Elevator.h"
#include "subsystems/Gripper.h"
#include "Constants.h"

class SetStageCmd
    : public frc2::CommandHelper<frc2::Command, SetStageCmd> {
 public:
  SetStageCmd(Elevator *pElevator, Gripper *pGripper, Stage stage);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private :
  Elevator* m_pElevator;
  Gripper* m_pGripper;
  Stage m_stage;
  bool isFinished;
};