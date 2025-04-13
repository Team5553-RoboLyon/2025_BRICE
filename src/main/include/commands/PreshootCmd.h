// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/Gripper.h"
#include "subsystems/Straffer.h"
#include "subsystems/Elevator.h"
#include <iostream>

class PreshootCmd
    : public frc2::CommandHelper<frc2::Command, PreshootCmd> {
 public:
  PreshootCmd(Gripper *pGripper, Straffer *pStraffer, Elevator *pElevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private :
  Gripper *m_pGripper;
  Straffer *m_pStraffer;
  Elevator *m_pElevator;
  bool isFinished = false;
};