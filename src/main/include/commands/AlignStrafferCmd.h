// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <iostream>
#include <array>

#include "Constants.h"
#include "subsystems/Straffer.h"
#include "subsystems/Gripper.h"

class AlignStrafferCmd
    : public frc2::CommandHelper<frc2::Command, AlignStrafferCmd> {
 public:
  AlignStrafferCmd(Straffer *pStraffer, Gripper *pGripper, Side side);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Straffer* m_pStraffer;
  Gripper* m_pGripper;
  Side m_targetSide;
  bool isFinished;
  
  double m_offset;
};