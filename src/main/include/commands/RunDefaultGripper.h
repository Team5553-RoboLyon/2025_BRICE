// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/PS4Controller.h>
#include "Constants.h"
#include "subsystems/Gripper.h"

class RunDefaultGripper
    : public frc2::CommandHelper<frc2::Command, RunDefaultGripper> {
 public:
  RunDefaultGripper(Gripper *pGripper, frc::PS4Controller *pGamepad);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Gripper *m_pGripper;
  frc::PS4Controller *m_pGamepad;
};