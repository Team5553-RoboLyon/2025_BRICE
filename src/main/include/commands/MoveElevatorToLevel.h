// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/elevator.h"
#include "Constants.h"

class MoveElevatorToLevel
    : public frc2::CommandHelper<frc2::Command, MoveElevatorToLevel> {
 public:
  MoveElevatorToLevel(Elevator *elevator, u_int16_t level);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Elevator *m_elevator;
    u_int16_t m_level;
};
