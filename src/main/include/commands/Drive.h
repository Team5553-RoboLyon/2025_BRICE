// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/drivetrain/Drivetrain.h"
#include "subsystems/elevator/ElevatorSubsystem.h"

class Drive
    : public frc2::CommandHelper<frc2::Command, Drive> {
 public:
  Drive(std::function<double()> forward, std::function<double()> turn, Drivetrain *pDrivetrain, ElevatorSubsystem *pElevator);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  std::function<double()> m_Forward;
  std::function<double()> m_Turn;
  Drivetrain *m_pDrivetrain;
  ElevatorSubsystem *m_pElevator;
};