// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/PS5Controller.h>

#include "subsystems/Elevator.h"
#include "subsystems/Straffer.h"
#include "Constants.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class MainCommand
    : public frc2::CommandHelper<frc2::Command, MainCommand> {
 public:
  MainCommand(Elevator *elevator, Straffer *straffer, frc::PS5Controller *gamepad);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  enum class State 
  {
  };
  private :
  Elevator* m_pElevator;
  Straffer* m_pStraffer;
  frc::PS5Controller* m_pGamepad;
  State m_state;
};
