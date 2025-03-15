// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc2/command/CommandPtr.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/CommandGenericHID.h>

#include <iostream>

#include "subsystems/Manipulator.h"
#include "Constants.h"

class MoveManipulator
    : public frc2::CommandHelper<frc2::Command, MoveManipulator> {
 public:
  MoveManipulator(Manipulator *manipulator, ManipulatorConstants::State target, std::function<bool()> drop);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Manipulator *m_manipulator;
    ManipulatorConstants::State m_target;
    std::function<bool()> m_Drop;
};
