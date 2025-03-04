// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/CommandGenericHID.h>

#include "commands/MoveElevatorToLevel.h"
#include "subsystems/elevator.h"
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  frc::XboxController m_xboxControllerCopilot{ControlPanelConstants::Joystick::XBOX_CONTROLLER_ID};
  frc2::JoystickButton m_L2{&m_xboxControllerCopilot, ControlPanelConstants::Button::L2};
  frc2::JoystickButton m_L3{&m_xboxControllerCopilot, ControlPanelConstants::Button::L3};
  frc2::JoystickButton m_L4{&m_xboxControllerCopilot, ControlPanelConstants::Button::L4};
  Elevator m_elevator;
 private:
  void ConfigureBindings();
};