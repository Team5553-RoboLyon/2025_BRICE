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
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/CommandGenericHID.h>
#include "commands/Drive.h"
#include "subsystems/Drivetrain.h"
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

    Drivetrain m_drivetrain;
 private:
  frc::Joystick m_joystickForward{ControlPanelConstants::Joystick::FORWARD_ID};
  frc::Joystick m_joystickRotation{ControlPanelConstants::Joystick::ROTATION_ID};
  frc::Joystick m_xboxControllerCopilot{ControlPanelConstants::Joystick::XBOX_CONTROLLER_ID};
  frc2::JoystickButton m_SlowDriveButton{&m_joystickRotation, ControlPanelConstants::Button::SLOW_DRIVE_BUTTON};
  frc2::JoystickButton m_ReversedDriveButton{&m_joystickForward, ControlPanelConstants::Button::REVERSED_DRIVE_BUTTON};
  void ConfigureBindings();
};
