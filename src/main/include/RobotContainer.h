// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/Joystick.h>
#include <frc/PS5Controller.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/CommandGenericHID.h>

#include "subsystems/Straffer.h"
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  
  frc::Joystick m_joystickForward{ControlPanelConstants::Joystick::FORWARD_ID};
  frc::Joystick m_joystickRotation{ControlPanelConstants::Joystick::ROTATION_ID};
  frc::PS5Controller m_controllerCopilot{ControlPanelConstants::Joystick::COPILOT_CONTROLLER_ID};

  Straffer m_straffer;
 private:

  frc2::JoystickButton m_CoralStationButton{&m_controllerCopilot, ControlPanelConstants::Button::CORAL_STATION};
  frc2::JoystickButton m_L1Button{&m_controllerCopilot, ControlPanelConstants::Button::L1};
  frc2::JoystickButton m_L2Button{&m_controllerCopilot, ControlPanelConstants::Button::L2};
  frc2::JoystickButton m_L3Button{&m_controllerCopilot, ControlPanelConstants::Button::L3};
  frc2::JoystickButton m_L4Button{&m_controllerCopilot, ControlPanelConstants::Button::L4};
  frc2::JoystickButton m_TakeButton{&m_controllerCopilot, ControlPanelConstants::Button::TAKE};
  frc2::JoystickButton m_OuttakeButton{&m_controllerCopilot, ControlPanelConstants::Button::OUTTAKE};
  frc2::JoystickButton m_OpenLoopButton{&m_controllerCopilot, ControlPanelConstants::Button::OPEN_LOOP};
  void ConfigureBindings();
};