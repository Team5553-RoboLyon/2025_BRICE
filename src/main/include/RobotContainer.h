// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc/Joystick.h>
#include <frc/PS4Controller.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/CommandPS4Controller.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/button/CommandGenericHID.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Gripper.h"
#include "subsystems/Straffer.h"
#include "Constants.h"

#include "commands/PreshootCmd.h"
#include "commands/IntakeCoralCmd.h"
#include "commands/RunDefaultGripper.h"
#include "commands/AlignStrafferCmd.h"
#include "commands/RunDefaultStraffer.h"
#include "commands/RunDefaultElevator.h"
#include "commands/SetStageCmd.h"
#include "commands/Drive.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

    Drivetrain m_drivetrain;
    Elevator m_elevator;
    Gripper m_gripper;
    Straffer m_straffer;

    frc::Joystick m_joystickForward{ControlPanelConstants::Joystick::FORWARD_ID};
    frc::Joystick m_joystickRotation{ControlPanelConstants::Joystick::ROTATION_ID};
    frc::PS4Controller m_controllerCopilot{ControlPanelConstants::Joystick::COPILOT_CONTROLLER_ID};
 private:
    frc2::JoystickButton m_SlowDriveButton{&m_joystickRotation, ControlPanelConstants::Button::SLOW_DRIVE_BUTTON};
    frc2::JoystickButton m_ReversedDriveButton{&m_joystickForward, ControlPanelConstants::Button::REVERSED_DRIVE_BUTTON};

    frc2::JoystickButton m_CoralStationButton{&m_controllerCopilot, ControlPanelConstants::Button::CORAL_STATION};
    frc2::JoystickButton m_L1Button{&m_controllerCopilot, ControlPanelConstants::Button::L1};
    frc2::JoystickButton m_L2Button{&m_controllerCopilot, ControlPanelConstants::Button::L2};
    frc2::JoystickButton m_L3Button{&m_controllerCopilot, ControlPanelConstants::Button::L3};
    frc2::JoystickButton m_L4Button{&m_controllerCopilot, ControlPanelConstants::Button::L4};

    frc2::JoystickButton m_leftSideButton{&m_controllerCopilot, ControlPanelConstants::Button::LEFT_SIDE};
    frc2::JoystickButton m_rightSideButton{&m_controllerCopilot, ControlPanelConstants::Button::RIGHT_SIDE};

    frc2::JoystickButton m_OpenLoopOuttakeButton{&m_controllerCopilot, ControlPanelConstants::Button::OPEN_LOOP_OUTTAKE};
    frc2::JoystickButton m_OpenLoopElevatorButton{&m_controllerCopilot, ControlPanelConstants::Button::OPEN_LOOP_ELEVATOR};
    frc2::JoystickButton m_OpenLoopStrafferButton{&m_controllerCopilot, ControlPanelConstants::Button::OPEN_LOOP_STRAFFER};

      frc2::Trigger m_IntakeButton{[this] { //L2 trigger
          return m_controllerCopilot.GetRawAxis(2) > 0.5;
    }};

    frc2::Trigger m_ShootButton{[this] { //R2 trigger
        return m_controllerCopilot.GetRawAxis(3) > 0.5;
    }};
  void ConfigureBindings();
};
