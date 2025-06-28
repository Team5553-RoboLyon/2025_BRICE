// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once


//TODO : clear all includes that are not used
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
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Commands.h>
#include <chrono>
#include <units/time.h>
#include "frc2/command/button/Trigger.h"

#include "Constants.h"
#include "subsystems/drivetrain/Drivetrain.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/vision/Camera.h"
#include "subsystems/straffer/StrafferSubsystem.h"
#include "subsystems/superstructure/Superstruture.h"

#include "subsystems/elevator/ElevatorIOSpark.h"
#include "subsystems/straffer/StrafferIOSpark.h"
#include "subsystems/gripper/GripperIOSpark.h"

#include "commands/Drive.h"

#include "lib/RevGamepad.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

    Drivetrain m_drivetrain;
    Camera m_camera;
    StrafferSubsystem m_straffer{new StrafferIOSpark(), &m_camera};
    ElevatorSubsystem m_elevator{new ElevatorIOSpark()};
    GripperSubsystem m_gripper{new GripperIOSpark()};

    Superstructure m_superstructure{&m_straffer, &m_elevator, &m_gripper};

    frc::Joystick m_joystickForward{ControlPanelConstants::Joystick::FORWARD_ID};
    frc::Joystick m_joystickRotation{ControlPanelConstants::Joystick::ROTATION_ID};
    RevGamepad m_controllerCopilot{ControlPanelConstants::Joystick::COPILOT_CONTROLLER_ID};
 private:
    frc2::JoystickButton m_SlowDriveButton{&m_joystickRotation, ControlPanelConstants::Button::SLOW_DRIVE_BUTTON};
    frc2::JoystickButton m_ReversedDriveButton{&m_joystickForward, ControlPanelConstants::Button::REVERSED_DRIVE_BUTTON};

    frc2::Trigger m_stageCoralStationButton{[this] { // X and not Advance mode
          return m_controllerCopilot.GetCrossButton() && 
                !m_controllerCopilot.GetShareButton();
    }};
    frc2::Trigger m_stageL1Button{[this] { // Option and not Advance mode
        return m_controllerCopilot.GetOptionsButton() && 
                !m_controllerCopilot.GetShareButton();
    }};
    frc2::Trigger m_stageL2Button{[this] { // Cicle and not Advance mode
        return m_controllerCopilot.GetCircleButton() && 
                !m_controllerCopilot.GetShareButton();
    }};
    frc2::Trigger m_stageL3Button{[this] { // square and not Advance mode
        return m_controllerCopilot.GetSquareButton() && 
                !m_controllerCopilot.GetCrossButton();
    }};
    frc2::Trigger m_stageL4Button{[this] { // triangle and not Advance mode
        return m_controllerCopilot.GetTriangleButton() && 
                !m_controllerCopilot.GetCrossButton();
    }};
    frc2::Trigger m_LeftReefButton{[this] { // Left only
        return m_controllerCopilot.GetL1Button() && 
                !m_controllerCopilot.GetR1Button();
    }};
    frc2::Trigger m_RightReefButton{[this] { // Right only
        return m_controllerCopilot.GetR1Button() && 
                !m_controllerCopilot.GetL1Button();
    }};
    frc2::Trigger m_activateAlignAssist{[this] { // Align Assist
        return m_controllerCopilot.GetL1Button() && 
                m_controllerCopilot.GetR1Button();
    }};
    frc2::Trigger m_scoreButton{[this] { // Score
        return m_controllerCopilot.GetR2AsButton();
    }}; 
    frc2::Trigger m_intakeButton{[this] { // Intake
        return m_controllerCopilot.GetL2AsButton();
    }};
  void ConfigureBindings();
};