// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

#include "subsystems/superstructure/Superstruture.h"
#include "subsystems/straffer/StrafferSubsystem.h"
#include "subsystems/elevator/ElevatorSubsystem.h"
#include "subsystems/gripper/GripperSubsystem.h"
#include "subsystems/drivetrain/DrivetrainSubsystem.h"
#include "subsystems/drivetrain/DrivetrainConstants.h"
#include "subsystems/vision/Camera.h"

#include "subsystems/elevator/ElevatorIOSpark.h"
#include "subsystems/straffer/StrafferIOSpark.h"
#include "subsystems/gripper/GripperIOSpark.h"
#include "subsystems/drivetrain/DrivetrainIOFlex.h"

#include "subsystems/operator/Operator.h"

class RobotContainer {
 public:
  RobotContainer();

    Camera m_camera;
    DrivetrainSubsystem drivetrain{new DrivetrainIOFlex()};
    StrafferSubsystem straffer{new StrafferIOSpark(), &m_camera};
    ElevatorSubsystem elevator{new ElevatorIOSpark()};
    GripperSubsystem gripper{new GripperIOSpark()};

    Superstructure superstructure{&straffer, &elevator, &gripper};

    frc::Joystick forwardJoystick{ControlPanelConstants::Joystick::FORWARD_ID};
    frc::Joystick rotationJoystick{ControlPanelConstants::Joystick::ROTATION_ID};
    Operator CopilotController{ControlPanelConstants::Joystick::COPILOT_CONTROLLER_ID};

 private:
    frc2::JoystickButton m_SlowDriveButton{&forwardJoystick, ControlPanelConstants::Button::SLOW_DRIVE_BUTTON};
    frc2::JoystickButton m_driveActionButton{&rotationJoystick, ControlPanelConstants::Button::ACTION_DRIVE_BUTTON};

  void ConfigureBindings();
};