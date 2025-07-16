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
#include "subsystems/vision/Camera.h"

#include "subsystems/elevator/ElevatorIOSpark.h"
#include "subsystems/straffer/StrafferIOSpark.h"
#include "subsystems/gripper/GripperIOSpark.h"
#include "subsystems/drivetrain/DrivetrainIOFlex.h"

#include "subsystems/operator/Operator.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

    Camera m_camera;
    DrivetrainSubsystem m_drivetrain{new DrivetrainIOFlex()};
    StrafferSubsystem m_straffer{new StrafferIOSpark(), &m_camera};
    ElevatorSubsystem m_elevator{new ElevatorIOSpark()};
    GripperSubsystem m_gripper{new GripperIOSpark()};

    Superstructure m_superstructure{&m_straffer, &m_elevator, &m_gripper};

    frc::Joystick m_joystickForward{ControlPanelConstants::Joystick::FORWARD_ID};
    frc::Joystick m_joystickRotation{ControlPanelConstants::Joystick::ROTATION_ID};
    Operator m_controllerCopilot{ControlPanelConstants::Joystick::COPILOT_CONTROLLER_ID};

 private:
    frc2::JoystickButton m_SlowDriveButton{&m_joystickRotation, Button::SLOW_DRIVE_BUTTON};
    frc2::JoystickButton m_ReversedDriveButton{&m_joystickForward, Button::REVERSED_DRIVE_BUTTON};

  void ConfigureBindings();
};