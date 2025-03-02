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

#include "subsystems/elevator.h"
#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  frc::Joystick joystick{0};
  frc2::JoystickButton m_L1{&joystick, 1};
  frc2::JoystickButton m_L2{&joystick, 2};
  frc2::JoystickButton m_L3{&joystick, 3};
  frc2::JoystickButton m_L4{&joystick, 4};
  frc2::JoystickButton m_atL1{&joystick, 5};
  frc2::JoystickButton m_atL1l2{&joystick, 6};
  frc2::JoystickButton m_atL2{&joystick, 7};
  frc2::JoystickButton m_atL2l3{&joystick, 8};
  frc2::JoystickButton m_atL3{&joystick, 9};
  frc2::JoystickButton m_atL3l4{&joystick, 10};
  frc2::JoystickButton m_atL4{&joystick, 11};
  frc2::JoystickButton m_movingUp{&joystick, 12};
  frc2::JoystickButton m_movingDown{&joystick, 13};
  frc2::JoystickButton m_stop{&joystick, 14};
  Elevator m_elevator;
 private:
  void ConfigureBindings();
};