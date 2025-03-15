// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
m_drivetrain.SetDefaultCommand(Drive( [=]
    { return m_joystickForward.GetY(); },
                                      [=]
    { return m_joystickRotation.GetZ(); },
    &m_drivetrain));
}

void RobotContainer::ConfigureBindings() {
  m_ReversedDriveButton.ToggleOnTrue(frc2::InstantCommand([this] { m_drivetrain.ReverseDrive(); }).ToPtr());
  m_SlowDriveButton.WhileTrue(frc2::InstantCommand([this] {m_drivetrain.slower = true;}).ToPtr());
  m_SlowDriveButton.WhileFalse(frc2::InstantCommand([this] {m_drivetrain.slower = false;}).ToPtr());

  // m_L2.WhileTrue(MoveManipulator(&m_manipulator, ManipulatorConstants::State::L2,   [=]{ return m_Gripper.Get();}).ToPtr()); DO NOT USE THIS
  m_L3.WhileTrue(MoveManipulator(&m_manipulator, ManipulatorConstants::State::L3,   [=]{ return m_Gripper.Get();}).ToPtr());
  m_L4.WhileTrue(MoveManipulator(&m_manipulator, ManipulatorConstants::State::L4,   [=]{ return m_Gripper.Get();}).ToPtr());
  m_CoralStation.WhileTrue(MoveManipulator(&m_manipulator, ManipulatorConstants::State::CoralStation, [=]{ return m_Gripper.Get();}).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}