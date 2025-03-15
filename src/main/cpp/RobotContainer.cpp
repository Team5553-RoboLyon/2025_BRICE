// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>
RobotContainer::RobotContainer() {
    ConfigureBindings();
  
  m_drivetrain.SetDefaultCommand(Drive( [=]
    { return m_joystickForward.GetY(); },
                                      [=]
    { return m_joystickRotation.GetZ(); },
    &m_drivetrain));
  m_manipulator.SetDefaultCommand(MoveManipulator(&m_manipulator, [=]{ return m_xboxControllerCopilot.GetY(); }, [=]{ return m_xboxControllerCopilot.GetZ(); }));
}

void RobotContainer::ConfigureBindings() {
  m_ReversedDriveButton.ToggleOnTrue(frc2::InstantCommand([this] { m_drivetrain.ReverseDrive(); }).ToPtr());
  m_SlowDriveButton.WhileTrue(frc2::InstantCommand([this] {m_drivetrain.slower = true;}).ToPtr());
  m_SlowDriveButton.WhileFalse(frc2::InstantCommand([this] {m_drivetrain.slower = false;}).ToPtr());

  Drop.WhileTrue(DropCoral(&m_Gripper).ToPtr());
  Catch.WhileTrue(TakeCoral(&m_Gripper).ToPtr());

  declimbButton.WhileTrue(DeClimb(&m_climb).ToPtr());
  climbButton.WhileTrue(Climb(&m_climb).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}