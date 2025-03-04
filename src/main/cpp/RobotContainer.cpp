// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  m_climb.SetDefaultCommand(frc2::InstantCommand([this] { m_climb.SetClimbSpeed((m_joystick.GetY() / 5.0)); }, {&m_climb}));
}

void RobotContainer::ConfigureBindings() {
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
