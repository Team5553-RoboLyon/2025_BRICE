// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_L1.WhileTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::State::L1);}).ToPtr());
  m_L2.WhileTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::State::L2);}).ToPtr());
  m_L3.WhileTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::State::L3);}).ToPtr());
  m_L4.WhileTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::State::L4);}).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
