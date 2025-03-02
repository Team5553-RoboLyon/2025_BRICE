// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_L1.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::Stage::L1);}).ToPtr());
  m_L2.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::Stage::L2);}).ToPtr());
  m_L3.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::Stage::L3);}).ToPtr());
  m_L4.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectWantedStage(elevatorConstants::Stage::L4);}).ToPtr());
  m_atL1.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(0);}).ToPtr());
  m_atL1l2.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(1);}).ToPtr());
  m_atL2.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(2);}).ToPtr());
  m_atL2l3.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(3);}).ToPtr());
  m_atL3.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(4);}).ToPtr());
  m_atL3l4.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(5);}).ToPtr());
  m_atL4.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectCurrentPosition(6);}).ToPtr());
  m_movingUp.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectMoving(1);}).ToPtr());
  m_movingDown.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectMoving(2);}).ToPtr());
  m_stop.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SelectMoving(0);}).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
