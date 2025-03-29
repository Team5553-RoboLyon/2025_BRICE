// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>
RobotContainer::RobotContainer() {
    ConfigureBindings();
    m_elevator.SetDefaultCommand(RunElevator(&m_elevator, [this] { return m_controllerCopilot.GetLeftY(); }).ToPtr());
}

void RobotContainer::ConfigureBindings() {
    m_CoralStationButton.WhileTrue(SetPositionCommand(&m_elevator, Stage::CORAL_STATION).ToPtr());
    m_L1Button.WhileTrue(SetPositionCommand(&m_elevator, Stage::L1).ToPtr());
    m_L2Button.WhileTrue(SetPositionCommand(&m_elevator, Stage::L2).ToPtr());
    m_L3Button.WhileTrue(SetPositionCommand(&m_elevator, Stage::L3).ToPtr());
    m_L4Button.WhileTrue(SetPositionCommand(&m_elevator, Stage::L4).ToPtr());
    m_OpenLoopButton.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SetControlMode(DriveMode::OPEN_LOOP); }).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}