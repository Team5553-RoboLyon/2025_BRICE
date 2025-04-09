// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>
RobotContainer::RobotContainer() {
    ConfigureBindings();
    m_elevator.SetDefaultCommand(MainCommand(&m_elevator,&m_straffer, &m_controllerCopilot).ToPtr());
}


void RobotContainer::ConfigureBindings() {
    m_CoralStationButton.WhileTrue(SetStageCommand(&m_elevator, Stage::CORAL_STATION).ToPtr());
    m_L1Button.WhileTrue(SetStageCommand(&m_elevator, Stage::L1).ToPtr());
    m_L2Button.WhileTrue(SetStageCommand(&m_elevator, Stage::L2).ToPtr());
    m_L3Button.WhileTrue(SetStageCommand(&m_elevator, Stage::L3).ToPtr());
    m_L4Button.WhileTrue(SetStageCommand(&m_elevator, Stage::L4).ToPtr());

    m_rightSideButton.ToggleOnTrue(AlignStrafferCommand(&m_straffer, Side::RIGHT).ToPtr());
    m_leftSideButton.ToggleOnTrue(AlignStrafferCommand(&m_straffer, Side::LEFT).ToPtr());
    m_CoralStationButton.ToggleOnTrue(AlignStrafferCommand(&m_straffer, Side::CENTER).ToPtr());

    m_OpenLoopElevatorButton.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SetControlMode(ControlMode::OPEN_LOOP); }).ToPtr());
    m_OpenLoopStrafferButton.ToggleOnTrue(frc2::InstantCommand([this] { m_straffer.SetControlMode(ControlMode::OPEN_LOOP); }).ToPtr());
    // m_OpenLoopOuttakeButton.ToggleOnTrue(frc2::InstantCommand([this] { m_outtake.SetControlMode(ControlMode::OPEN_LOOP); }).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}