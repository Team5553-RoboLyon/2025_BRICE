// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>
RobotContainer::RobotContainer() {
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    m_TakeButton.WhileTrue(CatchCoralCommand(&m_outtake, outtakeConstants::Speed::CATCHING).ToPtr()); // TODO : add interrupt
    m_TakeButton.WhileTrue(CatchCoralCommand(&m_outtake, outtakeConstants::Speed::UP).ToPtr());
    m_OpenLoopButton.ToggleOnTrue(frc2::InstantCommand([this] { m_outtake.SetControlMode(ControlMode::OPEN_LOOP); }).ToPtr()); //TODO : recheck triggers
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}