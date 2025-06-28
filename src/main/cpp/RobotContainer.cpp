// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>
RobotContainer::RobotContainer() :
                                m_gripper(new GripperIOSpark())
{
    // ConfigureBindings();
    // m_gripper.SetDefaultCommand(RunDefaultGripper(&m_gripper, &m_controllerCopilot));
    // m_straffer.SetDefaultCommand(RunDefaultStraffer(&m_straffer, &m_controllerCopilot));
    // m_elevator.SetDefaultCommand(RunDefaultElevator(&m_elevator, &m_controllerCopilot));
     m_drivetrain.SetDefaultCommand(Drive( [this]
    { return m_joystickForward.GetY(); },
                                      [this]
    { return m_joystickRotation.GetZ(); },
    &m_drivetrain, &m_elevator));
}

void RobotContainer::ConfigureBindings() {
  //TODO add operator class 
  //TODO add interaction with other command
  m_ReversedDriveButton.ToggleOnTrue(frc2::InstantCommand([this] { m_drivetrain.ReverseDrive(); }).ToPtr());
  m_SlowDriveButton.OnChange(frc2::InstantCommand([this] {m_drivetrain.slower = !m_drivetrain.slower;}).ToPtr());

  m_scoreButton.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::SCORE); }).ToPtr());
  m_intakeButton.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::COLLECT); }).ToPtr());
  m_stageL2Button.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::ALIGN_L2); }).ToPtr());
  m_stageL3Button.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::ALIGN_L3); }).ToPtr());
  m_stageL4Button.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::ALIGN_L4); }).ToPtr());
  m_stageL1Button.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::ALIGN_L1); }).ToPtr());
  m_stageCoralStationButton.WhileTrue(frc2::InstantCommand([this] { m_superstructure.SetWantedSuperState(Superstructure::WantedSuperState::MOVE_TO_STATION); }).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}