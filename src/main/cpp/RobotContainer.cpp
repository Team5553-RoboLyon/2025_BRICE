// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/SetWantedSuperStateCmd.h"
#include "commands/Drive.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();
     m_drivetrain.SetDefaultCommand(Drive( [this]
    { return m_joystickForward.GetY(); },
                                      [this]
    { return m_joystickRotation.GetZ(); },
    &m_drivetrain, &m_elevator));
}

void RobotContainer::ConfigureBindings() {
  m_ReversedDriveButton.ToggleOnTrue(frc2::InstantCommand([this] { m_drivetrain.ReverseDrive(); }).ToPtr());
  m_SlowDriveButton.OnChange(frc2::InstantCommand([this] {m_drivetrain.slower = !m_drivetrain.slower;}).ToPtr());


  //SUPERSTRUCTURE CONTROLLER BINDINGS
  m_controllerCopilot.scoreButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::SCORE)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));
  m_controllerCopilot.intakeButton.WhileTrue(
    frc2::ConditionalCommand(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::TOGGLE),
                            SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::COLLECT),
                            m_superstructure.HasCoral()).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));

  m_controllerCopilot.stageCoralStationButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::MOVE_TO_STATION)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageHomeButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::MOVE_TO_HOME)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  
  m_controllerCopilot.stageL1Button.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L1)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL2Button.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L2)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL3Button.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L3)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL4Button.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L4)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL2AButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L2_A)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL2BButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L2_B)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL3AButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L3_A)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL3BButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L3_B)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL4AButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L4_A)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  m_controllerCopilot.stageL4BButton.OnTrue(SetWantedSuperStateCmd(&m_superstructure, Superstructure::WantedSuperState::ALIGN_L4_B)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)); 

  m_controllerCopilot.toggleAlignAssistButton.OnTrue(frc2::InstantCommand([this] { m_superstructure.ToggleAlignAssist(); }).ToPtr());
  m_controllerCopilot.toggleScoreAssistButton.OnTrue(frc2::InstantCommand([this] { m_superstructure.ToggleShootAssist(); }).ToPtr());
  m_controllerCopilot.toggleAssistModeButton.OnTrue(frc2::InstantCommand([this] { m_superstructure.ToggleAssistMode(); }).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}