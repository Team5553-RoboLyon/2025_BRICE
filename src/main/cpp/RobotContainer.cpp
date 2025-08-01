// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/Commands.h>

#include "commands/SetWantedSuperStateCmd.h"

RobotContainer::RobotContainer()
{
    ConfigureBindings();

    superstructure.ConfigureManualAxis([this] { return CopilotController.GetLeftY(); },
                                         [this] { return CopilotController.GetRightX(); },
                                         [this] { return (-CopilotController.GetL2Axis() + CopilotController.GetR2Axis()); });

    drivetrain.ConfigureManualAxis([this] { return NDEADBAND(-forwardJoystick.GetY(), driveConstants::Settings::DEADBAND); },
                                      [this] { return NDEADBAND(rotationJoystick.GetZ(), driveConstants::Settings::DEADBAND); },
                                      [this] { return m_SlowDriveButton.Get(); },
                                      [this] { return m_driveActionButton.Get();},
                                      [this] { return NORMALIZE_HEIGHT(elevator.GetHeight()); });
}

void RobotContainer::ConfigureBindings() {

  //SUPERSTRUCTURE CONTROLLER BINDINGS
  CopilotController.scoreButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::SCORE)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));
  CopilotController.intakeButton.WhileTrue(
    frc2::ConditionalCommand(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::TOGGLE),
                            SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::COLLECT),
                            superstructure.HasCoral()).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelIncoming));

  CopilotController.stageCoralStationButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::MOVE_TO_STATION)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageHomeButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::MOVE_TO_HOME)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  
  CopilotController.stageL1Button.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L1)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL2Button.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L2)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL3Button.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L3)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL4Button.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L4)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL2AButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L2_A)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL2BButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L2_B)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL3AButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L3_A)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL3BButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L3_B)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL4AButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L4_A)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
  CopilotController.stageL4BButton.OnTrue(SetWantedSuperStateCmd(&superstructure, Superstructure::WantedSuperState::ALIGN_L4_B)
                                  .WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf)); 

  CopilotController.toggleAlignAssistButton.OnTrue(frc2::InstantCommand([this] { superstructure.ToggleAlignAssist(); }).ToPtr());
  CopilotController.toggleScoreAssistButton.OnTrue(frc2::InstantCommand([this] { superstructure.ToggleShootAssist(); }).ToPtr());
  CopilotController.toggleAssistModeButton.OnTrue(frc2::InstantCommand([this] { superstructure.ToggleAssistMode(); }).ToPtr());

  CopilotController.toggleElevatorManualControlButton.OnTrue(frc2::InstantCommand([this] { superstructure.ToggleElevatorControlMode(); }).ToPtr());
  CopilotController.toggleStrafferManualControlButton.OnTrue(frc2::InstantCommand([this] { superstructure.ToggleStrafferControlMode(); }).ToPtr());
  CopilotController.toggleGripperManualControlButton.OnTrue(frc2::InstantCommand([this] { superstructure.ToggleGripperControlMode(); }).ToPtr());
}