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

    m_superstructure.ConfigureManualAxis([this] { return m_controllerCopilot.GetLeftY(); },
                                         [this] { return m_controllerCopilot.GetRightX(); },
                                         [this] { return (-m_controllerCopilot.GetL2Axis() + m_controllerCopilot.GetR2Axis()); });

    m_drivetrain.ConfigureManualAxis([this] { return NDEADBAND(-m_joystickForward.GetY(), driveConstants::Settings::DEADBAND); },
                                      [this] { return NDEADBAND(m_joystickRotation.GetZ(), driveConstants::Settings::DEADBAND); },
                                      [this] { return m_SlowDriveButton.Get(); },
                                      [this] { return NORMALIZE_HEIGHT(m_elevator.GetHeight()); });
}

void RobotContainer::ConfigureBindings() {
  m_ReversedDriveButton.OnTrue(frc2::InstantCommand([this] { m_drivetrain.SetWantedDrive(DrivetrainSubsystem::WantedDrive::REVERSE_DRIVE);}).ToPtr());


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

  m_controllerCopilot.toggleElevatorManualControlButton.OnTrue(frc2::InstantCommand([this] { m_superstructure.ToggleElevatorControlMode(); }).ToPtr());
  m_controllerCopilot.toggleStrafferManualControlButton.OnTrue(frc2::InstantCommand([this] { m_superstructure.ToggleStrafferControlMode(); }).ToPtr());
  m_controllerCopilot.toggleGripperManualControlButton.OnTrue(frc2::InstantCommand([this] { m_superstructure.ToggleGripperControlMode(); }).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}