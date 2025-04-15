// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "frc/shuffleboard/Shuffleboard.h"

#include <frc2/command/Commands.h>
RobotContainer::RobotContainer() {
    ConfigureBindings();
    m_gripper.SetDefaultCommand(RunDefaultGripper(&m_gripper, &m_controllerCopilot));
    m_straffer.SetDefaultCommand(RunDefaultStraffer(&m_straffer, &m_controllerCopilot));
    m_elevator.SetDefaultCommand(RunDefaultElevator(&m_elevator, &m_controllerCopilot));
     m_drivetrain.SetDefaultCommand(Drive( [this]
    { return m_joystickForward.GetY(); },
                                      [this]
    { return m_joystickRotation.GetZ(); },
    &m_drivetrain, &m_elevator));
}

void RobotContainer::ConfigureBindings() {
  m_CoralStationButton.WhileTrue(SetStageCmd(&m_elevator, &m_gripper, Stage::CORAL_STATION).ToPtr());
  m_L1Button.WhileTrue(SetStageCmd(&m_elevator, &m_gripper,Stage::L1).ToPtr());
  m_L2Button.WhileTrue(SetStageCmd(&m_elevator,&m_gripper,Stage::L2).ToPtr());
  m_L3Button.WhileTrue(SetStageCmd(&m_elevator,&m_gripper,Stage::L3).ToPtr());
  m_L4Button.WhileTrue(SetStageCmd(&m_elevator,&m_gripper,Stage::L4).ToPtr());

  m_IntakeButton.WhileTrue(IntakeCoralCmd(&m_gripper, &m_straffer, &m_elevator).ToPtr());
  m_ShootButton.WhileTrue(PreshootCmd(&m_gripper, &m_straffer, &m_elevator).ToPtr());

  m_rightSideButton.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::RIGHT).ToPtr());
  m_leftSideButton.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::LEFT).ToPtr());
  m_CoralStationButton.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::CENTER).ToPtr());
  m_L1Button.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::CENTER).ToPtr());
  m_L2Button.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::CENTER).ToPtr());
  m_L3Button.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::CENTER).ToPtr());
  m_L4Button.WhileTrue(AlignStrafferCmd(&m_straffer, &m_gripper, Side::CENTER).ToPtr());

  m_ReversedDriveButton.ToggleOnTrue(frc2::InstantCommand([this] { m_drivetrain.ReverseDrive(); }).ToPtr());
  m_SlowDriveButton.OnChange(frc2::InstantCommand([this] {m_drivetrain.slower = !m_drivetrain.slower;}).ToPtr());

  m_OpenLoopOuttakeButton.ToggleOnTrue(frc2::InstantCommand([this] {m_gripper.ToggleControlMode();}).ToPtr());
  m_OpenLoopStrafferButton.ToggleOnTrue(frc2::InstantCommand([this] { m_straffer.SetControlMode(ControlMode::OPEN_LOOP); }).ToPtr());
  m_OpenLoopElevatorButton.ToggleOnTrue(frc2::InstantCommand([this] { m_elevator.SetControlMode(ControlMode::OPEN_LOOP); }).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}