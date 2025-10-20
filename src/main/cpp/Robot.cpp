// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>

#include <iostream>

Robot::Robot() {
}

void Robot::RobotInit()
{
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  m_container.drivetrain.SetWantedDrive(DriveMode::DISABLE);
  m_container.drivetrain.SetAlliance(frc::DriverStation::GetAlliance().value());

  m_pilot.Set(true);
  m_pilot.SetText("Pilot currently driving : " + (std::to_string)PILOT); //TODO : put names instead of numbers 
  m_operator.Set(true);
  m_operator.SetText("Operator currently operatoring : " + OPERATOR);
  m_robot.Set(true);
  m_robot.SetText("Robot used : " + ROBOT_MODEL);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  m_camera.Update();

  if(m_camera.HasTargets()) // TODO : REVIEW led that doesn't seem to work properly
  {
    m_led.SetSpeed(-0.39);
  }
  else if(m_container.gripper.GetSystemState() == GripperSubsystem::SystemState::REST_LOADED)
  {
    m_led.SetSpeed(-0.37);
  }
  else
  {
    m_led.SetSpeed(-0.41);
  }

  m_isNotCompetitionRobot.Set(ROBOT_MODEL != BRICE_COMPETITION);
}

void Robot::DisabledInit() {
  m_container.drivetrain.SetWantedDrive(DriveMode::DISABLE);
  m_container.elevator.SetControlMode(ControlMode::DISABLED);
  m_container.gripper.SetControlMode(ControlMode::DISABLED);
  m_container.straffer.SetControlMode(ControlMode::DISABLED);
}

void Robot::DisabledPeriodic() {
}

void Robot::DisabledExit() {
  if(m_container.superstructure.GetSuperControlMode() == Superstructure::SuperControlMode::SuperStateMachine)
  {
    m_container.superstructure.SetWantedSuperState(Superstructure::WantedSuperState::INITIALIZATION);
  }
  m_container.superstructure.ResetAllSubsystemsToMainControlMode();
  m_container.drivetrain.ResetOdometryPose(frc::Pose2d{units::meter_t{10.0}, units::meter_t{4.0}, frc::Rotation2d{units::radian_t{0.0}}});
}

void Robot::AutonomousInit() {
  m_container.drivetrain.SetWantedDrive(DriveMode::AUTO_PATH_FOLLOWER);
  
  if (Traj.has_value()) {
        // Get the initial pose of the trajectory
        std::optional<frc::Pose2d> initialPose = Traj.value().GetInitialPose(IS_RED_ALLIANCE(frc::DriverStation::GetAlliance()));
        if (initialPose.has_value()) {
            // Reset odometry to the start of the trajectory
            m_container.drivetrain.ResetOdometryPose(initialPose.value());
            m_container.drivetrain.SetDesiredAutoTrajectory(Traj.value());
        }
    }
}

void Robot::AutonomousPeriodic() {
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
  m_container.drivetrain.SetWantedDrive(driveConstants::desiredDriveControl);
}

void Robot::TeleopPeriodic() {
  if(CanRumble)
  {
    if(m_container.gripper.CanRumble || m_container.straffer.CanRumble)
    {
      CanRumble = false;
      m_rumbleCounter = 11;
    }
  }
  else
  {
    m_rumbleCounter--;
    if(m_rumbleCounter == 0)
    {
      CanRumble = true;
      m_container.straffer.CanRumble = false;
      m_container.gripper.CanRumble = false;
      m_container.CopilotController.SetRumble(Operator::RumbleType::kBothRumble, 0.0);
    }
    else 
    {
      m_container.CopilotController.SetRumble(Operator::RumbleType::kBothRumble, 0.5553);
    }
  }
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
