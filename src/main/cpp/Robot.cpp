// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h> 
#include <iostream>

Robot::Robot() {
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  m_camera.Update();

  if(m_camera.HasTargets()) // TODO : REVIEW led that doesn't seem to work properly
  {
    m_led.SetSpeed(-0.39);
  }
  else if(m_container.m_gripper.m_state == Gripper::State::REST_LOADED)
  {
    m_led.SetSpeed(-0.37);
  }
  else
  {
    m_led.SetSpeed(-0.41);
  }
}

void Robot::Leave(double target) {
  if((m_container.m_drivetrain.DriveAuto() - initialPosition) > target)
  {
    m_container.m_drivetrain.SetPower(0.0);
  }
  else {
      m_container.m_drivetrain.SetPower(0.3);
  }
}
void Robot::CenterToL4() {
  m_container.m_camera.Update();
  switch (m_state)
  {
  case AutoState::Leave:
    if(m_container.m_camera.HasTargets() && (m_container.m_camera.GetDistance(m_container.m_camera.GetBestTarget()) <0.62) )
    {
      m_container.m_drivetrain.SetPower(0.0);
      m_state = AutoState::Elevate;
      m_container.m_elevator.SetDesiredStage(Stage::L4);
    }
    else
    {
      m_container.m_drivetrain.SetPower(0.2);
      m_container.m_elevator.SetDesiredStage(Stage::L2); // position to detect targets
    }
    break;
  
  case AutoState::Elevate :
    if(m_container.m_elevator.IsAtL4())
    {
      m_state = AutoState::Align;
      m_container.m_straffer.m_state = Straffer::State::SEEK_APRIL_TAG;
      m_container.m_straffer.m_counter = strafferConstants::Counter::SEEK_APRIL_TAG; // counter for State::SEEK_APRIL_TAG 
      m_container.m_straffer.m_targetOffset = -0.17; //LeftOffSet
       m_container.m_straffer.m_lowestAmbiguity = 1.0;
    }
    break;
  
  case AutoState::Align : 
    if(m_container.m_straffer.m_state == Straffer::State::AT_REEF)
      {
        m_state = AutoState::Shoot;
        m_container.m_gripper.m_state = Gripper::State::PRESHOOT;
        m_container.m_gripper.SetSpeedOuttake(gripperConstants::Speed::PRESHOOT);
        m_container.m_gripper.m_ShoooootttttSpeed = 0.4;
        m_container.m_gripper.m_counter = gripperConstants::Counter::PRESHOOT;
      }
    break;
  
  case AutoState::Shoot :
    if(m_container.m_gripper.m_state == Gripper::State::REST_EMPTY)
    {
      m_container.m_elevator.SetDesiredStage(Stage::CORAL_STATION);
    }
    break;
  default:
    break;
  }
}
void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::DisabledExit() {
  m_container.m_straffer.isInitialized = false;
  m_container.m_elevator.ActivateInit();
}

void Robot::AutonomousInit() {
  // m_container.m_drivetrain.isAuto = true;
  // m_container.m_gripper.SetControlMode(ControlMode::AUTO_LOOP);
  // m_container.m_straffer.SetControlMode(ControlMode::AUTO_LOOP);
  // m_container.m_elevator.SetControlMode(ControlMode::AUTO_LOOP);
  // initialPosition = m_container.m_drivetrain.DriveAuto();
  // m_container.m_drivetrain.SetPower(0.5);
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  // Leave(3.0);
  CenterToL4();
}

void Robot::AutonomousExit() {
}

void Robot::TeleopInit() {
  m_container.m_straffer.SetControlMode(strafferConstants::defaultMode);
  m_container.m_elevator.SetControlMode(elevatorConstants::defaultMode);
  m_container.m_gripper.SetControlMode(gripperConstants::defaultMode);
  m_container.m_drivetrain.isAuto = false;
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {
  if(canRumble)
  {
    if(m_container.m_gripper.m_rumble || m_container.m_straffer.m_rumble)
    {
      canRumble = false;
      m_rumbleCounter = 11;
    }
  }
  else
  {
    m_rumbleCounter--;
    if(m_rumbleCounter == 0)
    {
      canRumble = true;
      m_container.m_straffer.m_rumble = false;
      m_container.m_gripper.m_rumble = false;
      m_container.m_controllerCopilot.SetRumble(frc::PS4Controller::RumbleType::kBothRumble, 0.0);
    }
    else 
    {
      m_container.m_controllerCopilot.SetRumble(frc::PS4Controller::RumbleType::kBothRumble, 0.5553);
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
