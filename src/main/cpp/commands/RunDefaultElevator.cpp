// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunDefaultElevator.h"


RunDefaultElevator::RunDefaultElevator(Elevator *pElevator, frc::PS4Controller *pGamepad) : m_pElevator(pElevator), m_pGamepad(pGamepad) {
  AddRequirements(m_pElevator);
}

// Called when the command is initially scheduled.
void RunDefaultElevator::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunDefaultElevator::Execute() {
  if(m_pElevator->GetControlMode() == ControlMode::OPEN_LOOP) 
  {
    if(NABS(m_pGamepad->GetLeftY()) > ControlPanelConstants::Settings::DEADBAND_OPEN_LOOP)
      m_pElevator->SetJoystickInput(m_pGamepad->GetLeftY()/elevatorConstants::Settings::JOYSTICK_REDUCTION);
    else 
      m_pElevator->SetJoystickInput(elevatorConstants::Speed::REST);
    
  }
}

// Called once the command ends or is interrupted.
void RunDefaultElevator::End(bool interrupted) {
  m_pElevator->SetJoystickInput(elevatorConstants::Speed::REST);
}

// Returns true when the command should end.
bool RunDefaultElevator::IsFinished() {
  return false;
}