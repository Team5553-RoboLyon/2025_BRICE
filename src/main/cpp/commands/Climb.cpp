// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"
#include "frc/shuffleboard/Shuffleboard.h"

Climb::Climb(DeepClimb *pclimb, std::function<double()> speed) : m_climb(pclimb), m_speed(speed) {
  AddRequirements({m_climb});
}

// Called when the command is initially scheduled.
void Climb::Initialize() {
  m_climb->ResetClimberPosition();
  m_state = State::Idle;
}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
  switch (m_state) {
    case State::Idle:
      if(m_climb->isClimbed){ 
        m_state = State::End;
      } 
      else{
        m_state = State::Climbing;
      }
      break;
    case State::Climbing:
      m_climb->SetClimbSpeed(m_speed());
      //until good position is reached
      //state = end
      break;
    case State::End:
      m_climb->StopClimb();
      m_climb->isClimbed = true;
      break;
  }
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  m_climb->StopClimb();
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  if(m_climb->isClimbed){
    return true;
  }
  return false;
} 