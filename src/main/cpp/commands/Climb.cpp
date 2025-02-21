// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Climb.h"
#include "frc/shuffleboard/Shuffleboard.h"

Climb::Climb(DeepClimb *pclimb) : m_climb(pclimb){
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
      if(m_climb->IsClimbed()){ 
        m_state = State::End;
      } 
      else{
        m_state = State::Climbing;
      }
      break;
    case State::Climbing:
      if(m_climb->IsClimbed()){ 
          m_state = State::End;
        } 
        m_climb->GoToPosition();
      break;
    case State::End:
      break;
  }
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
  m_climb->StopClimb();
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return m_climb->IsClimbed();
} 