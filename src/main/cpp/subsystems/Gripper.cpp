// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Gripper.h"

Gripper::Gripper() {
        m_gripperMotorTopConfig
                .VoltageCompensation(GrippperConstants::Motors::VOLTAGE_COMPENSATION)
                .Inverted(GrippperConstants::Motors::INVERTED)
                .SmartCurrentLimit(GrippperConstants::Motors::CURRENT_LIMIT)
                .ClosedLoopRampRate(GrippperConstants::Motors::RAMP_RATE)
                .SetIdleMode(GrippperConstants::Motors::IDLE_MODE);

    m_gripperMotorTop.Configure(
                m_gripperMotorTopConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    
    m_gripperMotorBottomConfig.Apply(m_gripperMotorTopConfig)
                              .Follow(m_gripperMotorTop, true);
    
    m_gripperMotorBottom.Configure(
                m_gripperMotorBottomConfig,
                rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                rev::spark::SparkBase::PersistMode::kNoPersistParameters);
}
void Gripper::SetIntakeSpeed(double speed) {
  gripperOutput = speed;
}
// void Gripper::SetIntakeSpeed(double speed) {
//   if(m_isIRBreakerTriggered) {
//     gripperOutput = 0;
//   } else {
//     gripperOutput = speed;
//   }
// }
void Gripper::SetDropSpeed(double speed) {
  gripperOutput = speed;
}

// This method will be called once per scheduler run
void Gripper::Periodic() {
  m_isIRBreakerTriggered = m_irBreaker.Get() == GrippperConstants::Sensor::IS_TRIGGERED;

  frc::SmartDashboard::PutNumber("gripper output", m_gripperMotorBottom.Get());
  frc::SmartDashboard::PutBoolean("IR Breaker",m_isIRBreakerTriggered);
  
  m_gripperMotorTop.Set(gripperOutput);
}
