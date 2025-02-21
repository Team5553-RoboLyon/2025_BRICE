// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DeepClimb.h"
#include <iostream>

DeepClimb::DeepClimb() {
    //configure the motors
    m_climbBackMotorConfig
        .SetIdleMode(DeepClimbConstants::Motors::Back::IDLE_MODE)
        .Inverted(DeepClimbConstants::Motors::Back::INVERTED)
        .SmartCurrentLimit(DeepClimbConstants::Motors::Back::CURRENT_LIMIT)
        .ClosedLoopRampRate(DeepClimbConstants::Motors::Back::RAMP)
        .VoltageCompensation(DeepClimbConstants::Motors::Back::VOLTAGE_COMPENSATION)
        .Follow(DeepClimbConstants::Motors::Front::ID, DeepClimbConstants::Motors::Back::INVERTED);
    
    m_climbBackMotor.Configure( 
        m_climbBackMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    
    m_climbFrontMotorConfig
        .SetIdleMode(DeepClimbConstants::Motors::Front::IDLE_MODE)
        .Inverted(DeepClimbConstants::Motors::Front::INVERTED)
        .SmartCurrentLimit(DeepClimbConstants::Motors::Front::CURRENT_LIMIT)
        .ClosedLoopRampRate(DeepClimbConstants::Motors::Front::RAMP)
        .VoltageCompensation(DeepClimbConstants::Motors::Front::VOLTAGE_COMPENSATION);
    
    m_climbFrontMotor.Configure( 
        m_climbFrontMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters); 

    //configure the encoder
    m_climbEncoder.SetDistancePerPulse(DeepClimbConstants::Encoder::Settings::DISTANCE_PER_PULSE);
    m_climbEncoder.Reset();
};

void DeepClimb::Periodic() {
    //frc::SmartDashboard::PutNumber("climb encoder", m_climbEncoder.GetDistance());
};

void DeepClimb::SetClimbSpeed(double speed) {
    m_climbFrontMotor.Set(speed);
};
void DeepClimb::StopClimb() {
    m_climbFrontMotor.Set(0);
};
double DeepClimb::GetArmPosition() {
    //to do
    return 0.0;
}
void DeepClimb::ResetClimberPosition() {
    //stop quand est en à 90% du max
    //sinon tourner
    m_climbEncoder.Reset();
};