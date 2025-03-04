// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DeepClimb.h"
#include <iostream>

DeepClimb::DeepClimb() {
    //configure the motors
    m_climbFrontMotorConfig
        .SetIdleMode(DeepClimbConstants::Motors::IDLE_MODE)
        .Inverted(DeepClimbConstants::Motors::INVERTED)
        .SmartCurrentLimit(DeepClimbConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(DeepClimbConstants::Motors::RAMP)
        .VoltageCompensation(DeepClimbConstants::Motors::VOLTAGE_COMPENSATION);
        
    
    m_climbBackMotor.Configure( 
        m_climbBackMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    
    m_climbBackMotorConfig
        .Apply(m_climbFrontMotorConfig)
        .Follow(m_climbFrontMotor);
    
    m_climbFrontMotor.Configure( 
        m_climbFrontMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters);
}

void DeepClimb::Periodic() {
    //frc::SmartDashboard::PutNumber("climb encoder", m_climbEncoder.GetDistance());
};

void DeepClimb::SetClimbSpeed(double speed) {
    m_climbFrontMotor.Set(speed);
}