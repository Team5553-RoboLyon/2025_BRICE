// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DeepClimb.h"
#include <iostream>

DeepClimb::DeepClimb() {
    //configure the motors
    m_climbBackMotorConfig
        .SetIdleMode(DeepClimbConstants::BACK_MOTOR_IDLE_MODE)
        .Inverted(DeepClimbConstants::BACK_MOTOR_INVERTED)
        .SmartCurrentLimit(DeepClimbConstants::BACK_MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(DeepClimbConstants::BACK_MOTOR_RAMP)
        .VoltageCompensation(DeepClimbConstants::BACK_MOTOR_VOLTAGE_COMPENSATION)
        .Follow(DeepClimbConstants::FRONT_MOTOR_ID, DeepClimbConstants::BACK_MOTOR_FOLLOW)
        .softLimit.ForwardSoftLimitEnabled(true)
            .ForwardSoftLimit(DeepClimbConstants::ENCODER_FWD_SOFT_LIMIT)
            .ReverseSoftLimitEnabled(true)
            .ReverseSoftLimit(DeepClimbConstants::ENCODER_RVS_SOFT_LIMIT);
    
    m_climbBackMotor.Configure( 
        m_climbBackMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    
    m_climbFrontMotorConfig
        .SetIdleMode(DeepClimbConstants::FRONT_MOTOR_IDLE_MODE)
        .Inverted(DeepClimbConstants::FRONT_MOTOR_INVERTED)
        .SmartCurrentLimit(DeepClimbConstants::FRONT_MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(DeepClimbConstants::FRONT_MOTOR_RAMP)
        .VoltageCompensation(DeepClimbConstants::FRONT_MOTOR_VOLTAGE_COMPENSATION)
        .softLimit.ForwardSoftLimitEnabled(true)
            .ForwardSoftLimit(DeepClimbConstants::ENCODER_FWD_SOFT_LIMIT)
            .ReverseSoftLimitEnabled(true)
            .ReverseSoftLimit(DeepClimbConstants::ENCODER_RVS_SOFT_LIMIT);
    
    m_climbFrontMotor.Configure( 
        m_climbFrontMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters); 
};

void DeepClimb::Periodic() {}

void DeepClimb::SetClimbSpeed(double speed) {
    frc::SmartDashboard::PutNumber("y", speed);
    frc::SmartDashboard::PutNumber("encodeur", encoder.GetPosition());
    m_climbFrontMotor.Set(speed);
}
