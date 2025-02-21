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
        .Follow(m_climbFrontMotor, DeepClimbConstants::Motors::INVERTED);
    
    m_climbFrontMotor.Configure( 
        m_climbFrontMotorConfig,
        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
        rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    //configure the encoder
    m_climbEncoder.SetDistancePerPulse(DeepClimbConstants::Encoder::Settings::DISTANCE_PER_PULSE);
    m_climbEncoder.Reset();
    
    //configure the PID
    m_pid.SetTolerance(DeepClimbConstants::PID::TOLERANCE);
}

void DeepClimb::Periodic() {
    //frc::SmartDashboard::PutNumber("climb encoder", m_climbEncoder.GetDistance());
};

void DeepClimb::SetClimbSpeed(double speed) {
    m_climbFrontMotor.Set(speed);
}

void DeepClimb::StopClimb() {
    m_climbFrontMotor.Set(0);
}

double DeepClimb::GetArmAngle() {
    return m_climbEncoder.GetDistance();
}

void DeepClimb::ResetClimberPosition() {
    //stop when it is at 90% of the max
    //sinon tourner
    if (!m_hallEffectSensorDown.Get()){
        m_climbFrontMotor.Set(0.1);
    }
    else{
        m_climbFrontMotor.Set(0);
        m_climbEncoder.Reset();
    }
}

void DeepClimb::GoToPosition() {
    if (!m_hallEffectSensorUp.Get()){
        SetClimbSpeed(m_pid.Calculate(m_climbEncoder.GetDistance()));
    }
    else{
        StopClimb();
    }
}

bool DeepClimb::IsClimbed(){
    m_isClimbed = m_pid.AtSetpoint();
return m_isClimbed;
}