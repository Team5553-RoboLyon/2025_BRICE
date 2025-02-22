// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() {
    // Set the back left motor configs
    m_MotorBackLeftConfig.SetIdleMode(DriveConstants::LeftGearbox::Motor::MOTOR_IDLE_MODE)
        .Inverted(DriveConstants::LeftGearbox::Motor::MOTOR_INVERTED)
        .SmartCurrentLimit(DriveConstants::LeftGearbox::Motor::MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(DriveConstants::LeftGearbox::Motor::MOTOR_RAMP)
        .VoltageCompensation(DriveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);

    // Set the back right motor configs
    m_MotorBackRightConfig.SetIdleMode(DriveConstants::RightGearbox::Motor::MOTOR_IDLE_MODE)
        .Inverted(DriveConstants::RightGearbox::Motor::MOTOR_INVERTED)
        .SmartCurrentLimit(DriveConstants::RightGearbox::Motor::MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(DriveConstants::RightGearbox::Motor::MOTOR_RAMP)
        .VoltageCompensation(DriveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);

    // Copy the back motor configs to the front motors and follow the back motors
    m_MotorFrontLeftConfig.Apply(m_MotorBackLeftConfig).Follow(m_MotorBackLeft);
    m_MotorFrontRightConfig.Apply(m_MotorBackRightConfig).Follow(m_MotorBackRight);
    
    // Apply the configs to the motors
    m_MotorBackLeft.Configure(m_MotorBackLeftConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_MotorBackRight.Configure(m_MotorBackRightConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_MotorFrontLeft.Configure(m_MotorFrontLeftConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_MotorFrontRight.Configure(m_MotorFrontRightConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    // Set distance per pulse
    m_EncoderLeft.SetDistancePerPulse(DriveConstants::LeftGearbox::Encoder::DISTANCE_PER_PULSE);
    m_EncoderRight.SetDistancePerPulse(DriveConstants::RightGearbox::Encoder::DISTANCE_PER_PULSE);

    m_JoystickPrelimited_V.Reset(0.0, 0.0, ControlPanelConstants::Settings::RATE_LIMITER_FOWARD); // reset des rate limiters
    m_JoystickLimited_V.Reset(0.0, 0.0, ControlPanelConstants::Settings::PRE_RATE_LIMITER_FOWARD);

    m_JoystickPrelimited_W.Reset(0.0, 0.0, ControlPanelConstants::Settings::RATE_LIMITER_ROTATION);
    m_JoystickLimited_W.Reset(0.0, 0.0, ControlPanelConstants::Settings::PRE_RATE_LIMITER_ROTATION);
}

void Drivetrain::SetPower(double v_motor) {
    m_MotorBackLeft.Set(v_motor);
    m_MotorBackRight.Set(v_motor);
}

void Drivetrain::SetVoltage(double voltageLeft, double voltageRight) {
    m_MotorBackLeft.Set(voltageLeft / DriveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);
    m_MotorBackRight.Set(voltageRight / DriveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);
}


void Drivetrain::DriveAuto(double speed, double rotation)
{
    m_MotorBackLeft.Set(speed + rotation);
    m_MotorBackRight.Set(speed - rotation);
}

void Drivetrain::Drive(double FwdJoystick, double RotateJoystick) {
    m_JoystickLimited_V.Update(m_JoystickPrelimited_V.Update(FwdJoystick));
    m_JoystickLimited_W.Update(m_JoystickPrelimited_W.Update(RotateJoystick));


    m_sigma = NLERP(0.5, 0.5, NABS(FwdJoystick)); // 0401

    if (utils::epsilonEquals(FwdJoystick, 0.0, 0.08))
    {
        FwdJoystick = 0.0;
    }
    if (utils::epsilonEquals(RotateJoystick, 0.0, 0.08))
    {
        RotateJoystick = 0.0;
    }
    m_MotorBackLeft.Set(Calcul_Of_Our_Cher_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), DriveConstants::LeftGearbox::WHEEL_SIDE));
    m_MotorBackRight.Set(Calcul_Of_Our_Cher_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), DriveConstants::RightGearbox::WHEEL_SIDE));
}


void Drivetrain::Stop() {
    m_MotorBackLeft.StopMotor();
    m_MotorBackRight.StopMotor();
}



double Drivetrain::Calcul_Of_Our_Cher_JM(double forward, double turn, bool wheelSide) {

    double left_wheel = forward + turn * m_sigma;
    double right_wheel = forward - turn * m_sigma;

    double k;
    k = 1.0 / (NMAX(1, NMAX(NABS(left_wheel), NABS(right_wheel))));
    left_wheel *= k;
    right_wheel *= k;

    if (wheelSide == false)
        return right_wheel;
    else
        return left_wheel;
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}
