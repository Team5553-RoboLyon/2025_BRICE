// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "frc/smartdashboard/SmartDashboard.h"

Drivetrain::Drivetrain() {
    // Set the back left motor configs
    m_MotorBackLeftConfig.SetIdleMode(driveConstants::LeftGearbox::Motor::MOTOR_IDLE_MODE)
        .Inverted(driveConstants::LeftGearbox::Motor::MOTOR_INVERTED)
        .SmartCurrentLimit(driveConstants::LeftGearbox::Motor::MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::LeftGearbox::Motor::MOTOR_RAMP)
        .VoltageCompensation(driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);

    // Set the back right motor configs
    m_MotorBackRightConfig.SetIdleMode(driveConstants::RightGearbox::Motor::MOTOR_IDLE_MODE)
        .Inverted(driveConstants::RightGearbox::Motor::MOTOR_INVERTED)
        .SmartCurrentLimit(driveConstants::RightGearbox::Motor::MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::RightGearbox::Motor::MOTOR_RAMP)
        .VoltageCompensation(driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);

    // Copy the back motor configs to the front motors and follow the back motors
    m_MotorFrontLeftConfig.Apply(m_MotorBackLeftConfig).Follow(m_MotorBackLeft);
    m_MotorFrontRightConfig.Apply(m_MotorBackRightConfig).Follow(m_MotorBackRight);
    
    // Apply the configs to the motors
    m_MotorBackLeft.Configure(m_MotorBackLeftConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_MotorBackRight.Configure(m_MotorBackRightConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_MotorFrontLeft.Configure(m_MotorFrontLeftConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_MotorFrontRight.Configure(m_MotorFrontRightConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters, rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    // Set distance per pulse
    m_EncoderLeft.SetDistancePerPulse(driveConstants::LeftGearbox::Encoder::DISTANCE_PER_PULSE);
    m_EncoderRight.SetDistancePerPulse(driveConstants::RightGearbox::Encoder::DISTANCE_PER_PULSE);

    m_JoystickLimited_V.Reset(0.0, 0.0, ControlPanelConstants::Settings::RATE_LIMITER_FOWARD);
    m_JoystickLimited_W.Reset(0.0, 0.0, ControlPanelConstants::Settings::RATE_LIMITER_ROTATION);
}

void Drivetrain::SetPower(double v_motor) {
    m_MotorBackLeft.Set(v_motor);
    m_MotorBackRight.Set(v_motor);
}

void Drivetrain::SetVoltage(double voltageLeft, double voltageRight) {
    m_MotorBackLeft.Set(voltageLeft / driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);
    m_MotorBackRight.Set(voltageRight / driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);
}


// void Drivetrain::DriveAuto(double speed, double rotation)
// {
//     // // m_MotorBackLeft.Set(speed);
//     // // m_MotorBackRight.Set(speed);
//     // m_MotorBackLeft.Set(Calcul_Of_Our_Cher_JM(speed, rotation, DriveConstants::LeftGearbox::WHEEL_SIDE));
//     // m_MotorBackRight.Set(Calcul_Of_Our_Cher_JM(speed, rotation, DriveConstants::RightGearbox::WHEEL_SIDE));
// }
double Drivetrain::DriveAuto()
{
    return m_EncoderLeft.GetDistance();
}

void Drivetrain::Drive(double FwdJoystick, double RotateJoystick, double heightFactor) {
    if(NABS(FwdJoystick)<ControlPanelConstants::Settings::DEADBAND)
    {
        FwdJoystick = 0.0;
    }
    if(NABS(RotateJoystick)<ControlPanelConstants::Settings::DEADBAND)
    {
        RotateJoystick = 0.0;
    }
    //Reversed drive mode
    if(m_reversedDrive)
        FwdJoystick = -FwdJoystick;

    //Slower mode
    if(slower) {
        FwdJoystick = FwdJoystick / ControlPanelConstants::Settings::SLOW_RATE;
        RotateJoystick = RotateJoystick / ControlPanelConstants::Settings::SLOW_RATE;
    }

    // Protect from falling
    double h = (1.0-heightFactor);
    double minMovingV = FwdJoystick * ControlPanelConstants::Settings::MIN_MOVING_V;
    double minMovingW = RotateJoystick * ControlPanelConstants::Settings::MIN_MOVING_W;
    if(FwdJoystick < 0.0)
    {
        FwdJoystick = NMAX(FwdJoystick, -h*h);
    }
    else 
    {
        FwdJoystick = NMIN(FwdJoystick, h*h);
    }

    if(RotateJoystick < 0.0)
    {
        RotateJoystick = NMAX(RotateJoystick, -h);
    }
    else 
    {
        RotateJoystick = NMIN(RotateJoystick, h);
    }
    FwdJoystick += minMovingV;
    RotateJoystick += minMovingW;
    

    //Limit the rate of change of the joystick
    m_JoystickLimited_V.Update(FwdJoystick);
    m_JoystickLimited_W.Update(RotateJoystick);

    m_sigma = NLERP(0.1, 0.4, NABS(RotateJoystick));

    m_MotorBackLeft.Set(Calcul_Of_Our_Cher_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), driveConstants::LeftGearbox::WHEEL_SIDE));
    m_MotorBackRight.Set(Calcul_Of_Our_Cher_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), driveConstants::RightGearbox::WHEEL_SIDE));
    frc::SmartDashboard::PutNumber("dLeft Power", m_MotorBackLeft.Get());
    frc::SmartDashboard::PutNumber("dRight Power", m_MotorBackRight.Get());

    frc::SmartDashboard::PutNumber("dLeft Encoder", m_EncoderLeft.GetDistance());
    frc::SmartDashboard::PutNumber("dRight Encoder", m_EncoderRight.GetDistance());


    frc::SmartDashboard::PutBoolean("dReversed drive", m_reversedDrive);
    frc::SmartDashboard::PutBoolean("dSlower", slower);
    frc::SmartDashboard::PutNumber("dSigma", m_sigma);
    frc::SmartDashboard::PutNumber("dFWD Joystick", FwdJoystick);
    frc::SmartDashboard::PutNumber("dRotate Joystick", RotateJoystick);
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

void Drivetrain::ReverseDrive() {
    m_reversedDrive = !m_reversedDrive;
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}