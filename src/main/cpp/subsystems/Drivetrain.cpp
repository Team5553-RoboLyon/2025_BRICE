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
};

/**
 * @brief Sets the power for the drivetrain motors.
 * 
 * This function sets the power level for both the back left and back right motors
 * of the drivetrain to the specified value.
 * 
 * @param v_motor The power level to set for the motors. This value is typically
 *                between -1.0 and 1.0, where -1.0 represents full reverse power,
 *                0.0 represents no power, and 1.0 represents full forward power.
 */
void Drivetrain::SetPower(double v_motor) {
    m_MotorBackLeft.Set(v_motor);
    m_MotorBackRight.Set(v_motor);
}

/**
 * Sets the voltage for the left and right motors of the drivetrain.
 *
 * @param voltageLeft The voltage to set for the left motor.
 * @param voltageRight The voltage to set for the right motor.
 */
void Drivetrain::SetVoltage(double voltageLeft, double voltageRight) {
    m_MotorBackLeft.Set(voltageLeft / DriveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);
    m_MotorBackRight.Set(voltageRight / DriveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);
}

/**
 * @brief Drives the robot autonomously by setting the speed and rotation of the motors.
 *
 * This function sets the speed and rotation for the back left and back right motors
 * to drive the robot autonomously. The speed and rotation values are combined to 
 * determine the final motor outputs.
 *
 * @param speed The forward/backward speed of the robot. Positive values move the robot forward, 
 *              and negative values move it backward.
 * @param rotation The rotational speed of the robot. Positive values rotate the robot clockwise, 
 *                 and negative values rotate it counterclockwise.
 */
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
    m_MotorBackLeft.Set(Calcul_De_Notre_Cher_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), DriveConstants::LeftGearbox::WHEEL_SIDE));
    m_MotorBackRight.Set(Calcul_De_Notre_Cher_JM(m_JoystickLimited_V.m_current, std::sin(m_JoystickLimited_W.m_current * (NF64_PI / 2)), DriveConstants::RightGearbox::WHEEL_SIDE));
}

/**
 * @brief Stops the drivetrain by setting the motor speeds to zero.
 * 
 * This function sets the speed of both the left and right motors to zero,
 * effectively stopping the drivetrain.
 */
void Drivetrain::Stop() {
    m_MotorBackLeft.Set(0.0);
    m_MotorBackRight.Set(0.0);
}


/**
 * @brief Calculates the wheel speeds for the drivetrain based on forward and turn inputs.
 *
 * This function computes the speeds for the left and right wheels of the drivetrain
 * using the provided forward and turn inputs. The wheel speeds are normalized to ensure
 * that neither exceeds a magnitude of 1.0.
 *
 * @param forward The forward input value, typically ranging from -1.0 to 1.0.
 * @param turn The turn input value, typically ranging from -1.0 to 1.0.
 * @param wheelSide A boolean indicating which wheel speed to return. If false, the function
 *                  returns the right wheel speed; if true, it returns the left wheel speed.
 * @return The speed of the specified wheel (left or right), normalized to a maximum magnitude of 1.0.
 */
double Drivetrain::Calcul_De_Notre_Cher_JM(double forward, double turn, bool wheelSide) {
    double m_forward = forward;
    double m_turn = turn;

    double left_wheel = m_forward + m_turn * m_sigma;
    double right_wheel = m_forward - m_turn * m_sigma;

    double k;
    k = 1.0 / (NMAX(1, NMAX(NABS(left_wheel), NABS(right_wheel))));
    left_wheel *= k;
    right_wheel *= k;

    if (wheelSide == 0)
        return right_wheel;
    else
        return left_wheel;

};

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}
