// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Encoder.h>
#include "rev/SparkFlex.h"
#include "rev/SparkMax.h"
#include "Constants.h"
#include "lib/UtilsRBL.h"
#include "lib/rate_limiter.h"
#include "lib/NRollingAverage.h"
#include "lib/Dynamic.h"
#include "lib/utils.h"

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

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
  void SetPower(double v_motor);

    /**
   * Sets the voltage for the left and right motors of the drivetrain.
   *
   * @param voltageLeft The voltage to set for the left motor.
   * @param voltageRight The voltage to set for the right motor.
   */
  void SetVoltage(double voltageLeft, double voltageRight);

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
  void DriveAuto(double speed, double rotation);

    /**
   * @brief Drives the robot using joystick inputs.
   *
   * This function drives the robot using the forward and rotation inputs from the joystick.
   * The joystick inputs are used to calculate the wheel speeds for the left and right motors.
   *
   * @param FwdJoystick The forward input value from the joystick, typically ranging from -1.0 to 1.0.
   * @param RotateJoystick The rotation input value from the joystick, typically ranging from -1.0 to 1.0.
   */
  void Drive(double FwdJoystick, double RotateJoystick);

  /**
 * @brief Stops the drivetrain by setting the motor speeds to zero.
 * 
 * This function sets the speed of both the left and right motors to zero,
 * effectively stopping the drivetrain.
 */
  void Stop();

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
  double Calcul_Of_Our_Cher_JM(double forward, double turn, bool wheelSide);

  void Reset();

  void ReverseDrive();

  void Periodic() override;

 private:
  //Definition of the motors
  rev::spark::SparkMax m_MotorFrontLeft{DriveConstants::LeftGearbox::Motor::FRONT_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_MotorBackLeft{DriveConstants::LeftGearbox::Motor::BACK_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_MotorFrontRight{DriveConstants::RightGearbox::Motor::FRONT_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_MotorBackRight{DriveConstants::RightGearbox::Motor::BACK_MOTOR_ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

  //Definition of the motors' configurations
  rev::spark::SparkBaseConfig m_MotorFrontLeftConfig{};
  rev::spark::SparkBaseConfig m_MotorBackLeftConfig{};
  rev::spark::SparkBaseConfig m_MotorFrontRightConfig{};
  rev::spark::SparkBaseConfig m_MotorBackRightConfig{};

  //Definition of the encoders
  frc::Encoder m_EncoderLeft{DriveConstants::LeftGearbox::Encoder::ID_ENCODER_A, DriveConstants::LeftGearbox::Encoder::ID_ENCODER_B, DriveConstants::LeftGearbox::Encoder::REVERSE_ENCODER, frc::Encoder::k4X};
  frc::Encoder m_EncoderRight{DriveConstants::RightGearbox::Encoder::ID_ENCODER_A, DriveConstants::RightGearbox::Encoder::ID_ENCODER_B, DriveConstants::RightGearbox::Encoder::REVERSE_ENCODER, frc::Encoder::k4X};

  double m_sigma = 0.0; //rotation weighting factor
  bool m_reversedDrive = false; //flag to indicate if the drive direction is reversed

  RateLimiter m_JoystickLimited_V;    // joystick V rate limiter, used to smooth out the joystick input for forward/backward movement
  RateLimiter m_JoystickLimited_W;    // joystick W rate limiter, used to smooth out the joystick input for rotation movement
};
