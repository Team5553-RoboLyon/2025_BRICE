#pragma once

#include "DrivetrainIO.h"
#include "rev/SparkFlex.h"
#include "DrivetrainConstants.h"
#include "frc/Encoder.h"

class DrivetrainIOFlex  final : public DrivetrainIO
{
  private:
  rev::spark::SparkFlex m_motorFrontLeft{driveConstants::LeftGearbox::Motor::FRONT_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorBackLeft{driveConstants::LeftGearbox::Motor::BACK_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorFrontRight{driveConstants::RightGearbox::Motor::FRONT_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorBackRight{driveConstants::RightGearbox::Motor::BACK_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};

  rev::spark::SparkBaseConfig m_motorFrontLeftConfig{};
  rev::spark::SparkBaseConfig m_motorBackLeftConfig{};
  rev::spark::SparkBaseConfig m_motorFrontRightConfig{};
  rev::spark::SparkBaseConfig m_motorBackRightConfig{};

  frc::Encoder m_encoderLeft{driveConstants::LeftGearbox::Encoder::ID_ENCODER_A, 
                            driveConstants::LeftGearbox::Encoder::ID_ENCODER_B, 
                            driveConstants::LeftGearbox::Encoder::REVERSE_ENCODER};
  frc::Encoder m_encoderRight{driveConstants::RightGearbox::Encoder::ID_ENCODER_A, 
                            driveConstants::RightGearbox::Encoder::ID_ENCODER_B,
                            driveConstants::RightGearbox::Encoder::REVERSE_ENCODER};

  public:
    DrivetrainIOFlex();
    ~DrivetrainIOFlex() = default;

    void UpdateInputs(DrivetrainIOInputs& inputs) override;

    void SetVoltage(double leftSideVoltage, double rightSideVoltage) override;
    void SetDutyCycle(double leftSideDutyCycle, double rightSideDutyCycle) override;
    void ResetPosition() override;
};