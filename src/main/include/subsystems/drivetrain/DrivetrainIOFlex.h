#pragma once

#include "DrivetrainIO.h"
#include "rev/SparkFlex.h"
#include "DrivetrainConstants.h"
// #include <frc/smartdashboard/Field2d.h>
#include "frc/Encoder.h"
#include "lib/localization/TankOdometryTracker.h"

class DrivetrainIOFlex  final : public DrivetrainIO
{
  private:
  rev::spark::SparkFlex m_motorFrontLeft{driveConstants::Motors::LEFT_FRONT_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorBackLeft{driveConstants::Motors::LEFT_BACK_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorFrontRight{driveConstants::Motors::RIGHT_FRONT_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};
  rev::spark::SparkFlex m_motorBackRight{driveConstants::Motors::RIGHT_BACK_MOTOR_ID, 
                                        rev::spark::SparkFlex::MotorType::kBrushless};

  rev::spark::SparkBaseConfig m_motorFrontLeftConfig{};
  rev::spark::SparkBaseConfig m_motorBackLeftConfig{};
  rev::spark::SparkBaseConfig m_motorFrontRightConfig{};
  rev::spark::SparkBaseConfig m_motorBackRightConfig{};

  frc::Encoder m_encoderLeft{driveConstants::Encoder::LEFT_ID_ENCODER_A, 
                            driveConstants::Encoder::LEFT_ID_ENCODER_B, 
                            driveConstants::Encoder::LEFT_REVERSE_ENCODER};
  frc::Encoder m_encoderRight{driveConstants::Encoder::RIGHT_ID_ENCODER_A,
                            driveConstants::Encoder::RIGHT_ID_ENCODER_B,
                            driveConstants::Encoder::RIGHT_REVERSE_ENCODER};

  double m_realLeftSideSpeed{0.0};
  double m_realRightSideSpeed{0.0};

  // frc::Field2d field; for sim only
  TankOdometryTracker m_odometry{&m_realLeftSideSpeed, &m_realRightSideSpeed, 
                                  driveConstants::Specifications::TRACKWIDTH, 
                                  driveConstants::Settings::ALPHA_ODOMETRY};

  public:
    DrivetrainIOFlex();
    ~DrivetrainIOFlex() = default;

    void UpdateInputs(DrivetrainIOInputs& inputs) override;

    void SetVoltage(const double leftSideVoltage, const double rightSideVoltage) override;
    void SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle) override;
    void SetChassisSpeed(const frc::ChassisSpeeds &speeds) override;
    
    void ResetPosition(const frc::Pose2d position) override;
};