#pragma once

#include "GripperIO.h"
#include "rev/SparkMax.h"
#include <frc/DigitalInput.h>
#include "GripperConstants.h"
#include  "lib/pidRBL.h"
class GripperIOSpark final : public GripperIO
{
  private:
    rev::spark::SparkMax m_outtakeMotor{outtakeConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax m_feederMotor{feederConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};

    rev::spark::SparkBaseConfig m_outtakeMotorConfig;
    rev::spark::SparkBaseConfig m_feederMotorConfig;

    frc::DigitalInput m_IRBreakerDown{gripperConstants::IRbreaker::DOWN_ID};
    frc::DigitalInput m_IRBreakerUp{gripperConstants::IRbreaker::UP_ID};
    frc::DigitalInput m_IRBreakerUp2{gripperConstants::IRbreaker::UP2_ID};

    PidRBL m_feederVelocityPID{
        feederConstants::VelocityPID::KP, 
        feederConstants::VelocityPID::KI,
        feederConstants::VelocityPID::KD,
        feederConstants::VelocityPID::KFF
    };
    PidRBL m_outtakeVelocityPID{
        outtakeConstants::VelocityPID::KP,
        outtakeConstants::VelocityPID::KI,
        outtakeConstants::VelocityPID::KD,
        outtakeConstants::VelocityPID::KFF
    };

    double m_feederVelocity{0.0};
    double m_outtakeVelocity{0.0};
    double m_timestamp{0.0};

  public:
    GripperIOSpark();
    ~GripperIOSpark() = default;

    void UpdateInputs(GripperIOInputs& inputs) override; //COMMENTME
    void SetFeederVoltage(const double voltage) override; //COMMENTME
    void SetOuttakeVoltage(const double voltage) override; //COMMENTME

    void SetFeederDutyCycle(const double dutyCycle) override; //COMMENTME
    void SetOuttakeDutyCycle(const double dutyCycle) override; //COMMENTME

    void SetFeederRPM(const double RPM) override; //COMMENTME
    void SetOuttakeRPM(const double RPM) override; //COMMENTME
};