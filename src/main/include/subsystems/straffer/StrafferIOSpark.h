#pragma once

#include <frc/DigitalInput.h>
#include "rev/SparkMax.h"
#include "frc/Encoder.h"

#include "StrafferConstants.h"
#include "StrafferIO.h"

class StrafferIOSpark  final : public StrafferIO
{
  private:
    rev::spark::SparkMax m_motor{strafferConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_motorConfig;

    frc::DigitalInput m_limitSwitchLeft{strafferConstants::LimitSwitch::LEFT_ID};
    frc::DigitalInput m_limitSwitchRight{strafferConstants::LimitSwitch::RIGHT_ID};

    frc::Encoder m_encoder{strafferConstants::Encoder::A_ID, strafferConstants::Encoder::B_ID, strafferConstants::Encoder::REVERSED};

    double m_H2Offset{0.0}; //COMMENTME

  public:
    StrafferIOSpark();
    ~StrafferIOSpark() = default;

    void UpdateInputs(StrafferIOInputs& inputs) override; //COMMENTME
    void SetVoltage(const double voltage) override; //COMMENTME
    void SetDutyCycle(const double dutyCycle) override; //COMMENTME
    void ResetPosition() override; //COMMENTME
    void ResetPositionLeft() override; //COMMENTME
    void ResetPositionRight() override; //COMMENTME
};