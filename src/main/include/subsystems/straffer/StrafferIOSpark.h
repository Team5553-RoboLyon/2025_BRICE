#pragma once

#include "StrafferIO.h"
#include "rev/SparkMax.h"
#include <frc/DigitalInput.h>
#include "StrafferConstants.h"
#include "frc/Encoder.h"
//TODO : verif comment utiliser encoder avec motion

class StrafferIOSpark  final : public StrafferIO
{
  private:
    rev::spark::SparkMax m_motor{strafferConstants::Motor::ID, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_motorConfig;

    frc::DigitalInput m_limitSwitchLeft{strafferConstants::Sensor::LimitSwitch::LEFT_ID};
    frc::DigitalInput m_limitSwitchRight{strafferConstants::Sensor::LimitSwitch::RIGHT_ID};

    frc::Encoder m_encoder{strafferConstants::Sensor::Encoder::A_ID, strafferConstants::Sensor::Encoder::B_ID, strafferConstants::Sensor::Encoder::REVERSED};

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