#pragma once

#include <frc/DigitalInput.h>
#include "rev/SparkMax.h"
#include "frc/Encoder.h"

#include "ElevatorIO.h"
#include "ElevatorConstants.h"

class ElevatorIOSpark  final : public ElevatorIO
{
  private:
    rev::spark::SparkMax m_leftMotor{elevatorConstants::Motors::ID_LEFT, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_leftMotorConfig;
    rev::spark::SparkMax m_rightMotor{elevatorConstants::Motors::ID_RIGHT, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkBaseConfig m_rightMotorConfig;

    frc::Encoder m_encoder{elevatorConstants::Encoder::A_ID, elevatorConstants::Encoder::B_ID, elevatorConstants::Encoder::REVERSED};
    
    frc::DigitalInput m_bottomLimitSwitch{elevatorConstants::LimitSwitch::BOTTOM_ID};
    frc::DigitalInput m_bottomLimitSwitch2{elevatorConstants::LimitSwitch::BOTTOM_2_ID};

  public:
    ElevatorIOSpark();
    ~ElevatorIOSpark() = default;

    void UpdateInputs(ElevatorIOInputs& inputs) override; //COMMENTME
    void SetVoltage(double voltage) override; //COMMENTME
    void SetDutyCycle(double dutyCycle) override; //COMMENTME
    void ResetPosition() override; //COMMENTME
};