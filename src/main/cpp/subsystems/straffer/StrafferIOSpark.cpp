#include "subsystems/straffer/StrafferIOSpark.h"

#include "lib/DebugUtils.h"
#include <frc/smartdashboard/SmartDashboard.h>

StrafferIOSpark::StrafferIOSpark()
{
    m_motorConfig.SetIdleMode(strafferConstants::Motor::IDLE_MODE)
        .Inverted(strafferConstants::Motor::INVERTED)
        .SmartCurrentLimit(strafferConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(strafferConstants::Motor::RAMP_RATE)
        .VoltageCompensation(strafferConstants::Motor::VOLTAGE_COMPENSATION);

    m_motor.Configure(  m_motorConfig, 
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_motor.ClearFaults();

    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(strafferConstants::Sensor::Encoder::DISTANCE_PER_PULSE);
}

void StrafferIOSpark::UpdateInputs(StrafferIOInputs& inputs)
{
    inputs.isMotorConnected = (m_motor.GetBusVoltage() !=0.0) && !m_motor.GetFaults().can; //IFBUG : float != float -> epsilon

    inputs.appliedVoltage = m_motor.GetAppliedOutput() * strafferConstants::Motor::VOLTAGE_COMPENSATION;
    inputs.busVoltage = m_motor.GetBusVoltage();
    inputs.current = m_motor.GetOutputCurrent();
    inputs.temperature = m_motor.GetMotorTemperature();

    inputs.limitSwitchLeft = m_limitSwitchLeft.Get() == strafferConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    inputs.limitSwitchRight = m_limitSwitchRight.Get() == strafferConstants::Sensor::LimitSwitch::IS_TRIGGERED;
    inputs.widthPosition = m_encoder.GetDistance() + m_H2Offset; //COMMENTME

    //only while waiting for AdScope (very bad performance)
    frc::SmartDashboard::PutBoolean("S.Connection", inputs.isMotorConnected);
    frc::SmartDashboard::PutNumber("S.Voltage", inputs.appliedVoltage);
    frc::SmartDashboard::PutNumber("S.BusVolt", inputs.busVoltage);
    frc::SmartDashboard::PutNumber("S.Current", inputs.current);
    frc::SmartDashboard::PutNumber("S.Temperature", inputs.temperature);
    frc::SmartDashboard::PutBoolean("S.Left LimitSwitch", inputs.limitSwitchLeft);
    frc::SmartDashboard::PutBoolean("S.Right LimitSwitch", inputs.limitSwitchRight);
    frc::SmartDashboard::PutNumber("S.Width Position", inputs.widthPosition);
}

void StrafferIOSpark::SetVoltage(const double voltage) 
{
    DEBUG_ASSERT((voltage <= strafferConstants::Motor::VOLTAGE_COMPENSATION) 
        && (voltage >= -strafferConstants::Motor::VOLTAGE_COMPENSATION) 
        , "Straffer Voltage out of range");
    m_motor.SetVoltage(units::volt_t(voltage));
}
void StrafferIOSpark::SetDutyCycle(const double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
            , "Straffer Duty Cycle out of range");
    m_motor.Set(dutyCycle);
}
void StrafferIOSpark::ResetPosition() 
{
    m_encoder.Reset();
}
void StrafferIOSpark::ResetPositionLeft()
{
    m_encoder.Reset();
    m_H2Offset = strafferConstants::Settings::LEFT_LIMIT;
}
void StrafferIOSpark::ResetPositionRight()
{
    m_encoder.Reset();
    m_H2Offset = strafferConstants::Settings::RIGHT_LIMIT;
}