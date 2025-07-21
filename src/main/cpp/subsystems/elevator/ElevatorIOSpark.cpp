#include "subsystems/elevator/ElevatorIOSpark.h"

#include "frc/smartdashboard/SmartDashboard.h"
#include "lib/DebugUtils.h"
ElevatorIOSpark::ElevatorIOSpark()
{
    // Set the left motor configs
    m_leftMotorConfig.SetIdleMode(elevatorConstants::Motors::IDLE_MODE)
        .Inverted(elevatorConstants::Motors::INVERTED_LEFT)
        .SmartCurrentLimit(elevatorConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(elevatorConstants::Motors::RAMP_RATE)
        .VoltageCompensation(elevatorConstants::Motors::VOLTAGE_COMPENSATION);

    // Set the right motor configs
    m_rightMotorConfig.SetIdleMode(elevatorConstants::Motors::IDLE_MODE)
        .Inverted(elevatorConstants::Motors::INVERTED_RIGHT)
        .SmartCurrentLimit(elevatorConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(elevatorConstants::Motors::RAMP_RATE)
        .VoltageCompensation(elevatorConstants::Motors::VOLTAGE_COMPENSATION);

    // Apply the configs to the motors
    m_leftMotor.Configure(  m_leftMotorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_rightMotor.Configure( m_rightMotorConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_leftMotor.ClearFaults();
    m_rightMotor.ClearFaults();

    m_encoder.Reset();
    m_encoder.SetDistancePerPulse(elevatorConstants::Encoder::DISTANCE_PER_PULSE);
}

void ElevatorIOSpark::UpdateInputs(ElevatorIOInputs& inputs) 
{
    inputs.isLeftMotorConnected = (m_leftMotor.GetBusVoltage() !=0.0) && !m_leftMotor.GetFaults().can;
    inputs.isRightMotorConnected = (m_rightMotor.GetBusVoltage() !=0.0) && !m_rightMotor.GetFaults().can;

    inputs.leftMotorAppliedVoltage = m_leftMotor.GetAppliedOutput() * elevatorConstants::Motors::VOLTAGE_COMPENSATION;
    inputs.rightMotorAppliedVoltage = m_rightMotor.GetAppliedOutput() * elevatorConstants::Motors::VOLTAGE_COMPENSATION;
    inputs.leftMotorBusVoltage = m_leftMotor.GetBusVoltage();
    inputs.rightMotorBusVoltage = m_rightMotor.GetBusVoltage();
    inputs.leftMotorCurrent = m_leftMotor.GetOutputCurrent();
    inputs.rightMotorCurrent = m_rightMotor.GetOutputCurrent();
    inputs.leftMotorTemperature = m_leftMotor.GetMotorTemperature();
    inputs.rightMotorTemperature = m_rightMotor.GetMotorTemperature();

    inputs.limitSwitchBottom = m_bottomLimitSwitch.Get() == elevatorConstants::LimitSwitch::IS_TRIGGERED;
    inputs.limitSwitchBottom2 = m_bottomLimitSwitch2.Get() == elevatorConstants::LimitSwitch::IS_TRIGGERED;

    inputs.heightPosition = m_encoder.GetDistance();

    frc::SmartDashboard::PutBoolean("El.Connection", inputs.isLeftMotorConnected);
    frc::SmartDashboard::PutBoolean("Er.Connection", inputs.isRightMotorConnected);

    frc::SmartDashboard::PutBoolean("E.LimitSwitch", inputs.limitSwitchBottom);
    frc::SmartDashboard::PutBoolean("E.LimitSwitch2", inputs.limitSwitchBottom2);
    frc::SmartDashboard::PutNumber("E.Height Position", inputs.heightPosition);
}

void ElevatorIOSpark::SetVoltage(double voltage)
{
    DEBUG_ASSERT((voltage <= elevatorConstants::Motors::VOLTAGE_COMPENSATION) 
        && (voltage >= -elevatorConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Elevator Voltage out of range");
    
    m_leftMotor.SetVoltage(units::volt_t(voltage));
    m_rightMotor.SetVoltage(units::volt_t(voltage));
}

void ElevatorIOSpark::SetDutyCycle(double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0) 
            ,"Straffer Duty Cycle out of range");
    
    m_leftMotor.Set(dutyCycle);
    m_rightMotor.Set(dutyCycle);
}

void ElevatorIOSpark::ResetPosition()
{
    m_encoder.Reset();
}