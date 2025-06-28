#include "subsystems/gripper/GripperIOSpark.h"

#include <assert.h>
#include <frc/smartdashboard/SmartDashboard.h>

GripperIOSpark::GripperIOSpark()
{
    m_outtakeMotorConfig.SetIdleMode(outtakeConstants::Motor::IDLE_MODE)
        .Inverted(outtakeConstants::Motor::INVERTED)
        .SmartCurrentLimit(outtakeConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(outtakeConstants::Motor::RAMP_RATE)
        .VoltageCompensation(outtakeConstants::Motor::VOLTAGE_COMPENSATION);
    
    m_feederMotorConfig.SetIdleMode(feederConstants::Motor::IDLE_MODE)
        .Inverted(feederConstants::Motor::INVERTED)
        .SmartCurrentLimit(feederConstants::Motor::CURRENT_LIMIT)
        .ClosedLoopRampRate(feederConstants::Motor::RAMP_RATE)
        .VoltageCompensation(feederConstants::Motor::VOLTAGE_COMPENSATION);

    m_outtakeMotor.Configure(  m_outtakeMotorConfig, 
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);

    m_feederMotor.Configure(  m_feederMotorConfig, 
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);
    m_feederMotor.ClearFaults();
    m_outtakeMotor.ClearFaults();

    m_feederVelocityPID.SetOutputLimits(feederConstants::VelocityPID::MIN, 
                                        feederConstants::VelocityPID::MAX);
    m_outtakeVelocityPID.SetOutputLimits(outtakeConstants::VelocityPID::MIN, 
                                        outtakeConstants::VelocityPID::MAX); 
    }

void GripperIOSpark::UpdateInputs(GripperIOInputs& inputs)
{
    m_feederVelocity = m_feederMotor.GetEncoder().GetVelocity() / feederConstants::GEAR_RATIO;
    m_outtakeVelocity = m_outtakeMotor.GetEncoder().GetVelocity() / outtakeConstants::GEAR_RATIO;

    inputs.isFeederMotorConnected = (m_feederMotor.GetBusVoltage() !=0.0) && !m_feederMotor.GetFaults().can;
    inputs.isOuttakeMotorConnected = (m_outtakeMotor.GetBusVoltage() != 0.0) && !m_outtakeMotor.GetFaults().can;

    inputs.feederAppliedVoltage = m_feederMotor.GetAppliedOutput() * feederConstants::Motor::VOLTAGE_COMPENSATION;
    inputs.outtakeAppliedVoltage = m_outtakeMotor.GetAppliedOutput() * outtakeConstants::Motor::VOLTAGE_COMPENSATION;
    inputs.feederBusVoltage = m_feederMotor.GetBusVoltage();
    inputs.outtakeBusVoltage = m_outtakeMotor.GetBusVoltage();
    inputs.feederCurrent = m_feederMotor.GetOutputCurrent();
    inputs.outtakeCurrent = m_outtakeMotor.GetOutputCurrent();
    inputs.feederTemperature = m_feederMotor.GetMotorTemperature();
    inputs.outtakeTemperature = m_outtakeMotor.GetMotorTemperature();
    inputs.feederRPM = m_feederVelocity;
    inputs.outtakeRPM = m_outtakeVelocity;

    inputs.IRBreakerUp = m_IRBreakerUp.Get() == gripperConstants::IRbreaker::IS_TRIGGERED;
    inputs.IRBreakerDown = m_IRBreakerDown.Get() == gripperConstants::IRbreaker::IS_TRIGGERED;
    inputs.IRBreakerUp2 = m_IRBreakerUp2.Get() == gripperConstants::IRbreaker::IS_TRIGGERED;


    //only while waiting for AdScope (very bad performance)
    frc::SmartDashboard::PutBoolean("O.Connection", inputs.isOuttakeMotorConnected);
    frc::SmartDashboard::PutNumber("O.Voltage", inputs.outtakeAppliedVoltage);
    frc::SmartDashboard::PutNumber("O.BusVolt", inputs.outtakeBusVoltage);
    frc::SmartDashboard::PutNumber("O.Current", inputs.outtakeCurrent);
    frc::SmartDashboard::PutNumber("O.Temperature", inputs.outtakeTemperature);
    frc::SmartDashboard::PutNumber("O.Velocity", inputs.outtakeRPM);

    frc::SmartDashboard::PutBoolean("F.Connection", inputs.isFeederMotorConnected);
    frc::SmartDashboard::PutNumber("F.Voltage", inputs.feederAppliedVoltage);
    frc::SmartDashboard::PutNumber("F.BusVolt", inputs.feederBusVoltage);
    frc::SmartDashboard::PutNumber("F.Current", inputs.feederCurrent);
    frc::SmartDashboard::PutNumber("F.Temperature", inputs.feederTemperature);
    frc::SmartDashboard::PutNumber("F.Velocity", inputs.feederRPM);

    frc::SmartDashboard::PutBoolean("G.Up IRbreaker", inputs.IRBreakerUp);
    frc::SmartDashboard::PutBoolean("G.Up2 IRbreaker", inputs.IRBreakerUp2);
    frc::SmartDashboard::PutNumber("G.Down IRbreaker", inputs.IRBreakerDown);
}

void GripperIOSpark::SetFeederVoltage(const double voltage) 
{
    assert((voltage <= feederConstants::Motor::VOLTAGE_COMPENSATION) 
    && (voltage >= -feederConstants::Motor::VOLTAGE_COMPENSATION) 
    && "Feeder Voltage out of range");
    m_feederMotor.SetVoltage(units::volt_t(voltage));
}

void GripperIOSpark::SetOuttakeVoltage(const double voltage)
{
    assert((voltage <= outtakeConstants::Motor::VOLTAGE_COMPENSATION) 
    && (voltage >= -outtakeConstants::Motor::VOLTAGE_COMPENSATION) 
    && "Outtake Voltage out of range");
    m_outtakeMotor.SetVoltage(units::volt_t(voltage));
}

void GripperIOSpark::SetFeederDutyCycle(const double dutyCycle)
{
    assert((dutyCycle <= 1.0) && (dutyCycle >= -1.0) && "Feeder Duty Cycle out of range");
    m_feederMotor.Set(dutyCycle);
}

void GripperIOSpark::SetOuttakeDutyCycle(const double dutyCycle)
{
    assert((dutyCycle <= 1.0) && (dutyCycle >= -1.0) && "Outtake Duty Cycle out of range");
    m_outtakeMotor.Set(dutyCycle);
}

void GripperIOSpark::SetFeederRPM(const double RPM)
{
    m_feederMotor.Set(m_feederVelocityPID.Calculate(RPM, m_feederVelocity));
}

void GripperIOSpark::SetOuttakeRPM(const double RPM)
{
    m_outtakeMotor.Set(m_outtakeVelocityPID.Calculate(RPM, m_outtakeVelocity));
}