#include "subsystems/gripper/GripperIOSpark.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include "lib/DebugUtils.h"
#include "lib/TimerRBL.h"

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

    m_feederVelocityPID.SetOutputLimits(feederConstants::Gains::VelocityPID::MIN, 
                                        feederConstants::Gains::VelocityPID::MAX);
    m_outtakeVelocityPID.SetOutputLimits(outtakeConstants::Gains::VelocityPID::MIN, 
                                        outtakeConstants::Gains::VelocityPID::MAX); 
    m_feederVelocityPID.SetInputLimits(true);
    m_feederVelocityPID.SetInputLimits(feederConstants::Velocity::MIN, 
                                        feederConstants::Velocity::MAX);
    m_outtakeVelocityPID.SetInputLimits(true);
    m_outtakeVelocityPID.SetInputLimits(outtakeConstants::Velocity::MIN,
                                        outtakeConstants::Velocity::MAX);
    

    }

void GripperIOSpark::UpdateInputs(GripperIOInputs& inputs)
{
    m_feederVelocity = m_feederMotor.GetEncoder().GetVelocity() / feederConstants::Specifications::GEAR_RATIO;
    m_outtakeVelocity = m_outtakeMotor.GetEncoder().GetVelocity() / feederConstants::Specifications::GEAR_RATIO;

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

    frc::SmartDashboard::PutBoolean("Gripper/Outtake/Connection", inputs.isOuttakeMotorConnected);
    frc::SmartDashboard::PutBoolean("Gripper/Feeder/Connection", inputs.isFeederMotorConnected);

    frc::SmartDashboard::PutBoolean("Gripper/Sensors/Up IRbreaker", inputs.IRBreakerUp);
    frc::SmartDashboard::PutBoolean("Gripper/Sensors/Up2 IRbreaker", inputs.IRBreakerUp2);
    frc::SmartDashboard::PutBoolean("Gripper/Sensors/Down IRbreaker", inputs.IRBreakerDown);
}

void GripperIOSpark::SetFeederVoltage(const double voltage) 
{
    DEBUG_ASSERT((voltage <= feederConstants::Motor::VOLTAGE_COMPENSATION) 
    && (voltage >= -feederConstants::Motor::VOLTAGE_COMPENSATION) 
    ,"Feeder Voltage out of range");
    m_feederMotor.SetVoltage(units::volt_t(voltage));

    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_outtakeVelocityPID.Reset(m_timestamp);
    m_feederVelocityPID.Reset(m_timestamp);
}

void GripperIOSpark::SetOuttakeVoltage(const double voltage)
{
    DEBUG_ASSERT((voltage <= outtakeConstants::Motor::VOLTAGE_COMPENSATION) 
    && (voltage >= -outtakeConstants::Motor::VOLTAGE_COMPENSATION) 
    ,"Outtake Voltage out of range");
    m_outtakeMotor.SetVoltage(units::volt_t(voltage));

    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_outtakeVelocityPID.Reset(m_timestamp);
    m_feederVelocityPID.Reset(m_timestamp);
}

void GripperIOSpark::SetFeederDutyCycle(const double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0),"Feeder Duty Cycle out of range");
    m_feederMotor.Set(dutyCycle);

    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_outtakeVelocityPID.Reset(m_timestamp);
    m_feederVelocityPID.Reset(m_timestamp);
}

void GripperIOSpark::SetOuttakeDutyCycle(const double dutyCycle)
{
    DEBUG_ASSERT((dutyCycle <= 1.0) && (dutyCycle >= -1.0),"Outtake Duty Cycle out of range");
    m_outtakeMotor.Set(dutyCycle);

    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_outtakeVelocityPID.Reset(m_timestamp);
    m_feederVelocityPID.Reset(m_timestamp);
}

void GripperIOSpark::SetFeederRPM(const double RPM)
{
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_feederVelocityPID.SetFeedforward(NSIGN(RPM - m_feederVelocity) * 
                                        feederConstants::Gains::VelocityPID::KS);
    m_feederMotor.SetVoltage(units::volt_t(m_feederVelocityPID.CalculateWithRealTime(RPM, m_feederVelocity, m_timestamp)));
}

void GripperIOSpark::SetOuttakeRPM(const double RPM)
{
    m_timestamp = TimerRBL::GetFPGATimestampInSeconds();
    m_outtakeVelocityPID.SetFeedforward(NSIGN(RPM - m_outtakeVelocity) * 
                                        outtakeConstants::Gains::VelocityPID::KS);
    m_outtakeMotor.SetVoltage(units::volt_t(m_outtakeVelocityPID.CalculateWithRealTime(RPM, m_outtakeVelocity, m_timestamp)));
}