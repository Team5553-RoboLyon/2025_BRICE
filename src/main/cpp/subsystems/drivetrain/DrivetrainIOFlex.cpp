#include "subsystems/drivetrain/DrivetrainIOFlex.h"

#include "frc/smartdashboard/SmartDashboard.h"


DrivetrainIOFlex::DrivetrainIOFlex()
{
    // Set the back left motor configs
    m_motorBackLeftConfig.SetIdleMode(driveConstants::LeftGearbox::Motor::MOTOR_IDLE_MODE)
        .Inverted(driveConstants::LeftGearbox::Motor::MOTOR_INVERTED)
        .SmartCurrentLimit(driveConstants::LeftGearbox::Motor::MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::LeftGearbox::Motor::MOTOR_RAMP)
        .VoltageCompensation(driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);

    // Set the back right motor configs
    m_motorBackRightConfig.SetIdleMode(driveConstants::RightGearbox::Motor::MOTOR_IDLE_MODE)
        .Inverted(driveConstants::RightGearbox::Motor::MOTOR_INVERTED)
        .SmartCurrentLimit(driveConstants::RightGearbox::Motor::MOTOR_CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::RightGearbox::Motor::MOTOR_RAMP)
        .VoltageCompensation(driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION);

    m_motorFrontLeftConfig.Apply(m_motorBackLeftConfig).Follow(m_motorBackLeft);
    m_motorFrontRightConfig.Apply(m_motorBackRightConfig).Follow(m_motorBackRight);
    
    m_motorBackLeft.Configure(m_motorBackLeftConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_motorBackRight.Configure(m_motorBackRightConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_motorFrontLeft.Configure(m_motorFrontLeftConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    m_motorFrontRight.Configure(m_motorFrontRightConfig, 
                            rev::spark::SparkBase::ResetMode::kResetSafeParameters, 
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_motorBackLeft.ClearFaults();
    m_motorBackRight.ClearFaults();
    m_motorFrontLeft.ClearFaults(); 
    m_motorFrontRight.ClearFaults();

    m_encoderLeft.SetDistancePerPulse(driveConstants::LeftGearbox::Encoder::DISTANCE_PER_PULSE);
    m_encoderRight.SetDistancePerPulse(driveConstants::RightGearbox::Encoder::DISTANCE_PER_PULSE);
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
}

void DrivetrainIOFlex::UpdateInputs(DrivetrainIOInputs& inputs)
{
    inputs.isBackLeftMotorConnected = (m_motorBackLeft.GetBusVoltage() !=0.0) && !m_motorBackLeft.GetFaults().can;
    inputs.isBackRightMotorConnected = (m_motorBackRight.GetBusVoltage() != 0.0) && !m_motorBackRight.GetFaults().can;
    inputs.isFrontLeftMotorConnected = (m_motorFrontLeft.GetBusVoltage() != 0.0) && !m_motorFrontLeft.GetFaults().can;
    inputs.isFrontRightMotorConnected = (m_motorFrontRight.GetBusVoltage() != 0.0) && !m_motorFrontRight.GetFaults().can;

    inputs.backLeftMotorAppliedVoltage = m_motorBackLeft.GetAppliedOutput() * driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION;
    inputs.backRightMotorAppliedVoltage = m_motorBackRight.GetAppliedOutput() * driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION;
    inputs.frontLeftMotorAppliedVoltage = m_motorFrontLeft.GetAppliedOutput() * driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION;
    inputs.frontRightMotorAppliedVoltage = m_motorFrontRight.GetAppliedOutput() * driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION;

    inputs.backLeftMotorBusVoltage = m_motorBackLeft.GetBusVoltage();
    inputs.backRightMotorBusVoltage = m_motorBackRight.GetBusVoltage();
    inputs.frontLeftMotorBusVoltage = m_motorFrontLeft.GetBusVoltage();
    inputs.frontRightMotorBusVoltage = m_motorFrontRight.GetBusVoltage();

    inputs.backLeftMotorCurrent = m_motorBackLeft.GetOutputCurrent();
    inputs.backRightMotorCurrent = m_motorBackRight.GetOutputCurrent();
    inputs.frontLeftMotorCurrent = m_motorFrontLeft.GetOutputCurrent();
    inputs.frontRightMotorCurrent = m_motorFrontRight.GetOutputCurrent();

    inputs.backLeftMotorTemperature = m_motorBackLeft.GetMotorTemperature();
    inputs.backRightMotorTemperature = m_motorBackRight.GetMotorTemperature();
    inputs.frontLeftMotorTemperature = m_motorFrontLeft.GetMotorTemperature();
    inputs.frontRightMotorTemperature = m_motorFrontRight.GetMotorTemperature();

    inputs.leftDistance = m_encoderLeft.GetDistance();
    inputs.leftVelocity = m_encoderLeft.GetRate();
    inputs.rightDistance = m_encoderRight.GetDistance();
    inputs.rightVelocity = m_encoderRight.GetRate();


    //only while waiting for AdScope (very bad performance)
    frc::SmartDashboard::PutBoolean("TDlf.Connection", inputs.isFrontLeftMotorConnected);
    // frc::SmartDashboard::PutNumber("TDlf.Voltage", inputs.frontLeftMotorAppliedVoltage);
    // frc::SmartDashboard::PutNumber("TDlf.BusVolt", inputs.frontLeftMotorBusVoltage);
    // frc::SmartDashboard::PutNumber("TDlf.Current", inputs.frontLeftMotorCurrent);
    // frc::SmartDashboard::PutNumber("TDlf.Temperature", inputs.frontLeftMotorTemperature);

    frc::SmartDashboard::PutBoolean("TDrb.Connection", inputs.isBackRightMotorConnected);
    // frc::SmartDashboard::PutNumber("TDrb.Voltage", inputs.backRightMotorAppliedVoltage);
    // frc::SmartDashboard::PutNumber("TDrb.BusVolt", inputs.backRightMotorBusVoltage);
    // frc::SmartDashboard::PutNumber("TDrb.Current", inputs.backRightMotorCurrent);
    // frc::SmartDashboard::PutNumber("TDrb.Temperature", inputs.backRightMotorTemperature);

    frc::SmartDashboard::PutBoolean("TDrf.Connection", inputs.isFrontRightMotorConnected);
    // frc::SmartDashboard::PutNumber("TDrf.Voltage", inputs.frontRightMotorAppliedVoltage);
    // frc::SmartDashboard::PutNumber("TDrf.BusVolt", inputs.frontRightMotorBusVoltage);
    // frc::SmartDashboard::PutNumber("TDrf.Current", inputs.frontRightMotorCurrent);
    // frc::SmartDashboard::PutNumber("TDrf.Temperature", inputs.frontRightMotorTemperature);

    frc::SmartDashboard::PutBoolean("TDlb.Connection", inputs.isBackLeftMotorConnected);
    // frc::SmartDashboard::PutNumber("TDlb.Voltage", inputs.backLeftMotorAppliedVoltage);
    // frc::SmartDashboard::PutNumber("TDlb.BusVolt", inputs.backLeftMotorBusVoltage);
    // frc::SmartDashboard::PutNumber("TDlb.Current", inputs.backLeftMotorCurrent);
    // frc::SmartDashboard::PutNumber("TDlb.Temperature", inputs.backLeftMotorTemperature);

    frc::SmartDashboard::PutNumber("TD.LeftDistance", inputs.leftDistance);
    frc::SmartDashboard::PutNumber("TD.LeftVelocity", inputs.leftVelocity);
    frc::SmartDashboard::PutNumber("TD.RightDistance", inputs.rightDistance);
    frc::SmartDashboard::PutNumber("TD.RightVelocity", inputs.rightVelocity);
}

void DrivetrainIOFlex::SetVoltage(double leftSideVoltage, double rightSideVoltage)
{
    DEBUG_ASSERT((leftSideVoltage <= driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION) 
        && (leftSideVoltage >= -driveConstants::LeftGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION) 
        ,"Drivetrain left side Voltage out of range");

    DEBUG_ASSERT((rightSideVoltage <= driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION) 
        && (rightSideVoltage >= -driveConstants::RightGearbox::Motor::MOTOR_VOLTAGE_COMPENSATION) 
        ,"Drivetrain right side Voltage out of range");
    
    m_motorBackLeft.SetVoltage(units::volt_t(leftSideVoltage));
    m_motorBackRight.SetVoltage(units::volt_t(rightSideVoltage));
}

void DrivetrainIOFlex::SetDutyCycle(double leftSideDutyCycle, double rightSideDutyCycle)
{
    DEBUG_ASSERT((leftSideDutyCycle <= 1.0) && (leftSideDutyCycle >= -1.0) 
            ,"Drivetrain left side Duty Cycle out of range");

    DEBUG_ASSERT((rightSideDutyCycle <= 1.0) && (rightSideDutyCycle >= -1.0) 
            ,"Drivetrain right side Duty Cycle out of range");
    
    m_motorBackLeft.Set(leftSideDutyCycle);
    m_motorBackRight.Set(rightSideDutyCycle);
}

void DrivetrainIOFlex::ResetPosition()
{
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
}