#include "subsystems/drivetrain/DrivetrainIOFlex.h"

#include "frc/smartdashboard/SmartDashboard.h"

DrivetrainIOFlex::DrivetrainIOFlex()
{
    // Set the back left motor configs
    m_motorBackLeftConfig.SetIdleMode(driveConstants::Motors::IDLE_MODE)
        .Inverted(driveConstants::Motors::LEFT_MOTOR_INVERTED)
        .SmartCurrentLimit(driveConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::Motors::RAMP_RATE)
        .VoltageCompensation(driveConstants::Motors::VOLTAGE_COMPENSATION);

    // Set the back right motor configs
    m_motorBackRightConfig.SetIdleMode(driveConstants::Motors::IDLE_MODE)
        .Inverted(driveConstants::Motors::RIGHT_MOTORS_INVERTED)
        .SmartCurrentLimit(driveConstants::Motors::CURRENT_LIMIT)
        .ClosedLoopRampRate(driveConstants::Motors::RAMP_RATE)
        .VoltageCompensation(driveConstants::Motors::VOLTAGE_COMPENSATION);

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

    m_encoderLeft.SetDistancePerPulse(driveConstants::Encoder::DISTANCE_PER_PULSE);
    m_encoderRight.SetDistancePerPulse(driveConstants::Encoder::DISTANCE_PER_PULSE);
    m_encoderLeft.Reset();
    m_encoderRight.Reset();
}

void DrivetrainIOFlex::UpdateInputs(DrivetrainIOInputs& inputs)
{
    inputs.isBackLeftMotorConnected = (m_motorBackLeft.GetBusVoltage() !=0.0) && !m_motorBackLeft.GetFaults().can;
    inputs.isBackRightMotorConnected = (m_motorBackRight.GetBusVoltage() != 0.0) && !m_motorBackRight.GetFaults().can;
    inputs.isFrontLeftMotorConnected = (m_motorFrontLeft.GetBusVoltage() != 0.0) && !m_motorFrontLeft.GetFaults().can;
    inputs.isFrontRightMotorConnected = (m_motorFrontRight.GetBusVoltage() != 0.0) && !m_motorFrontRight.GetFaults().can;

    inputs.backLeftMotorAppliedVoltage = m_motorBackLeft.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION;
    inputs.backRightMotorAppliedVoltage = m_motorBackRight.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION;
    inputs.frontLeftMotorAppliedVoltage = m_motorFrontLeft.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION;
    inputs.frontRightMotorAppliedVoltage = m_motorFrontRight.GetAppliedOutput() * driveConstants::Motors::VOLTAGE_COMPENSATION;

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

    inputs.leftSideTraveledDistance = m_encoderLeft.GetDistance();
    inputs.leftSideVelocity = m_encoderLeft.GetRate();
    inputs.rightSideTraveledDistance = m_encoderRight.GetDistance();
    inputs.rightSideVelocity = m_encoderRight.GetRate();

    m_realLeftSideSpeed = inputs.leftSideVelocity;
    m_realRightSideSpeed = inputs.rightSideVelocity;

    inputs.robotPosition = m_odometry.UpdateUsingFusionTwistExp(inputs.leftSideTraveledDistance, inputs.rightSideTraveledDistance, TIME_PER_CYCLE);
    frc::SmartDashboard::PutBoolean("Drivetrain/LeftSide/Front/Connection", inputs.isFrontLeftMotorConnected);
    frc::SmartDashboard::PutBoolean("Drivetrain/RightSide/Back/Connection", inputs.isBackRightMotorConnected);
    frc::SmartDashboard::PutBoolean("Drivetrain/RightSide/Front/Connection", inputs.isFrontRightMotorConnected);
    frc::SmartDashboard::PutBoolean("Drivetrain/LeftSide/Back/Connection", inputs.isBackLeftMotorConnected);
}

void DrivetrainIOFlex::SetVoltage(const double leftSideVoltage, const double rightSideVoltage)
{
    DEBUG_ASSERT((leftSideVoltage <= driveConstants::Motors::VOLTAGE_COMPENSATION) 
        && (leftSideVoltage >= -driveConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Drivetrain left side Voltage out of range");

    DEBUG_ASSERT((rightSideVoltage <= driveConstants::Motors::VOLTAGE_COMPENSATION) 
        && (rightSideVoltage >= -driveConstants::Motors::VOLTAGE_COMPENSATION) 
        ,"Drivetrain right side Voltage out of range");
    
    m_motorBackLeft.SetVoltage(units::volt_t(leftSideVoltage));
    m_motorBackRight.SetVoltage(units::volt_t(rightSideVoltage));
}

void DrivetrainIOFlex::SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle)
{
    DEBUG_ASSERT((leftSideDutyCycle <= 1.0) && (leftSideDutyCycle >= -1.0) 
            ,"Drivetrain left side Duty Cycle out of range");

    DEBUG_ASSERT((rightSideDutyCycle <= 1.0) && (rightSideDutyCycle >= -1.0) 
            ,"Drivetrain right side Duty Cycle out of range");
    
    m_motorBackLeft.Set(leftSideDutyCycle);
    m_motorBackRight.Set(rightSideDutyCycle);
}

void DrivetrainIOFlex::SetChassisSpeed(const frc::ChassisSpeeds &speeds)
{
    DEBUG_ASSERT(speeds.vy() == 0.0, "Are You stupid ? Did you know that a tank can't move on the Y axis ?");

    //Differential :
    // Vr = Vf - Rb * omega
    // Vl = Vf + Rb * omega
    double rightSideSpeed = speeds.vx() - driveConstants::Specifications::BASE_TRACK_RADIUS * speeds.omega();
    double leftSideSpeed = speeds.vx() + driveConstants::Specifications::BASE_TRACK_RADIUS * speeds.omega();
    double highestSpeedSide = NMAX(NABS(rightSideSpeed), NABS(leftSideSpeed));
    if(highestSpeedSide > driveConstants::Specifications::MAX_LINEAR_SPEED)
    {
        double scaleFactor = driveConstants::Specifications::MAX_LINEAR_SPEED / highestSpeedSide;
        rightSideSpeed *= scaleFactor;
        leftSideSpeed *= scaleFactor;
    }

    double rightOutput = rightSideSpeed * driveConstants::Specifications::LINEAR_TO_MOTOR_SPEED_FACTOR;
    double leftOutput = leftSideSpeed * driveConstants::Specifications::LINEAR_TO_MOTOR_SPEED_FACTOR;


    //TODO : add SparkPID
    //HACK : speed to voltage
    m_motorBackLeft.SetVoltage(units::volt_t(leftOutput / driveConstants::Specifications::MOTOR_FREE_SPEED 
                                            * driveConstants::Motors::VOLTAGE_COMPENSATION));
    m_motorBackRight.SetVoltage(units::volt_t(rightOutput / driveConstants::Specifications::MOTOR_FREE_SPEED 
                                            * driveConstants::Motors::VOLTAGE_COMPENSATION));
}

void DrivetrainIOFlex::ResetPosition(const frc::Pose2d position)
{
    m_encoderLeft.Reset();
    m_encoderRight.Reset();

    m_odometry.ResetPose2D(position);
}