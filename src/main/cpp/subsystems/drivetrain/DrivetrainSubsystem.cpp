#include "subsystems/drivetrain/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainIO *pIO) 
                    : m_pTankDriveIO(pIO), 
                    m_fxForwardAxis([]() { return 0.0; }),
                    m_fxRotationAxis([]() { return 0.0; }),
                    m_fxSlowDriveButton([]() { return false; }),
                    m_fxDriveActionButton([]() { return false; }),
                    m_fxHeightFactor([]() { return 0.0; }),
                    m_axisAreActive(false)
{}

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainIO *pIO, 
                    std::function<double()> fxForwardAxis,
                    std::function<double()> fxRotationAxis,
                    std::function<bool()> fxSlowDriveButton,
                    std::function<bool()> fxDriveActionButton,
                    std::function<double()> fxHeightFactor)
                    : m_pTankDriveIO(pIO), 
                    m_fxForwardAxis(fxForwardAxis),
                    m_fxRotationAxis(fxRotationAxis),
                    m_fxSlowDriveButton(fxSlowDriveButton),
                    m_fxDriveActionButton(fxDriveActionButton),
                    m_fxHeightFactor(fxHeightFactor),
                    m_axisAreActive(true)
{
}
void DrivetrainSubsystem::SetWantedDrive(const DriveMode wantedDrive)
{
    m_wantedDrive = wantedDrive;
}

DrivetrainSubsystem::SystemDrive DrivetrainSubsystem::GetSystemDrive() const
{
    return m_systemDrive;
}

void DrivetrainSubsystem::ConfigureManualControlInputsAxis(const std::function<double()> fxForwardAxis,
                                            const std::function<double()> fxRotationAxis,
                                            const std::function<bool()> fxSlowDriveButton,
                                            const std::function<bool()> fxDriveActionButton,
                                            const std::function<double()> fxHeightFactor)
{
    m_fxForwardAxis = fxForwardAxis;
    m_fxRotationAxis = fxRotationAxis;
    m_fxSlowDriveButton = fxSlowDriveButton;
    m_fxDriveActionButton = fxDriveActionButton;
    m_fxHeightFactor = fxHeightFactor;
    m_axisAreActive = true;
}

void DrivetrainSubsystem::SetAlliance(frc::DriverStation::Alliance alliance)
{
    m_alliance = alliance;
}

void DrivetrainSubsystem::SetDesiredAutoTrajectory(choreo::Trajectory<choreo::DifferentialSample> trajectory)
{
    m_desiredAutoTrajectory = trajectory;
    m_autoTimer.Restart();
}
void DrivetrainSubsystem::ResetOdometryPose(const frc::Pose2d pose)
{
    m_pTankDriveIO->ResetPosition(pose);
}

void DrivetrainSubsystem::Periodic()
{
    m_pTankDriveIO->UpdateInputs(inputs);
    m_logger.Log(inputs);

    m_frontLeftMotorDisconnected.Set(!inputs.isFrontLeftMotorConnected);
    m_frontRightMotorDisconnected.Set(!inputs.isFrontRightMotorConnected);
    m_backLeftMotorDisconnected.Set(!inputs.isBackLeftMotorConnected);
    m_backRightMotorDisconnected.Set(!inputs.isBackRightMotorConnected);

    m_frontLeftMotorHot.Set(inputs.frontLeftMotorTemperature > driveConstants::Motors::HOT_THRESHOLD);
    m_frontRightMotorHot.Set(inputs.frontRightMotorTemperature > driveConstants::Motors::HOT_THRESHOLD);
    m_backLeftMotorHot.Set(inputs.backLeftMotorTemperature > driveConstants::Motors::HOT_THRESHOLD);
    m_backRightMotorHot.Set(inputs.backRightMotorTemperature > driveConstants::Motors::HOT_THRESHOLD);

    m_frontLeftMotorOverheating.Set(inputs.frontLeftMotorTemperature > driveConstants::Motors::OVERHEATING_THRESHOLD);
    m_frontRightMotorOverheating.Set(inputs.frontRightMotorTemperature > driveConstants::Motors::OVERHEATING_THRESHOLD);
    m_backLeftMotorOverheating.Set(inputs.backLeftMotorTemperature > driveConstants::Motors::OVERHEATING_THRESHOLD);
    m_backRightMotorOverheating.Set(inputs.backRightMotorTemperature > driveConstants::Motors::OVERHEATING_THRESHOLD);


    switch (m_wantedDrive) //Handle State transition
    {
    case DriveMode::ARCADE_DRIVE :
        if(m_systemDrive != SystemDrive::ARCADE_DRIVE)
        {
            m_rotationSigma = 0.0;
            m_forwardLimitedAxis.Reset();
            m_rotationLimitedAxis.Reset();
            m_forwardLimitedAxis.SetRateLimit(driveConstants::ArcadeDrive::TIME_TO_REACH_FULL_FORWARD,
                                            driveConstants::ArcadeDrive::TIME_TO_STOP_FORWARD);
            m_rotationLimitedAxis.SetRateLimit(driveConstants::ArcadeDrive::TIME_TO_REACH_FULL_ROTATION,
                                            driveConstants::ArcadeDrive::TIME_TO_STOP_ROTATION);
            m_systemDrive = SystemDrive::ARCADE_DRIVE;
        }
        break;
    
    case DriveMode::CURVE_DRIVE :
        if(m_systemDrive != SystemDrive::CURVE_DRIVE)
        {
            m_forwardLimitedAxis.Reset();
            m_forwardLimitedAxis.SetRateLimit(driveConstants::CurveDrive::TIME_TO_REACH_FULL_FORWARD,
                                            driveConstants::CurveDrive::TIME_TO_STOP_FORWARD);
            m_previousRotation = 0.0;
            m_negInertiaAccumulator = 0.0;
            m_quickStopAccumulator = 0.0;
            m_systemDrive = SystemDrive::CURVE_DRIVE;
        }
        break;

    case DriveMode::AUTO_PATH_FOLLOWER :
        if(m_systemDrive != SystemDrive::AUTO_PATH_FOLLOWER)
        {
            m_systemDrive = SystemDrive::AUTO_PATH_FOLLOWER;
            m_autoTimer.Restart();
            m_autoSampleToBeApplied = m_desiredAutoTrajectory.SampleAt(units::time::second_t(m_autoTimer.GetElapsedTimeSeconds()), IS_RED_ALLIANCE(m_alliance));
        }
        break;

    case DriveMode::DISABLE :
        if(m_systemDrive != SystemDrive::DISABLE)
        {
            m_systemDrive = SystemDrive::DISABLE;
        }
        break;

    default:
        DEBUG_ASSERT(false, "DrivetrainSubsystem::Periodic: Invalid WantedDrive state");
        break;
    }

    switch (m_systemDrive) //Calculate output from SystemDrive
    {
    case SystemDrive::AUTO_PATH_FOLLOWER:
        DEBUG_ASSERT(false, "work in progress..");
        m_autoSampleToBeApplied = m_desiredAutoTrajectory.SampleAt((units::time::second_t)m_autoTimer.GetElapsedTimeSeconds(), IS_RED_ALLIANCE(m_alliance));
        m_output = FollowPath();
        break;

    case SystemDrive::ARCADE_DRIVE :
        m_output = ArcadeDrive(GetSafetyPercentages());
        break;
    
    case SystemDrive::CURVE_DRIVE :
        m_output = CurveDrive(GetSafetyPercentages(), m_fxDriveActionButton());
        break;
    
    case SystemDrive::DISABLE :
        m_output = restSpeeds;
        break;
    
    default:
        DEBUG_ASSERT(false, "tu n'es pas censé lire ça. Sinon bravo, tu as activé une assert !");
        break;
    }

    //TODO : NavX protection

    frc::SmartDashboard::PutNumber("Drivetrain/SystemDrive", (int)m_systemDrive);
    frc::SmartDashboard::PutNumber("Drivetrain/WantedDrive", (int)m_wantedDrive);

    m_pTankDriveIO->SetChassisSpeed(m_output);
}

frc::ChassisSpeeds DrivetrainSubsystem::ArcadeDrive(const std::pair<double, double> percentage)
{ 
    frc::ChassisSpeeds output;

    if(m_fxDriveActionButton())
    {
        m_forwardLimitedAxis.Update(-percentage.first);
    }
    else
    {
        m_forwardLimitedAxis.Update(percentage.first);
    }
    m_rotationLimitedAxis.Update(percentage.second);

    m_rotationSigma = NLERP(driveConstants::ArcadeDrive::MIN_ROTATION_SIGMA,
                            driveConstants::ArcadeDrive::MAX_ROTATION_SIGMA, 
                            NABS(m_rotationLimitedAxis.GetCurrentSpeed()));

    output.vx = (units::velocity::meters_per_second_t)
                std::sin(m_forwardLimitedAxis.GetCurrentSpeed() * (NF64_PI_2)) * 
                driveConstants::Specifications::MAX_LINEAR_SPEED;

    output.vy = (units::velocity::meters_per_second_t)0.0; // Ce n'est pas des swerves donc pas de Vy pour cette fois, dsl

    output.omega = (units::angular_velocity::radians_per_second_t)
                    std::sin(m_rotationLimitedAxis.GetCurrentSpeed() * (NF64_PI_2)) * m_rotationSigma *
                    driveConstants::Specifications::MAX_ROTATION_SPEED;
    
    return output;
}

frc::ChassisSpeeds DrivetrainSubsystem::CurveDrive(const std::pair<double, double> percentage, const bool quickTurnEnabled)
{
    frc::ChassisSpeeds output;
    bool QuickTurn = quickTurnEnabled;

    // Deadband → activate quickTurn
    if(NABS(percentage.first) < driveConstants::Settings::DEADBAND)
    {
        QuickTurn = true;
    }

    m_forwardLimitedAxis.Update(percentage.first);

    //R1 = sin(PI/2 * SIN_CURVE_I * R) / sin(PI/2 * SIN_CURVE_I)
    //Rnlr = sin(PI/2 * SIN_CURVE_I * R1) / sin(PI/2 * SIN_CURVE_I)
    double nonLinearRotation = std::sin((std::sin((NF64_PI_2) * driveConstants::CurveDrive::SINUSOIDAL_CURVATURE_INTENSITY * percentage.second)
                                / driveConstants::CurveDrive::DENOMINATOR) * (NF64_PI_2) * driveConstants::CurveDrive::SINUSOIDAL_CURVATURE_INTENSITY) 
                                / driveConstants::CurveDrive::DENOMINATOR;

    
    if(QuickTurn)
    {
        output.omega = (units::angular_velocity::radians_per_second_t)nonLinearRotation * NABS(nonLinearRotation) *
                        driveConstants::Specifications::MAX_ROTATION_SPEED;
        m_quickStopAccumulator = (1 - driveConstants::CurveDrive::QUICK_STOP_ALPHA) * m_quickStopAccumulator +
                                driveConstants::CurveDrive::QUICK_STOP_ALPHA * 
                                std::clamp(nonLinearRotation * NABS(nonLinearRotation), -1.0, 1.0) * 2.0;
    }
    else
    {
        double negInertia = (percentage.second - m_previousRotation) * driveConstants::CurveDrive::NEG_INERTIA_SCALAR;
        m_negInertiaAccumulator += negInertia;

        double angularPower = std::abs(percentage.first) *
                              (nonLinearRotation + m_negInertiaAccumulator) *
                               driveConstants::CurveDrive::TURN_SENSITIVITY -
                              m_quickStopAccumulator;

        if (m_negInertiaAccumulator > 1.0)       m_negInertiaAccumulator -= 1.0;
        else if (m_negInertiaAccumulator < -1.0) m_negInertiaAccumulator += 1.0;
        else                                     m_negInertiaAccumulator = 0.0;

        if (m_quickStopAccumulator > 1.0)       m_quickStopAccumulator -= 1.0;
        else if (m_quickStopAccumulator < -1.0) m_quickStopAccumulator += 1.0;
        else                                    m_quickStopAccumulator = 0.0;
        
        output.vx = (units::velocity::meters_per_second_t)m_forwardLimitedAxis.GetCurrentSpeed() *
                    driveConstants::Specifications::MAX_LINEAR_SPEED;
        
        output.vy = (units::velocity::meters_per_second_t)0.0; // Ce n'est pas des swerves donc pas de Vy pour cette fois, dsl

        output.omega = (units::angular_velocity::radians_per_second_t)angularPower * 
                        driveConstants::Specifications::MAX_ROTATION_SPEED;
    }

    m_previousRotation = percentage.second;
    return output;
}

frc::ChassisSpeeds DrivetrainSubsystem::FollowPath() {
    frc::ChassisSpeeds output{};

    // Verify if there is a sample to follow
    if (!m_autoSampleToBeApplied.has_value()) {
        return output; // Nothing to do
    }

    // Current target sample from the trajectory
    const choreo::DifferentialSample& currentSample = m_autoSampleToBeApplied.value();
    const frc::ChassisSpeeds targetSpeeds = currentSample.GetChassisSpeeds();
    const frc::Pose2d targetPose = currentSample.GetPose();

    // Current robot pose
    const frc::Pose2d currentPose = inputs.robotPosition;
    const double robotTheta = currentPose.Rotation().Radians().to<double>();

    // --- Step 1 : Calcul of position error in field coordinates ---
    const double dx = targetPose.X().to<double>() - currentPose.X().to<double>();
    const double dy = targetPose.Y().to<double>() - currentPose.Y().to<double>();

    // --- Step 2 : Projection of the position error in robot coordinates ---
    // Rotation inverse de -θ
    const double cosTheta = std::cos(robotTheta);
    const double sinTheta = std::sin(robotTheta);

    // Error in robot coordinates
    const double errorX =  cosTheta * dx + sinTheta * dy;  // error forward/backward
    // const double errorY = -sinTheta * dx + cosTheta * dy;  // error left/right

    // --- Step 3 : Orientation error ---
    double angleError = (targetPose.Rotation() - currentPose.Rotation()).Radians().to<double>();
    // Wrapp in [-π, π]
    while (angleError > M_PI)  angleError -= 2.0 * M_PI;
    while (angleError < -M_PI) angleError += 2.0 * M_PI;

    // --- Step 4 : PID corrections ---
    // linear PID correcting the distance error in X
    const double linearCorrection = m_pidAutoX.Calculate(errorX, 0.0);

    // angular PID correcting the angle error
    const double angularCorrection = m_pidAutoTheta.Calculate(angleError, 0.0);

    // --- Step 5 : Apply corrections to target speeds ---
    output.vx = targetSpeeds.vx + units::meters_per_second_t(linearCorrection);
    output.omega = targetSpeeds.omega + units::radians_per_second_t(angularCorrection);

    return output;
}

std::pair<double, double> DrivetrainSubsystem::GetSafetyPercentages()
{
    std::pair<double, double> output;
    DEBUG_ASSERT(m_axisAreActive, "DrivetrainSubsystem : Manual Functions aren't assigned");

    double fwdPercentage = NCLAMP(-1.0, m_fxForwardAxis(), 1.0);
    double rotationPercentage = NCLAMP(-1.0, m_fxRotationAxis(), 1.0);

    if (m_fxSlowDriveButton()) 
    {
        fwdPercentage /= driveConstants::Settings::SLOW_RATE;
        rotationPercentage /= driveConstants::Settings::SLOW_RATE;
    }

    double h = (1.0 - m_fxHeightFactor());
    double minMovingFwd = fwdPercentage * driveConstants::StabilityGuard::MIN_MOVING_FORWARD;
    double minTurning = rotationPercentage * driveConstants::StabilityGuard::MIN_TURNING;

    if(fwdPercentage < 0.0)
    {
        fwdPercentage = NMAX(fwdPercentage, -h*h);
    }
    else 
    {
        fwdPercentage = NMIN(fwdPercentage, h*h);
    }

    if(rotationPercentage < 0.0)
    {
        rotationPercentage = NMAX(rotationPercentage, -h);
    }
    else 
    {
        rotationPercentage = NMIN(rotationPercentage, h);
    }
    output.first = fwdPercentage + minMovingFwd;
    output.second = rotationPercentage + minTurning;

    return output;
}