#include "subsystems/drivetrain/DrivetrainSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainIO *pIO) 
                    : m_pTankDriveIO(pIO), 
                    m_fxForwardAxis([]() { return 0.0; }),
                    m_fxRotationAxis([]() { return 0.0; }),
                    m_fxSlowDriveButton([]() { return false; }),
                    m_fxHeightFactor([]() { return 0.0; }),
                    m_axisAreActive(false)
{}

DrivetrainSubsystem::DrivetrainSubsystem(DrivetrainIO *pIO, 
                    std::function<double()> fxForwardAxis,
                    std::function<double()> fxRotationAxis,
                    std::function<bool()> fxSlowDriveButton,
                    std::function<double()> fxHeightFactor)
                    : m_pTankDriveIO(pIO), 
                    m_fxForwardAxis(fxForwardAxis),
                    m_fxRotationAxis(fxRotationAxis),
                    m_fxSlowDriveButton(fxSlowDriveButton),
                    m_fxHeightFactor(fxHeightFactor),
                    m_axisAreActive(true)
{}

void DrivetrainSubsystem::SetWantedDrive(const WantedDrive wantedDrive)
{
    m_wantedDrive = wantedDrive;
    //TODO : add reset each times m_wantedDrive is changed
    switch (m_wantedDrive)
    {
    case WantedDrive::ARCADE_DRIVE :
        m_systemDrive = SystemDrive::ARCADE_DRIVE;
        break;
    case WantedDrive::AUTO_PATH_FOLLOWER :
        m_systemDrive = SystemDrive::AUTO_PATH_FOLLOWER;
        break;
    case WantedDrive::REVERSE_DRIVE :
        m_systemDrive = SystemDrive::REVERSE_ARCADE_DRIVE;
        break;
    
    case WantedDrive::STAND_BY :
        break;
    default:
        DEBUG_ASSERT(false, "DrivetrainSubsystem::Periodic: Invalid WantedDrive state");
        break;
    }
}

void DrivetrainSubsystem::ConfigureManualAxis(const std::function<double()> fxForwardAxis,
                                            const std::function<double()> fxRotationAxis,
                                            const std::function<bool()> fxSlowDriveButton,
                                            const std::function<double()> fxHeightFactor)
{
    m_fxForwardAxis = fxForwardAxis;
    m_fxRotationAxis = fxRotationAxis;
    m_fxSlowDriveButton = fxSlowDriveButton;
    m_fxHeightFactor = fxHeightFactor;
    m_axisAreActive = true;
}

void DrivetrainSubsystem::Periodic()
{
    m_pTankDriveIO->UpdateInputs(inputs);

    m_frontLeftMotorDisconnected.Set(!inputs.isFrontLeftMotorConnected);
    m_frontRightMotorDisconnected.Set(!inputs.isFrontRightMotorConnected);
    m_backLeftMotorDisconnected.Set(!inputs.isBackLeftMotorConnected);
    m_backRightMotorDisconnected.Set(!inputs.isBackRightMotorConnected);

    m_frontLeftMotorHot.Set(inputs.frontLeftMotorTemperature > 60.0); //TODO : add const
    m_frontRightMotorHot.Set(inputs.frontRightMotorTemperature > 60.0);
    m_backLeftMotorHot.Set(inputs.backLeftMotorTemperature > 60.0);
    m_backRightMotorHot.Set(inputs.backRightMotorTemperature > 60.0);

    m_frontLeftMotorOverheating.Set(inputs.frontLeftMotorTemperature > 75.0);
    m_frontRightMotorOverheating.Set(inputs.frontRightMotorTemperature > 75.0);
    m_backLeftMotorOverheating.Set(inputs.backLeftMotorTemperature > 75.0);
    m_backRightMotorOverheating.Set(inputs.backRightMotorTemperature > 75.0);


    DEBUG_ASSERT(m_axisAreActive, "DrivetrainSubsystem : Manual Functions aren't assigned");
    double m_forwardAxis = NCLAMP(-1.0, -m_fxForwardAxis(), 1.0);
    double m_rotationAxis = NCLAMP(-1.0, m_fxRotationAxis(), 1.0);

    if (m_fxSlowDriveButton()) {
        m_forwardAxis /= Settings::SLOW_RATE;
        m_rotationAxis /= Settings::SLOW_RATE;
    }

    //Protect from falling
    //TODO : rework this with a proper way and NavX
    double h = (1.0 - m_fxHeightFactor());
    double minMovingV = m_forwardAxis * Settings::MIN_MOVING_V;
    double minMovingW = m_rotationAxis * Settings::MIN_MOVING_W;
    if(m_forwardAxis < 0.0)
    {
        m_forwardAxis = NMAX(m_forwardAxis, -h*h);
    }
    else 
    {
        m_forwardAxis = NMIN(m_forwardAxis, h*h);
    }

    if(m_rotationAxis < 0.0)
    {
        m_rotationAxis = NMAX(m_rotationAxis, -h);
    }
    else 
    {
        m_rotationAxis = NMIN(m_rotationAxis, h);
    }
    m_forwardAxis += minMovingV;
    m_rotationAxis += minMovingW;

    switch (m_systemDrive)
    {
    case SystemDrive::ARCADE_DRIVE:
        m_output = ArcadeDrive(m_forwardAxis, m_rotationAxis);
        break;
    
    case SystemDrive::REVERSE_ARCADE_DRIVE:
        m_output = ArcadeDrive(-m_forwardAxis, m_rotationAxis);
        break;
    
    case SystemDrive::AUTO_PATH_FOLLOWER:
        m_output = {0.0, 0.0}; //TODO: Implement auto path follower
        break;
    default:
        DEBUG_ASSERT(false, "DrivetrainSubsystem::Periodic: Invalid SystemDrive state");
        break;
    }

    //TODO : add log
    m_pTankDriveIO->SetDutyCycle(m_output.first, m_output.second);
}

std::pair<double, double> DrivetrainSubsystem::ArcadeDrive(const double forward, const double rotation)
{
    m_forwardLimitedAxis.Update(forward);
    m_rotationLimitedAxis.Update(rotation);

    m_rotationSigma = NLERP(0.1, 0.45, NABS(m_rotationLimitedAxis.GetCurrentSpeed()));

    double leftWheelOutput = std::sin(m_forwardLimitedAxis.GetCurrentSpeed() * (NF64_PI / 2)) + 
                            std::sin(m_rotationLimitedAxis.GetCurrentSpeed() * (NF64_PI / 2)) * m_rotationSigma;
    
    double rightWheelOutput = std::sin(m_forwardLimitedAxis.GetCurrentSpeed() * (NF64_PI / 2)) - 
                            std::sin(m_rotationLimitedAxis.GetCurrentSpeed() * (NF64_PI / 2)) * m_rotationSigma;
    
    double scaleFactor = 1.0 / NMAX(NABS(leftWheelOutput), NABS(rightWheelOutput));

    leftWheelOutput *= scaleFactor;
    rightWheelOutput *= scaleFactor;

    return {leftWheelOutput, rightWheelOutput};
}