#include "localization/OdometryTracker.h"

#include <cmath>
#include "lib/DebugUtils.h"

TankOdometryTracker::TankOdometryTracker(double *pLeftSideVelocity, double *pRightSideVelocity)
                                        : m_pLeftSideVelocity(pLeftSideVelocity),
                                        m_pRightSideVelocity(pRightSideVelocity)
{
}

TankOdometryTracker::TankOdometryTracker(double *pLeftSideVelocity, double *pRightSideVelocity, double alpha)
                                        : m_pLeftSideVelocity(pLeftSideVelocity),
                                        m_pRightSideVelocity(pRightSideVelocity),
                                        m_alpha(alpha)
{
}


void TankOdometryTracker::ResetPose2D(const frc::Pose2d newPose)
{
    m_lastPose = newPose;
    m_lastLeftDistance = 0.0;
    m_lastRightDistance = 0.0;
}

void TankOdometryTracker::SetAlpha(const double alpha)
{
    DEBUG_ASSERT((alpha >= 0.0) && (alpha <=1.0),"alpha must be within the [0.0, 1.0] range");
    if((alpha >= 0.0) && (alpha <=1.0))
    {
        m_alpha = alpha;
    }
}

frc::Pose2d TankOdometryTracker::UpdateOdometryFromDistances(const double leftDistance, const double rightDistance)
{
    double deltaLeftDistance = leftDistance - m_lastLeftDistance;
    double deltaRightDistance = rightDistance - m_lastRightDistance;

    double deltaBaseDistance = (deltaLeftDistance + deltaRightDistance) /2.0;
    double deltaBaseTheta = (deltaRightDistance - deltaLeftDistance) / driveConstants::Specifications::TRACKWIDTH;

    if(NABS(deltaBaseTheta) < 1e-6)
    {
        m_generalDeltaX = deltaBaseDistance * std::cos((double)m_lastPose.Rotation().Radians());
        m_generalDeltaY = deltaBaseDistance * std::sin((double)m_lastPose.Rotation().Radians());
    }
    else
    {   
        //instantaneous center of curvature radius
        double IccRadius = deltaBaseDistance / deltaBaseTheta;

        double dx = IccRadius * std::sin(deltaBaseTheta);
        double dy = IccRadius * (1 - std::cos(deltaBaseTheta));

        m_generalDeltaX = std::cos((double)m_lastPose.Rotation().Radians()) * dx 
                        - std::sin((double)m_lastPose.Rotation().Radians()) * dy;
        
        m_generalDeltaY = std::sin((double)m_lastPose.Rotation().Radians()) * dx 
                        - std::cos((double)m_lastPose.Rotation().Radians()) * dy;
    }

    m_lastLeftDistance = leftDistance;
    m_lastRightDistance = rightDistance;

    m_lastPose = frc::Pose2d{(units::length::meter_t)m_generalDeltaX + m_lastPose.X(), 
                            (units::length::meter_t)m_generalDeltaY + m_lastPose.Y(),
                            frc::Rotation2d{units::radian_t(WRAP_ANGLE_0_TO_2PI(deltaBaseTheta + double(m_lastPose.Rotation().Radians())))}};

    return m_lastPose;
}

//IFBUG : toggle velocity to distances : lastDistances ?
frc::Pose2d TankOdometryTracker::UpdateOdometryFromVelocity(const double dt)
{
    DEBUG_ASSERT(dt >0.0, "dt must be positive");
    double v = (*m_pLeftSideVelocity + *m_pRightSideVelocity) / 2.0;
    double omega = (*m_pRightSideVelocity - *m_pLeftSideVelocity) / driveConstants::Specifications::TRACKWIDTH;

    frc::Twist2d twist{(units::meter_t)v * dt, (units::meter_t)0_m, (units::radian_t)omega * dt};
    m_lastPose = m_lastPose.Exp(twist);

    return m_lastPose;
}

frc::Pose2d TankOdometryTracker::UpdateFiltredOdometry(const double leftDistance, const double rightDistance, const double dt)
{
    //TODO : differente ref : Exp and Geom. How does it affect the pose ? Move filter before ?
    DEBUG_ASSERT(dt >0.0, "dt must be positive");
    //distances : 
    double deltaLeftDistance = leftDistance - m_lastLeftDistance;
    double deltaRightDistance = rightDistance - m_lastRightDistance;

    double deltaBaseDistance = (deltaLeftDistance + deltaRightDistance) /2.0;
    double deltaBaseTheta = (deltaRightDistance - deltaLeftDistance) / driveConstants::Specifications::TRACKWIDTH;

    if(NABS(deltaBaseTheta) < 1e-6)
    {
        m_generalDeltaX = deltaBaseDistance * std::cos((double)m_lastPose.Rotation().Radians());
        m_generalDeltaY = deltaBaseDistance * std::sin((double)m_lastPose.Rotation().Radians());
    }
    else
    {   
        //instantaneous center of curvature radius
        double IccRadius = deltaBaseDistance / deltaBaseTheta;

        double dx = IccRadius * std::sin(deltaBaseTheta);
        double dy = IccRadius * (1 - std::cos(deltaBaseTheta));

        m_generalDeltaX = std::cos((double)m_lastPose.Rotation().Radians()) * dx 
                        - std::sin((double)m_lastPose.Rotation().Radians()) * dy;
        
        m_generalDeltaY = std::sin((double)m_lastPose.Rotation().Radians()) * dx 
                        - std::cos((double)m_lastPose.Rotation().Radians()) * dy;
    }

    m_lastLeftDistance = leftDistance;
    m_lastRightDistance = rightDistance;

    //Velocity :
    double v = (*m_pLeftSideVelocity + *m_pRightSideVelocity) / 2.0;
    double omega = (*m_pRightSideVelocity - *m_pLeftSideVelocity) / driveConstants::Specifications::TRACKWIDTH;

    frc::Twist2d twist{(units::meter_t)v * dt, (units::meter_t)0_m, (units::radian_t)omega * dt};

    //Filtred :
    double m_filtredX = (1.0 - m_alpha) * (double)m_lastPose.Exp(twist).X() + m_alpha * (m_generalDeltaX + double(m_lastPose.X()));
    double m_filtredY = (1.0 - m_alpha) * (double)m_lastPose.Exp(twist).Y() + m_alpha * (m_generalDeltaY + double(m_lastPose.Y()));
    double m_filtredTheta = WRAP_ANGLE_0_TO_2PI((1.0 - m_alpha) * WRAP_ANGLE_0_TO_2PI((double)m_lastPose.Exp(twist).Rotation().Radians()) 
                            + m_alpha * WRAP_ANGLE_0_TO_2PI(deltaBaseTheta + double(m_lastPose.Rotation().Radians())));

    m_lastPose = frc::Pose2d{(units::length::meter_t)m_filtredX,
                            (units::length::meter_t)m_filtredY,
                            units::radian_t(m_filtredTheta)};
    return m_lastPose;
}


frc::Pose2d TankOdometryTracker::GetPose()
{
    return m_lastPose;
}