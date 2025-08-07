#include "lib/localization/TankOdometryTracker.h"

#include <cmath>
#include "lib/DebugUtils.h"

TankOdometryTracker::TankOdometryTracker(double* pLeftSideVelocity, double* pRightSideVelocity, double trackwidth)
                                        : m_pLeftSideVelocity(pLeftSideVelocity),
                                        m_pRightSideVelocity(pRightSideVelocity),
                                        m_alpha(0.5)
{
    if(trackwidth > 0.0)
    {
        m_drivetrainTrackwidth = trackwidth;
    }
    else
    {
        DEBUG_ASSERT(false, "trackwidth impossible");
        m_drivetrainTrackwidth = 0.5; //default value for protection
    }
}

TankOdometryTracker::TankOdometryTracker(double* pLeftSideVelocity, double* pRightSideVelocity, double trackwidth, double alpha)
                                        : m_pLeftSideVelocity(pLeftSideVelocity),
                                        m_pRightSideVelocity(pRightSideVelocity)
{
    if(trackwidth > 0.0)
    {
        m_drivetrainTrackwidth = trackwidth;
    }
    else
    {
        DEBUG_ASSERT(false, "trackwidth impossible");
        m_drivetrainTrackwidth = 0.5; //default value for protection
    }

    if((alpha >= 0.0) && (alpha <= 1.0))
    {
        m_alpha = alpha;
    }
    else
    {
        DEBUG_ASSERT(false, "invalid range alpha factor");
        m_alpha = 0.5; //default value for protection
    }
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

frc::Pose2d TankOdometryTracker::UpdateUsingICCFromDistances(const double leftDistance, const double rightDistance)
{
    double deltaLeftDistance = leftDistance - m_lastLeftDistance;
    double deltaRightDistance = rightDistance - m_lastRightDistance;

    double deltaBaseDistance = (deltaLeftDistance + deltaRightDistance) /2.0;
    double deltaBaseTheta = (deltaRightDistance - deltaLeftDistance) / m_drivetrainTrackwidth;

    if(NABS(deltaBaseTheta) < 1e-6)
    {
        m_generalDeltaX = deltaBaseDistance * std::cos((double)m_lastPose.Rotation().Radians());
        m_generalDeltaY = deltaBaseDistance * std::sin((double)m_lastPose.Rotation().Radians());
    }
    else
    {   
        // Instantaneous Center of Curvature (ICC) model
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

    m_lastPose = frc::Pose2d{units::length::meter_t(m_generalDeltaX) + m_lastPose.X(), 
                            units::length::meter_t(m_generalDeltaY) + m_lastPose.Y(),
                            frc::Rotation2d{units::radian_t(WRAP_ANGLE_0_TO_2PI(deltaBaseTheta + double(m_lastPose.Rotation().Radians())))}};

    return m_lastPose;
}

frc::Pose2d TankOdometryTracker::UpdateUsingTwistExpFromDistances(const double leftDistance, const double rightDistance)
{
    double deltaLeftDistance = leftDistance - m_lastLeftDistance;
    double deltaRightDistance = rightDistance - m_lastRightDistance;

    units::meter_t deltaBaseDistance = units::meter_t((deltaLeftDistance + deltaRightDistance) /2.0);
    units::radian_t deltaBaseTheta = units::radian_t((deltaRightDistance - deltaLeftDistance) / m_drivetrainTrackwidth);

    m_lastLeftDistance = leftDistance;
    m_lastRightDistance = rightDistance;

    frc::Twist2d twist{deltaBaseDistance, units::meter_t(0.0), deltaBaseTheta};
    m_lastPose = m_lastPose.Exp(twist);

    return m_lastPose;
}

frc::Pose2d TankOdometryTracker::UpdateUsingTwistExpFromVelocity(double dt)
{
    // Ensure dt is positive; fallback to 0.02 seconds (standard dt in FRC) if not.
    DEBUG_ASSERT(dt > 0.0, "dt must be positive");
    dt = (dt > 0.0) ? dt : 0.02;

    double v = (*m_pLeftSideVelocity + *m_pRightSideVelocity) / 2.0;
    double omega = (*m_pRightSideVelocity - *m_pLeftSideVelocity) / m_drivetrainTrackwidth;

    m_lastLeftDistance += *m_pLeftSideVelocity * dt;
    m_lastRightDistance += *m_pRightSideVelocity * dt;
    frc::Twist2d twist{units::meter_t(v * dt), units::meter_t(0.0), units::radian_t(omega * dt)};
    m_lastPose = m_lastPose.Exp(twist);

    return m_lastPose;
}

frc::Pose2d TankOdometryTracker::UpdateUsingFusionTwistExp(const double leftDistance, const double rightDistance, double dt)
{
    // Ensure dt is positive; fallback to 0.02 seconds (standard dt in FRC) if not.
    DEBUG_ASSERT(dt > 0.0, "dt must be positive");
    dt = (dt > 0.0) ? dt : 0.02;

    // Distance-based twist
    double deltaLeftDistance = leftDistance - m_lastLeftDistance;
    double deltaRightDistance = rightDistance - m_lastRightDistance;

    units::meter_t deltaBaseDistance = units::meter_t((deltaLeftDistance + deltaRightDistance) /2.0);
    units::radian_t deltaBaseTheta = units::radian_t((deltaRightDistance - deltaLeftDistance) / m_drivetrainTrackwidth);

    m_lastLeftDistance = leftDistance;
    m_lastRightDistance = rightDistance;

    frc::Twist2d twistDistance{deltaBaseDistance, units::meter_t(0.0), deltaBaseTheta};

    // Velocity-based twist 
    double v = (*m_pLeftSideVelocity + *m_pRightSideVelocity) / 2.0;
    double omega = (*m_pRightSideVelocity - *m_pLeftSideVelocity) / m_drivetrainTrackwidth;

    frc::Twist2d twistVelocity{units::meter_t(v * dt), units::meter_t(0.0), units::radian_t(omega * dt)};

    // Linear interpolation between the two twist estimates
    frc::Twist2d twistFiltred{
    (1.0 - m_alpha) * twistVelocity.dx + m_alpha * twistDistance.dx,
    (1.0 - m_alpha) * twistVelocity.dy + m_alpha * twistDistance.dy,
    (1.0 - m_alpha) * twistVelocity.dtheta + m_alpha * twistDistance.dtheta};
    m_lastPose = m_lastPose.Exp(twistFiltred);

    return m_lastPose;
}

frc::Pose2d TankOdometryTracker::GetPose()
{
    return m_lastPose;
}