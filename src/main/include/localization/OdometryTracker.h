#pragma once 
//WORK IN PROGRESS
#include "frc/geometry/Pose2d.h"
#include "subsystems/drivetrain/DrivetrainSubsystem.h"

class TankOdometryTracker
{
private:
    frc::Pose2d m_lastPose;
    double m_alpha{0.5};
    double m_lastLeftDistance{0.0};
    double m_lastRightDistance{0.0};

    double m_generalDeltaX{0.0};
    double m_generalDeltaY{0.0};

    double *m_pLeftSideVelocity;
    double *m_pRightSideVelocity;

public:
    TankOdometryTracker(double *pLeftSideVelocity, double *pRightSideVelocity, double alpha);
    TankOdometryTracker(double *pLeftSideVelocity, double *pRightSideVelocity);
    ~TankOdometryTracker() = default;

    void ResetPose2D(const frc::Pose2d newPose);
    void SetAlpha(const double alpha);
    frc::Pose2d GetPose();

    frc::Pose2d UpdateOdometryFromDistances(const double leftDistance, const double rightDistance);
    frc::Pose2d UpdateOdometryFromVelocity(const double dt);

    frc::Pose2d UpdateFiltredOdometry(const double leftDistance, const double rightDistance, const double dt);
};