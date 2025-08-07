/*******************************************************************************
 * 
 * File        : TankOdometryTracker.h (v1.1)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Provides odometry tracking for tank drive systems using : 
 *              - ICC model.
 *              - Twist exponential integration.
 *              - Fusion of distance and velocity data.
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

#pragma once 
#include "frc/geometry/Pose2d.h"

class TankOdometryTracker
{
private:

    double *m_pLeftSideVelocity;
    double *m_pRightSideVelocity;

    double m_alpha;
    double m_drivetrainTrackwidth;

    frc::Pose2d m_lastPose;
    double m_lastLeftDistance{0.0};
    double m_lastRightDistance{0.0};

    double m_generalDeltaX{0.0};
    double m_generalDeltaY{0.0};

public:
    /**
     * @brief Constructs a TankOdometryTracker object to track the odometry of a tank drive system.
     * 
     * @param pLeftSideVelocity Pointer to the velocity of the left side of the drivetrain (in meters per second).
     * @param pRightSideVelocity Pointer to the velocity of the right side of the drivetrain (in meters per second).
     * @param trackwidth The distance between the left and right wheels of the drivetrain (in meters). 
     *                   Must be greater than 0.0. If an invalid value is provided, a default value of 0.5 meters is used.
     * @param alpha A double value in the range [0.0, 1.0] representing the blending factor of the alpha-filter.
     *              Values outside this range are ignored.
     * 
     * @warning The trackwidth is a critical parameter for accurate odometry calculations. Ensure it is measured correctly.
     */
    TankOdometryTracker(double* pLeftSideVelocity, double* pRightSideVelocity, double trackwidth, double alpha);
    /**
     * @brief Constructs a TankOdometryTracker object to track the odometry of a tank drive system.
     * 
     * @param pLeftSideVelocity Pointer to the velocity of the left side of the drivetrain (in meters per second).
     * @param pRightSideVelocity Pointer to the velocity of the right side of the drivetrain (in meters per second).
     * @param trackwidth The distance between the left and right wheels of the drivetrain (in meters). 
     *                   Must be greater than 0.0. If an invalid value is provided, a default value of 0.5 meters is used.
     * 
     * @warning The trackwidth is a critical parameter for accurate odometry calculations. Ensure it is measured correctly.
     * 
     * @note A default alpha value of 0.5 is assigned. To change it, you should use "SetAlpha(const double alpha)"
     */
    TankOdometryTracker(double* pLeftSideVelocity, double* pRightSideVelocity, double trackwidth);
    ~TankOdometryTracker() = default;

    /**
     * Resets the odometry tracker to a new pose.
     *
     * This method updates the internal state of the odometry tracker by setting
     * the current pose to the specified new pose and resetting the left and right
     * distance measurements to zero. It is typically used to reinitialize the
     * odometry tracker when the robot's position is known or needs to be corrected.
     *
     * @param newPose The new pose to reset the odometry tracker to, represented
     *                as an frc::Pose2d object.
     */
    void ResetPose2D(const frc::Pose2d newPose);
    /**
     * @brief Sets the alpha blending factor used in sensor fusion for odometry tracking.
     * 
     * The alpha parameter determines the weight given to encoder data versus velocity data
     * in the fusion process:
     * - alpha = 1.0: Use encoder data only.
     * - alpha = 0.0: Use velocity data only.
     * - alpha in (0.0, 1.0): Blend encoder and velocity data proportionally.
     * 
     * @param alpha A double value in the range [0.0, 1.0] representing the blending factor.
     *              Values outside this range are ignored.
     * 
     * @note If the provided alpha value is outside the valid range, the function will not
     *       update the blending factor.
     */
    void SetAlpha(const double alpha);

    /**
     * @brief Retrieves the last recorded pose of the robot.
     * 
     * This function returns the most recently updated pose of the robot
     * as tracked by the odometry system. The pose includes the robot's
     * position and orientation on the field.
     * 
     * @return frc::Pose2d The last recorded pose of the robot.
     */
    frc::Pose2d GetPose();

    /**
     * Updates the robot's pose using the Instantaneous Center of Curvature (ICC) model
     * based on the distances traveled by the left and right sides of the drivetrain.
     *
     * @param leftDistance The total distance traveled by the left side of the drivetrain (in meters).
     * @param rightDistance The total distance traveled by the right side of the drivetrain (in meters).
     * 
     * This method calculates the change in position and orientation of the robot
     * based on the difference in distances traveled by the left and right sides.
     * If the change in orientation (deltaBaseTheta) is negligible, the robot is assumed
     * to have moved in a straight line. Otherwise, the ICC model is used to compute
     * the robot's movement in a circular arc.
     * 
     * @return The updated pose of the robot as an frc::Pose2d object.
     * 
     * @note Good for small turns, but less stable for larger deltaTheta or noisy data
     */
    frc::Pose2d UpdateUsingICCFromDistances(const double leftDistance, const double rightDistance);
    /**
     * Updates the robot's pose using the twist exponential method based on the distances
     * traveled by the drivetrain.
     *
     * @param leftDistance The total distance traveled by the left side of the drivetrain (in meters).
     * @param rightDistance The total distance traveled by the right side of the drivetrain (in meters).
     * @return The updated pose of the robot as an frc::Pose2d object.
     *
     * This method calculates the change in distance and angle based on the differences
     * in the left and right distances since the last update. It then uses these values
     * to compute a twist, which is applied to the last known pose to determine the new pose.
     * 
     * @note This method is mathematically sound and preferred for most applications
     */
    frc::Pose2d UpdateUsingTwistExpFromDistances(const double leftDistance, const double rightDistance);
    /**
     * Updates the robot's pose using the twist exponential method based on the current velocity.
     *
     * This method calculates the robot's new pose by integrating the linear and angular velocities
     * over a given time interval (dt). It uses the twist exponential method to compute the pose
     * transformation.
     *
     * @param dt The time interval over which to integrate the velocities, in seconds. 
     *           Must be positive; defaults to 0.02 seconds if not positive.
     * @return The updated pose of the robot as an frc::Pose2d object.
     *
     * @note Useful for high-rate updates or fallback when encoder readings are unavailable
     */
    frc::Pose2d UpdateUsingTwistExpFromVelocity(double dt);
    /**
     * @brief Updates the robot's pose using a fusion of distance and velocity data with exponential twist integration.
     *
     * This method calculates the robot's new pose based on the distances traveled by the drivetrain,
     * as well as its velocity. It applies an alpha filter to combine the distance-based and 
     * velocity-based twist calculations for smoother pose estimation.
     *
     * @param leftDistance The total distance traveled by the left side of the drivetrain (in meters).
     * @param rightDistance The total distance traveled by the right side of the drivetrain (in meters).
     * @param dt The time interval since the last update (in seconds). If dt is not positive, it defaults to 0.02 seconds.
     * 
     * @return The updated pose of the robot as an frc::Pose2d object.
     *
     * @note Useful for smoothing noise or correcting latency effects in either sensor
     */
    frc::Pose2d UpdateUsingFusionTwistExp(const double leftDistance, const double rightDistance, double dt);
};