#pragma once
#include "Constants.h"
#include "lib/DebugUtils.h"
#include <cmath>

#define NORMALIZE_HEIGHT(height) ((height) / (elevatorConstants::Settings::TOP_LIMIT))

enum class DriveMode
{
    ARCADE_DRIVE,
    CURVE_DRIVE,
    AUTO_PATH_FOLLOWER,
    DISABLE
};

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if (ROBOT_MODEL != (BABY_BRICE))
namespace driveConstants 
{
    constexpr DriveMode desiredDriveControl = DriveMode::ARCADE_DRIVE;
    namespace Motors
    {
        //LEFT GEARBOX
        constexpr int LEFT_FRONT_MOTOR_ID = 2;
        constexpr int LEFT_BACK_MOTOR_ID = 3;
        constexpr bool LEFT_MOTOR_INVERTED = true;

        //RIGHT GEARBOX
        constexpr int RIGHT_FRONT_MOTOR_ID = 4;
        constexpr int RIGHT_BACK_MOTOR_ID = 5;
        constexpr bool RIGHT_MOTORS_INVERTED = false;

        //BOTH
        constexpr IdleMode MOTOR_IDLE_MODE = IdleMode::kBrake;
        constexpr int MOTOR_CURRENT_LIMIT = 40;
        constexpr double MOTOR_RAMP = 0.0; //TUNEME
        constexpr double MOTOR_VOLTAGE_COMPENSATION = 12.0;
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (62.0/11.0) * (56.0 / 26.0); //ul
        constexpr int KV = 559; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motors::MOTOR_VOLTAGE_COMPENSATION * KV; //RPM
        
        constexpr double WHEEL_RADIUS = 2 * 0.0254; //m
        constexpr double TRACKWIDTH = 0.554; //m
        constexpr double BASE_TRACK_RADIUS = TRACKWIDTH/2.0; //m

        constexpr double MAX_LINEAR_SPEED = WHEEL_RADIUS * (2.0 * M_PI) * ((MOTOR_FREE_SPEED / GEAR_RATIO) / 60.0); // m.s-1
        constexpr double MAX_ROTATION_SPEED = MAX_LINEAR_SPEED / BASE_TRACK_RADIUS;
        constexpr double LINEAR_TO_MOTOR_SPEED_FACTOR = (30.0 * GEAR_RATIO) / (M_PI * WHEEL_RADIUS); // RPM.s.m-1
    }

    namespace Encoder
    {
        constexpr int LEFT_ID_ENCODER_A = 0;
        constexpr int LEFT_ID_ENCODER_B = 1;
        constexpr bool LEFT_REVERSE_ENCODER = true; 

        constexpr int RIGHT_ID_ENCODER_A = 2;
        constexpr int RIGHT_ID_ENCODER_B = 3;
        constexpr bool RIGHT_REVERSE_ENCODER = false;

        constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * Specifications::WHEEL_RADIUS)/ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace ArcadeDrive
    {
        constexpr double MIN_ROTATION_SIGMA = 0.1; //TUNEME
        constexpr double MAX_ROTATION_SIGMA = 0.45; //TUNEME

        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.5; //TUNEME

        constexpr double TIME_TO_STOP_FORWARD = 0.7; //TUNEME
        constexpr double TIME_TO_STOP_ROTATION = 0.35; //TUNEME
    }

    namespace CurveDrive //COMMENTME
    {
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8; //TUNEME
        constexpr double TIME_TO_STOP_FORWARD = 0.7; //TUNEME

        constexpr double SINUSOIDAL_CURVATURE_INTENSITY = 0.5; //TUNEME
        constexpr double DENOMINATOR = 0.7071067812; // Precomputed value of sin(SINUSOIDAL_CURVATURE_INTENSITY{0.5} * M_PI_2)

        constexpr double QUICK_STOP_ALPHA = 0.1; //TUNEME
        constexpr double NEG_INERTIA_SCALAR = 4.0; //TUNEME
        constexpr double TURN_SENSITIVITY   = 1.0; //TUNEM
    }

    namespace StabilityGuard
    {
        constexpr double MIN_MOVING_FORWARD = 0.07; //TUNEME
        constexpr double MIN_TURNING = 0.25; //TUNEME
    }

    namespace Settings
    {
        constexpr double SLOW_RATE = 2.0; //TUNEME
        constexpr double DEADBAND = 0.05; 
    }
}

#else //#elif (ROBOT_MODEL == (BABY_BRICE))
namespace driveConstants {

    namespace LeftGearbox{
        namespace Motor{
            constexpr int FRONT_MOTOR_ID = 2;
            constexpr int BACK_MOTOR_ID = 3;
            constexpr IdleMode MOTOR_IDLE_MODE = IdleMode::kBrake;
            constexpr bool MOTOR_INVERTED = true;
            constexpr int MOTOR_CURRENT_LIMIT = 40;
            constexpr double MOTOR_RAMP = 0.1;
            constexpr double MOTOR_VOLTAGE_COMPENSATION = 12.0;
        }

        namespace Encoder{
            constexpr int ID_ENCODER_A = 0;
            constexpr int ID_ENCODER_B = 1;
            constexpr bool REVERSE_ENCODER = true; 
            constexpr double RADIUS = 0.0254 *2;
            constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * RADIUS)/ENCODER_TICKS_PER_REVOLUTION_K2X;
        }
        constexpr bool WHEEL_SIDE = true;
    }

    namespace RightGearbox{
        namespace Motor{
            constexpr int FRONT_MOTOR_ID = 4;
            constexpr int BACK_MOTOR_ID = 5;
            constexpr rev::spark::SparkBaseConfig::IdleMode MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool MOTOR_INVERTED = false;
            constexpr int MOTOR_CURRENT_LIMIT = 40;
            constexpr double MOTOR_RAMP = 0.1;
            constexpr double MOTOR_VOLTAGE_COMPENSATION = 12.0;
        }

        namespace Encoder{
            constexpr int ID_ENCODER_A = 2;
            constexpr int ID_ENCODER_B = 3;
            constexpr bool REVERSE_ENCODER = false;
            constexpr double RADIUS = 0.0254 *2;
            constexpr double DISTANCE_PER_PULSE = (2 * M_PI * RADIUS)/ENCODER_TICKS_PER_REVOLUTION_K2X;
        }
        constexpr bool WHEEL_SIDE = false;
    }
}

#endif