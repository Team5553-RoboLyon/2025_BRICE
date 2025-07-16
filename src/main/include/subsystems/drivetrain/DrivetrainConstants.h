#pragma once
#include "Constants.h"

//TODO : rewrite constants in a proper way
#define NORMALIZE_HEIGHT(height) ((height) / (elevatorConstants::Settings::TOP_LIMIT))

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if (ROBOT_MODEL != (BABY_BRICE))
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



    namespace Button {
        // FORWARD Joystick
        constexpr int REVERSED_DRIVE_BUTTON = 1;
        // ROTATION Joystick
        constexpr int SLOW_DRIVE_BUTTON = 1;
    }
    namespace Settings{
        constexpr double SLOW_RATE = 2.0;
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8;
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.5;
        constexpr double DEADBAND= 0.05;
        constexpr double MIN_MOVING_V = 0.07;
        constexpr double MIN_MOVING_W = 0.25;
    }