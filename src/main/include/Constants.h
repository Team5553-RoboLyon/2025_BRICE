#pragma once
#include "rev/SparkMax.h"
#include "lib/UtilsRBL.h"
// Write ALL your robot-specific constants here
// Use namespacing to group constants together depending on the subsystems
// Example: namespace DriveConstants {}
// Example for variable : constexpr int kMOTOR_DRIVE = 0;
// Example for macro : #define SQUARE(x) x*x

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


constexpr double ENCODER_TICKS_PER_REVOLUTION_K2X = 2048.0;
constexpr double TIME_PER_CYCLE = 0.02; // 20ms

enum class ControlMode {
    PROFILED_PID, //Motion profiling + PID sur l'output
    MOTION_PROFILING, 
    POSITION_PID, 
    VELOCITY, // RPM ou RPS
    VOLTAGE, // volts (-12V à 12V)
    DUTY_CYCLE, // percentage (-1 to 1)
    OPEN_LOOP // bypass the StateMachine
};
#define ALLOWS_STATE_MACHINE(mode) ((mode) != (ControlMode::OPEN_LOOP))

#define NORMALIZE_HEIGHT(height) ((height) / (elevatorConstants::Settings::TOP_LIMIT))


namespace driveConstants {

    namespace LeftGearbox{
        namespace Motor{
            constexpr int FRONT_MOTOR_ID = 2;
            constexpr int BACK_MOTOR_ID = 3;
            constexpr rev::spark::SparkBaseConfig::IdleMode MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
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

namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int COPILOT_CONTROLLER_ID = 2;
    }
    namespace Button {
        // FORWARD Joystick
        constexpr int REVERSED_DRIVE_BUTTON = 1;
        // ROTATION Joystick
        constexpr int SLOW_DRIVE_BUTTON = 1; 
        // XBOX_CONTROLLER
        constexpr int CORAL_STATION = 1;
        constexpr int L1 = 8;
        constexpr int L2 = 2;
        constexpr int L3 = 3;
        constexpr int L4 = 4;
        constexpr int LEFT_SIDE = 5;
        constexpr int RIGHT_SIDE = 6;
        constexpr int OPEN_LOOP_OUTTAKE = 7;
        constexpr int OPEN_LOOP_ELEVATOR = 9;
        constexpr int OPEN_LOOP_STRAFFER = 10;
    }
    namespace Settings{
        constexpr double SLOW_RATE = 2.0;
        constexpr double DEADBAND = 0.05;
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8;
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.5;
        constexpr double DEADBAND_OPEN_LOOP = 0.1;
        constexpr double MIN_MOVING_V = 0.07;
        constexpr double MIN_MOVING_W = 0.25;
    }
}