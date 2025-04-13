#pragma once
#include "rev/SparkMax.h"
#include "lib/UtilsRBL.h"
#include "lib/rate_limiter.h"

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
    CLOSED_LOOP,
    OPEN_LOOP,
    AUTO_LOOP
};
enum class Stage {
    HOME,
    L1,
    CORAL_STATION,
    L2,
    L3,
    L4
};
enum class Side {
    LEFT,
    CENTER,
    RIGHT
};

#define NORMALIZE_HEIGHT(height) ((height) / (elevatorConstants::Settings::TOP_LIMIT))
namespace elevatorConstants
{   
    constexpr ControlMode defaultMode = ControlMode::OPEN_LOOP; // Might be Closed Loop
    namespace Motors
    {
        namespace Left
        {
            constexpr int ID = 6;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = false;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
        namespace Right
        {
            constexpr int ID = 7;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = true;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
    }
    namespace Sensor 
    {
        namespace Encoder 
        {
            constexpr int A_ID = 4;
            constexpr int B_ID = 5;
            constexpr bool REVERSED = true;
            constexpr double REDUCTION = 1.0;
            constexpr double CIRCUMFERENCE = 0.005*36.0;
            constexpr double DISTANCE_PER_PULSE = CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
        }
        namespace LimitSwitch 
        {
            constexpr int BOTTOM_2_ID = 6;
            constexpr int BOTTOM_ID = 7;
            constexpr bool IS_TRIGGERED = false;
        }
    }
    namespace PID // TODO : set good Ks
    {
        constexpr double KP = 8.0;
        constexpr double KI = 0.0;
        constexpr double KD = 0.00;
        constexpr double TOLERANCE = 0.001;
    }
    namespace Setpoint // TODO : calibrate these values
    {
        constexpr double HOME = 0.00;
        constexpr double CORAL_STATION = 0.00;
        constexpr double L1 = 0.3;
        constexpr double L2 = 0.55;
        constexpr double L3 = 0.95;
        constexpr double L4 = 1.45;
    } 
    namespace Speed 
    {
        constexpr double MAX = 1.0;
        constexpr double MIN = -1.0;
        constexpr double CALIBRATION = -0.25;
        constexpr double REST = 0.0;
    }
    namespace Settings
    {
        constexpr double RATE_LIMITER = TIME_TO_REACH_MAX(0.25); // only for open-loop
        constexpr double BOTTOM_LIMIT = 0.04;
        constexpr double TOP_LIMIT = 1.45;
        constexpr double JOYSTICK_REDUCTION = -2.0;
    }
}

namespace strafferConstants
{   
    constexpr ControlMode defaultMode = ControlMode::OPEN_LOOP; // Might be Closed Loop
    namespace Motor
    {
        constexpr int ID = 8;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr bool INVERTED = true; 
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
    }
    namespace Sensor 
    {
        namespace LimitSwitch
        {
            constexpr int LEFT_ID = 12;
            constexpr int RIGHT_ID = 11;
            constexpr bool IS_TRIGGERED = false;;
        }
        namespace Encoder 
        {
            constexpr int A_ID = 8;
            constexpr int B_ID = 9;
            constexpr bool REVERSED = false;
            constexpr double REDUCTION = 1.0;
            constexpr double CIRCUMFERENCE = (0.005*18);
            constexpr double DISTANCE_PER_PULSE = CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
        }
    }
    namespace Speed 
    {
        constexpr double REST = 0.0;
        constexpr double MIN = -1.0;
        constexpr double MAX = 1.0;
        constexpr double CALIBRATION = - 0.25; //TODO : calibrate this value
    }
    namespace PID // TODO : set good Ks
    {
        constexpr double KP = 5.0;
        constexpr double KI = 0.0;
        constexpr double KD = 0.00;
        constexpr double TOLERANCE = 0.000;
    }
    namespace Setpoint // TODO : calibrate these values
    {
        constexpr double LEFT_SIDE = 0.05;
        constexpr double RIGHT_SIDE = 0.31;
        constexpr double CENTER = 0.1945;
    } 
    namespace Settings
    {
        constexpr double RATE_LIMITER = TIME_TO_REACH_MAX(0.2); // only for open-loop
        constexpr double LEFT_LIMIT = 0.05;
        constexpr double RIGHT_LIMIT = 0.32;
    }
}

namespace driveConstants {

    namespace LeftGearbox{
        namespace Motor{
            constexpr int FRONT_MOTOR_ID = 2;
            constexpr int BACK_MOTOR_ID = 3;
            constexpr rev::spark::SparkBaseConfig::IdleMode MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool MOTOR_INVERTED = false;
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
            constexpr bool MOTOR_INVERTED = true;
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

namespace gripperConstants
{   
    constexpr ControlMode defaultMode = ControlMode::CLOSED_LOOP;
    constexpr double TIME_FOR_DROP = 0.2 / TIME_PER_CYCLE;
    constexpr double TIME_RUMBLE_CAUGHT = 0.36 / TIME_PER_CYCLE;
    constexpr double TIME_RUMBLE_DROPPED = 0.16 / TIME_PER_CYCLE;
    namespace Motors
    {
        namespace Outtake 
        {
            constexpr int ID = 9;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.1;
            constexpr bool INVERTED = false;         //TODO : test rotation
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
        namespace Intake 
        {
            constexpr int ID = 10;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.1;
            constexpr bool INVERTED = true;         //TODO : test rotation
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
    }
    namespace Sensor 
    {
        namespace IRbreaker
        {
            constexpr int DOWN_ID = 13;
            constexpr int UP_ID = 18;
            constexpr int UP2_ID = 19;
            constexpr bool IS_TRIGGERED = false;
        }
    }
    
    namespace Speed  // TODO : calibrate these values
    {
        constexpr double REST = 0.0;
        constexpr double SHY = 0.05;
        constexpr double INTAKE_EMPTY = 0.5553;
        constexpr double OUTTAKE_EMPTY = 0.4;
        constexpr double FEEDING_FORWARD = 0.3;
        constexpr double FEEDING_BACKWARD = -0.15;
        constexpr double PRESHOOT = -0.2;
        constexpr double SHOOTTTT = 1;

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
        constexpr int L2 = 3;
        constexpr int L3 = 2;
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
        constexpr double RATE_LIMITER_FOWARD = TIME_TO_REACH_MAX(0.8);
        constexpr double RATE_LIMITER_ROTATION = TIME_TO_REACH_MAX(0.5);
        constexpr double DEADBAND_OPEN_LOOP = 0.1;
        constexpr double MIN_MOVING_V = 0.07;
        constexpr double MIN_MOVING_W = 0.25;
    }
}