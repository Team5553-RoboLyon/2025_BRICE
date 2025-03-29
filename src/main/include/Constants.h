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
constexpr double ENCODER_TICKS_PER_REVOLUTION = 2048.0;

enum class DriveMode {
    CLOSED_LOOP,
    OPEN_LOOP
};
enum class Stage {
    HOME,
    L1,
    CORAL_STATION,
    L2,
    L3,
    L4
};

namespace elevatorConstants
{   
    constexpr DriveMode defaultMode = DriveMode::OPEN_LOOP;
    namespace Motors
    {
        namespace Left
        {
            constexpr int ID = 6;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = true;         //TODO : test rotation
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
        namespace Right
        {
            constexpr int ID = 7;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = true;         //TODO : test rotation
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
    }
    namespace Sensor 
    {
        namespace Encoder 
        {
            constexpr int A_ID = 4;
            constexpr int B_ID = 5;
            constexpr bool REVERSED = false;     //TODO : test rotation
            constexpr double REDUCTION = 1.0; //TODO : test reduc
            constexpr double RADIUS = (0.005*36.0/M_PI)/2.0; //TODO : test radius
            constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * RADIUS) / REDUCTION / ENCODER_TICKS_PER_REVOLUTION;
        }
        namespace LimitSwitch 
        {
            constexpr int TOP_ID = 6;
            constexpr int TOP_2_ID = 7;
            constexpr int BOTTOM_ID = 8;
            constexpr bool IS_TRIGGERED = true;
        }
    }
    namespace PID 
    {
        constexpr double KP = 8.0;
        constexpr double KI = 0.0;
        constexpr double KD = 0.00;
        constexpr double TOLERANCE = 0.001;
    }
    namespace Setpoint
    {
        constexpr double HOME = 0.0;
        constexpr double L1 = 0.0;
        constexpr double CORAL_STATION = 0.25;
        constexpr double L2 = 0.5;
        constexpr double L3 = 0.75;
        constexpr double L4 = 1.0;
    } 
    
    namespace Speed 
    {
        constexpr double MAX = 1.0;
        constexpr double MIN = -1.0;
        constexpr double CALIBRATION = -0.3;
        constexpr double REST = 0.0;
    }
}


namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int XBOX_CONTROLLER_ID = 2;
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
        constexpr int TAKE = 5;
        constexpr int OUTTAKE = 6;
        constexpr int OPEN_LOOP = 7;
    }
}
