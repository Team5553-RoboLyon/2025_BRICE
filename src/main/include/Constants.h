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
constexpr double TIME_PER_CYCLE = 0.02; // 20ms

enum class ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
};

namespace outtakeConstants
{   
    constexpr ControlMode defaultMode = ControlMode::CLOSED_LOOP;
    constexpr double TIME_FOR_DROP = 0.2 / TIME_PER_CYCLE;
    constexpr double TIME_FOR_RUMBLE = 0.4 / TIME_PER_CYCLE;
    namespace Motor
    {
        constexpr int ID = 9;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.5553;
        constexpr bool INVERTED = true;         //TODO : test rotation
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
    }
    namespace Sensor 
    {
        namespace IRbreaker
        {
            constexpr int DOWN_ID = 6;
            constexpr int UP_ID = 7;
            constexpr bool IS_TRIGGERED = true;
        }
    }
    
    namespace Speed 
    {
        constexpr double REST = 0.0;
        constexpr double STARTING = 1.0;
        constexpr double CATCHING = 0.5;
        constexpr double DROPPING = STARTING;
        constexpr double UP = -0.5;
    }
}


namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int COPILOT_CONTROLLER_ID = 2;
    }
    namespace Button { // TODO : recheck triggers
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
