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
constexpr double ENCODER_TICKS_PER_REVOLUTION_K4X  = 8192.0;
constexpr double TIME_PER_CYCLE = 0.02; // 20ms

enum class ControlMode {
    CLOSED_LOOP,
    OPEN_LOOP
};

namespace strafferConstants
{   
    constexpr ControlMode defaultMode = ControlMode::OPEN_LOOP;
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
            constexpr bool IS_TRIGGERED = true;
            constexpr bool IS_RIGHT_TRIGGERED = false;
        }
        namespace Encoder 
        {
            constexpr int A_ID = 8; // TODO : change LDCHT
            constexpr int B_ID = 9;
            constexpr bool REVERSED = false;
            constexpr double REDUCTION = 1.0;
            constexpr double RADIUS = (0.005*18/M_PI)/2.0;
            constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * RADIUS) / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
        }
    }
    
    namespace Speed 
    {
        //speed for open loop
        constexpr double MIN = -1.0;
        constexpr double MAX = 1.0;
        constexpr double INIT = - 0.15;
    }
    namespace PID // TODO : set good Ks
    {
        constexpr double KP = 1.0;
        constexpr double KI = 0.0;
        constexpr double KD = 0.00;
        constexpr double TOLERANCE = 0.001;
    }
    namespace Setpoint
    {
        constexpr double LEFT_SIDE = 0.0;
        constexpr double RIGHT_SIDE = 0.0;
        constexpr double CENTER = 0.25;
    } 
    namespace Settings
    {
        constexpr double RATE_LIMITER = TIME_TO_REACH_MAX(0.2);
        constexpr double LEFT_LIMIT = 0.05;
        constexpr double RIGHT_LIMIT = 0.32;
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