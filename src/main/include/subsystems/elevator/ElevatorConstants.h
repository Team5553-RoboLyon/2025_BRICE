#pragma once
#include "Constants.h"
#include "lib/ControlMode.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if (ROBOT_MODEL != (BABY_BRICE))
namespace elevatorConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_SETPOINT;

    namespace Motors
    {
        const int ID_LEFT = 6;
        const int ID_RIGHT = 7;
        const double VOLTAGE_COMPENSATION = 10.0;
        const double CURRENT_LIMIT = 40.0;
        const double RAMP_RATE = 0.0;
        const bool INVERTED_LEFT = false;
        const bool INVERTED_RIGHT = true;
        const IdleMode IDLE_MODE = IdleMode::kBrake;
        const int HOT_THRESHOLD = 60; //TUNEME
        const int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Encoder 
    {
        constexpr int A_ID = 4;
        constexpr int B_ID = 5;
        constexpr bool REVERSED = true;
        constexpr double REDUCTION = 1.0;
        constexpr double CIRCUMFERENCE = 0.005*36.0; //COMMENTME
        constexpr double DISTANCE_PER_PULSE = CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace LimitSwitch 
    {
        constexpr int BOTTOM_2_ID = 6;
        constexpr int BOTTOM_ID = 7;
        constexpr bool IS_TRIGGERED = false;
    }

    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 10.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.2; //TUNEME
            constexpr double KG = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.001;
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 8.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.1; //TUNEME
            constexpr double KG = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.001;
        }
    }

    namespace Setpoint
    {
        constexpr double HOME = 0.00; //TUNEME
        constexpr double CORAL_STATION = 0.00; //TUNEME
        constexpr double L1 = 0.3; //TUNEME
        constexpr double L2 = 0.45; //TUNEME
        constexpr double L3 = 0.86; //TUNEME
        constexpr double L4 = 1.45; //TUNEME
        constexpr double VISION = 0.43; //TUNEME
        constexpr double TOLERANCE = 0.01; //TUNEME
    }

    namespace Speed 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double CALIBRATION = -0.25; //TUNEME
        constexpr double REST = 0.0;
    }
    
    namespace Settings
    {
        constexpr double TIME_TO_REACH_FULL_SPEED = 0.25; // only for manual open-loop //TUNEME
        constexpr double BOTTOM_LIMIT = 0.005; //TUNEME
        constexpr double TOP_LIMIT = 1.45; //TUNEME
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (TOP_LIMIT - BOTTOM_LIMIT) / (2.5/TIME_PER_CYCLE); //TUNEME
        constexpr double OPEN_LOOP_REDUC = -2.0;
    }
}
#else
namespace elevatorConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_PID;
    constexpr double OPEN_LOOP_REDUC = -2.0;
    namespace Motors
    {
        namespace Left
        {
            constexpr int ID = 6;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = false;
            constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
            constexpr int HOT_THRESHOLD = 60; //TUNEME
            constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
        }
        namespace Right
        {
            constexpr int ID = 7;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = true;
            constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
            constexpr int HOT_THRESHOLD = 60; //TUNEME
            constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
        }
    }
    namespace Encoder 
    {
        constexpr int A_ID = 4;
        constexpr int B_ID = 5;
        constexpr bool REVERSED = true;
        constexpr double REDUCTION = 1.0;
        constexpr double CIRCUMFERENCE = 0.005*36.0; //COMMENTME
        constexpr double DISTANCE_PER_PULSE = CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
    }
    namespace LimitSwitch 
    {
        constexpr int BOTTOM_2_ID = 6;
        constexpr int BOTTOM_ID = 7;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace PID
    {
        constexpr double KP = 10.0; //TUNEME
        constexpr double KI = 0.0; //TUNEME
        constexpr double KD = 0.2; //TUNEME
        constexpr double KFF = 0.0; //TUNEME
        constexpr double TOLERANCE = 0.001;
    }
    namespace Setpoint
    {
        constexpr double HOME = 0.00; //TUNEME
        constexpr double CORAL_STATION = 0.00; //TUNEME
        constexpr double L1 = 0.3; //TUNEME
        constexpr double L2 = 0.45; //TUNEME
        constexpr double L3 = 0.86; //TUNEME
        constexpr double L4 = 1.45; //TUNEME
        constexpr double VISION = 0.43; //TUNEME
        constexpr double TOLERANCE = 0.01; //TUNEME
    } 
    namespace Speed 
    {
        constexpr double MAX = 1.0; 
        constexpr double MIN = -1.0;
        constexpr double CALIBRATION = -0.25; //TUNEME
        constexpr double REST = 0.0;
    }
    namespace Settings
    {
        constexpr double TIME_TO_REACH_FULL_SPEED = 0.25; // only for open-loop //TUNEME
        constexpr double BOTTOM_LIMIT = 0.005; //TUNEME
        constexpr double TOP_LIMIT = 1.45; //TUNEME
    }
}
#endif