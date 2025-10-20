#pragma once
#include "Constants.h"
#include "lib/ControlMode.h"
#include "lib/DebugUtils.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if (ROBOT_MODEL != (BABY_BRICE))
namespace elevatorConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_POSITION;

    namespace Motors
    {
        constexpr int ID_LEFT = 6;
        constexpr int ID_RIGHT = 7;
        constexpr bool INVERTED_LEFT = false;
        constexpr bool INVERTED_RIGHT = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (3.0/1.0) * (4.0 / 1.0); //ul //TODO : find the real ratio
        constexpr int KV = 473; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motors::VOLTAGE_COMPENSATION * KV; //RPM

        constexpr double PULLEY_TEETH_NUMBER = 36.0; //ul
        constexpr double BELT_PITCH = 0.005; //m
        constexpr double PITCH_CIRCUMFERENCE = BELT_PITCH * PULLEY_TEETH_NUMBER; //m
    }

    namespace Encoder 
    {
        constexpr int A_ID = 4;
        constexpr int B_ID = 5;
        constexpr bool REVERSED = true;
        constexpr double REDUCTION = 1.0;
        constexpr double DISTANCE_PER_PULSE = Specifications::PITCH_CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
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
        constexpr double OPEN_LOOP_REDUC = 2.0;
    }
}
#else
namespace elevatorConstants
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_SETPOINT;

    namespace Motors
    {
        constexpr int ID_LEFT = 6;
        constexpr int ID_RIGHT = 7;
        constexpr bool INVERTED_LEFT = false;
        constexpr bool INVERTED_RIGHT = true;

        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (3.0/1.0) * (4.0 / 1.0); //ul //TODO : find the real ratio
        constexpr int KV = 473; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motors::VOLTAGE_COMPENSATION * KV; //RPM

        constexpr double PULLEY_TEETH_NUMBER = 36.0; //ul
        constexpr double BELT_PITCH = 0.005; //m
        constexpr double PITCH_CIRCUMFERENCE = BELT_PITCH * PULLEY_TEETH_NUMBER; //m
    }

    namespace Encoder 
    {
        constexpr int A_ID = 4;
        constexpr int B_ID = 5;
        constexpr bool REVERSED = true;
        constexpr double REDUCTION = 1.0;
        constexpr double DISTANCE_PER_PULSE = Specifications::PITCH_CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
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
        constexpr double OPEN_LOOP_REDUC = 2.0;
    }
}
#endif