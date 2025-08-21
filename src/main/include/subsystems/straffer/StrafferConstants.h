#pragma once
#include "Constants.h"

#include "lib/ControlMode.h"
#include "lib/DebugUtils.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if (ROBOT_MODEL != (BABY_BRICE))
namespace strafferConstants 
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_POSITION;

    namespace Motor
    {
        constexpr int ID = 8;
        constexpr bool INVERTED = true;
    
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 55; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 70; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (4.0/1.0) * (4.0 / 1.0); //ul //TODO : find the real ratio
        constexpr int KV = 917; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * KV; //RPM

        constexpr double PULLEY_TEETH_NUMBER = 18.0; //ul
        constexpr double BELT_PITCH = 0.005; //m
        constexpr double PITCH_CIRCUMFERENCE = BELT_PITCH * PULLEY_TEETH_NUMBER; //m
    }

    namespace LimitSwitch
    {
        constexpr int LEFT_ID = 12;
        constexpr int RIGHT_ID = 11;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Encoder 
    {
        constexpr int A_ID = 8;
        constexpr int B_ID = 9;
        constexpr bool REVERSED = false;
        constexpr double REDUCTION = 1.0;
        constexpr double DISTANCE_PER_PULSE = Specifications::PITCH_CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace Speed 
    {
        constexpr double REST = 0.0;
        constexpr double MIN = -1.0;
        constexpr double MAX = 1.0;
        constexpr double CALIBRATION = -0.25; //TUNEME
    }

    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 6.5; //TUNEME
            constexpr double KI = 0.000; //TUNEME
            constexpr double KD = 0.4; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.005; //TUNEME
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 4.5; //TUNEME
            constexpr double KI = 0.000; //TUNEME
            constexpr double KD = 0.2; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.005; //TUNEME
        }
    }

    namespace Setpoint 
    {
        constexpr double LEFT_SIDE = 0.05; //TUNEME
        constexpr double RIGHT_SIDE = 0.34; //TUNEME
        constexpr double CENTER = 0.1975; //TUNEME
        constexpr double TOLERANCE = 0.005; //TUNEME
    } 

    namespace Settings
    {
        constexpr double TIME_TO_REACH_FULL_SPEED = 0.2; // only for manual open-loop //TUNEME
        constexpr double LEFT_LIMIT = 0.025; //TUNEME
        constexpr double RIGHT_LIMIT = 0.37; //TUNEME
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (RIGHT_LIMIT - LEFT_LIMIT) / (1.0/TIME_PER_CYCLE); //TUNEME
    } 

    namespace Seeking
    {
        constexpr int COUNTER = 10; //TUNEME
        constexpr double LEFT_OFFSET = -0.17; //TUNEME
        constexpr double RIGHT_OFFSET = 0.19; //TUNEME
        constexpr double HIGHEST_AMBIGUITY_ACCEPTED = 0.2;
    }
}

#else //#elif (ROBOT_MODEL == (BABY_BRICE))
namespace strafferConstants 
{
    constexpr ControlMode MainControlMode = ControlMode::POSITION_DUTYCYCLE_PID;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_SETPOINT;

    namespace Motor
    {
        constexpr int ID = 8;
        constexpr bool INVERTED = true;
    
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.0;
        constexpr int HOT_THRESHOLD = 55; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 70; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (4.0/1.0) * (4.0 / 1.0); //ul //TODO : find the real ratio
        constexpr int KV = 917; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * KV; //RPM

        constexpr double PULLEY_TEETH_NUMBER = 18.0; //ul
        constexpr double BELT_PITCH = 0.005; //m
        constexpr double PITCH_CIRCUMFERENCE = BELT_PITCH * PULLEY_TEETH_NUMBER; //m
    }

    namespace LimitSwitch
    {
        constexpr int LEFT_ID = 12;
        constexpr int RIGHT_ID = 11;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Encoder 
    {
        constexpr int A_ID = 8;
        constexpr int B_ID = 9;
        constexpr bool REVERSED = false;
        constexpr double REDUCTION = 1.0;
        constexpr double DISTANCE_PER_PULSE = Specifications::PITCH_CIRCUMFERENCE / REDUCTION / ENCODER_TICKS_PER_REVOLUTION_K2X;
    }

    namespace Speed 
    {
        constexpr double REST = 0.0;
        constexpr double MIN = -1.0;
        constexpr double MAX = 1.0;
        constexpr double CALIBRATION = -0.25; //TUNEME
    }

    namespace Gains
    {
        namespace POSITION_DUTYCYCLE_PID
        {
            constexpr double KP = 6.5; //TUNEME
            constexpr double KI = 0.000; //TUNEME
            constexpr double KD = 0.4; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.005; //TUNEME
        }
        namespace MANUAL_SETPOINT_PID
        {
            constexpr double KP = 4.5; //TUNEME
            constexpr double KI = 0.000; //TUNEME
            constexpr double KD = 0.2; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double TOLERANCE = 0.005; //TUNEME
        }
    }

    namespace Setpoint 
    {
        constexpr double LEFT_SIDE = 0.05; //TUNEME
        constexpr double RIGHT_SIDE = 0.34; //TUNEME
        constexpr double CENTER = 0.1975; //TUNEME
        constexpr double TOLERANCE = 0.005; //TUNEME
    } 

    namespace Settings
    {
        constexpr double TIME_TO_REACH_FULL_SPEED = 0.2; // only for manual open-loop //TUNEME
        constexpr double LEFT_LIMIT = 0.025; //TUNEME
        constexpr double RIGHT_LIMIT = 0.37; //TUNEME
        constexpr double MANUAL_SETPOINT_CHANGE_LIMIT = (RIGHT_LIMIT - LEFT_LIMIT) / (1.0/TIME_PER_CYCLE); //TUNEME
    } 

    namespace Seeking
    {
        constexpr int COUNTER = 10; //TUNEME
        constexpr double LEFT_OFFSET = -0.17; //TUNEME
        constexpr double RIGHT_OFFSET = 0.19; //TUNEME
        constexpr double HIGHEST_AMBIGUITY_ACCEPTED = 0.2;
    }
}
#endif