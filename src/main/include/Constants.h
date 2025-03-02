#pragma once
#include <math.h>
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
const double ENCODER_TICKS_PER_REVOLUTION = 2048.0;


namespace elevatorConstants
{
    namespace Motors
    {
        constexpr int ID = 7;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.2;
        constexpr bool INVERTED = false;
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;

    }
    namespace Sensor
    {
        namespace HallEffect
        {
            constexpr int ID_TOP = 2;
            constexpr int ID_MIDDLE = 3;
            constexpr int ID_BOTTOM = 4;
            //TODO : Tune this value
            constexpr double THRESHOLD = 3.0;
        }
        namespace Encoder
        {
            constexpr int A_ID = 6;
            constexpr int B_ID = 7;
            //TODO : Check these values
            constexpr double REDUCTION = 30.0/26.0;
            constexpr double RADIUS = 1.0;
            constexpr double DISTANCE_PER_PULSE = 2.0 * M_PI * RADIUS / REDUCTION / ENCODER_TICKS_PER_REVOLUTION;
            namespace Distance {
                //ckech these values
                constexpr double L1 = 0.0;
                constexpr double L2 = 0.5;
                constexpr double L3 = 1.0;
                constexpr double L4 = 1.5;
                constexpr double TOLERANCE_PER_STAGE = 0.5;
            }
        }
    }
    namespace PID {
        //TODO : Tune these values
        constexpr double KP = 0.1;
        constexpr double KI = 0.0;
        constexpr double KD = 0.0;
        constexpr double TOLERANCE = 0.1;
        constexpr double SETPOINT = 0.0;
    }
    namespace Speed {
        constexpr double MAX_SPEED = 1.0;
        constexpr double MIN_SPEED = -1.0;
    }
    namespace Stage {
        constexpr int8_t L1 = 0;
        constexpr int8_t L2 = 1;
        constexpr int8_t L3 = 2;
        constexpr int8_t L4 = 3;
    }

    namespace State {
        constexpr int16_t m_maskMovingType = 0x3F;
        constexpr int16_t m_maskCurrentPosition = 0x1C7;
        constexpr int16_t m_maskDesiredPosition = 0x1F8;

        constexpr int8_t m_L1Desired =  0b000;
        constexpr int8_t m_L2Desired =  0b010;
        constexpr int8_t m_L3Desired =  0b100;
        constexpr int8_t m_L4Desired =  0b110;

        constexpr int8_t m_AtL1 =      0b000   << 3;
        constexpr int8_t m_AtL1L2 =    0b001   << 3;
        constexpr int8_t m_AtL2 =      0b010   << 3;
        constexpr int8_t m_AtL2L3 =    0b011   << 3;
        constexpr int8_t m_AtL3 =      0b100   << 3;
        constexpr int8_t m_AtL3L4 =    0b101   << 3;
        constexpr int8_t m_AtL4 =      0b110   << 3;

        constexpr int8_t Rest =         0b000   << 6;
        constexpr int8_t Up =           0b001   << 6;
        constexpr int16_t Down =        0b010   << 6;
    }
    #define CLEAR_DESIRED_POSITION(val) BITSGET(val, elevatorConstants::State::m_maskDesiredPosition)
    #define CLEAR_CURRENT_POSITION(val) BITSGET(val,  elevatorConstants::State::m_maskCurrentPosition)
    #define CLEAR_MOVING_TYPE(val) BITSGET(val,  elevatorConstants::State::m_maskMovingType)
    #define GET_DESIRED_POSITION(val) BITSGET(val, !elevatorConstants::State::m_maskDesiredPosition)
    #define GET_CURRENT_POSITION(val) BITSGET(val,  !elevatorConstants::State::m_maskCurrentPosition) >> 3
    #define GET_MOVING_TYPE(val) BITSGET(val,  !elevatorConstants::State::m_maskMovingType) >> 6

    #define SET_DESIRED_POSITION(bits, setValue) BITSSET(bits, setValue)
    #define SET_CURRENT_POSITION(bits, setValue) BITSSET(bits, setValue)
    #define SET_MOVING_TYPE(bits, setValue) BITSSET(bits, setValue)
}