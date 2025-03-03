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
        constexpr uint8_t L1 = 0;
        constexpr uint8_t L2 = 1;
        constexpr uint8_t L3 = 2;
        constexpr uint8_t L4 = 3;
    }

    namespace State {
        constexpr uint16_t m_maskDesiredPosition = 0xF;      // 0000 0000 0000 0000 0000 0000 0000 1111
        constexpr uint16_t m_maskCurrentPosition = 0xF << 4; // 0000 0000 0000 0000 0000 0000 1111 0000
        constexpr uint16_t m_maskMovingType = 0xF << 8;      // 0000 0000 0000 0000 0000 1111 0000 0000

        constexpr uint16_t m_L1Desired =  0b0000;
        constexpr uint16_t m_L2Desired =  0b0010;
        constexpr uint16_t m_L3Desired =  0b0100;
        constexpr uint16_t m_L4Desired =  0b0110;

        constexpr uint16_t m_AtL1 =      0b0000   << 4;
        constexpr uint16_t m_AtL1L2 =    0b0001   << 4;
        constexpr uint16_t m_AtL2 =      0b0010   << 4;
        constexpr uint16_t m_AtL2L3 =    0b0011   << 4;
        constexpr uint16_t m_AtL3 =      0b0100   << 4;
        constexpr uint16_t m_AtL3L4 =    0b0101   << 4;
        constexpr uint16_t m_AtL4 =      0b0110   << 4;

        constexpr uint16_t Rest =         0b0000   << 8;
        constexpr uint16_t Up =           0b0001   << 8;
        constexpr uint16_t Down =        0b0010    << 8;
    }
    #define MAKE_STATE(desiredPosition, currentPosition, movingType) ((desiredPosition) | ((currentPosition) << 4) | ((movingType) << 8))
    // #define CLEAR_DESIRED_POSITION(val) (BITSGET(val, ~elevatorConstants::State::m_maskDesiredPosition))
    // #define CLEAR_CURRENT_POSITION(val) (BITSGET(val,  ~elevatorConstants::State::m_maskCurrentPosition))
    // #define CLEAR_MOVING_TYPE(val) (BITSGET(val, ~elevatorConstants::State::m_maskMovingType))

    #define GET_DESIRED_POSITION(val) BITSGET(val, elevatorConstants::State::m_maskDesiredPosition)
    #define GET_CURRENT_POSITION(val) (BITSGET(val,  elevatorConstants::State::m_maskCurrentPosition) >> 4)
    #define GET_MOVING_TYPE(val) (BITSGET(val,  elevatorConstants::State::m_maskMovingType) >> 8)

    #define SET_DESIRED_POSITION(bits, setValue) (BITSSET((BITSGET(bits, ~elevatorConstants::State::m_maskDesiredPosition)), setValue))
    #define SET_CURRENT_POSITION(bits, setValue) (BITSSET((BITSGET(bits, ~elevatorConstants::State::m_maskCurrentPosition)), setValue))
    #define SET_MOVING_TYPE(bits, setValue) (BITSSET((BITSGET(bits, ~elevatorConstants::State::m_maskMovingType)), setValue))
}