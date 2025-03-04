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
constexpr double ENCODER_TICKS_PER_REVOLUTION = 2048.0;


namespace elevatorConstants
{
    constexpr int TIMEOUT = 260/20;
    namespace Motors
    {
        constexpr int ID = 8;
        constexpr double VOLTAGE_COMPENSATION = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.2;
        constexpr bool INVERTED = false;
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;

    }
    namespace Sensor
    {
        // namespace HallEffect
        // {
        //     constexpr int ID_L2 = 2;
        //     constexpr int ID_L3 = 3;
        //     //TODO : Tune this value
        //     constexpr double THRESHOLD = 3.0;
        // }
        namespace Encoder
        {
            constexpr int A_ID = 6;
            constexpr int B_ID = 7;
            //TODO : Check these values
            constexpr double REDUCTION = 30.0/26.0;
            constexpr double RADIUS = (3.60*3.0/M_PI)/2.0;
            constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * RADIUS) / REDUCTION / ENCODER_TICKS_PER_REVOLUTION;
        }
        namespace LimitSwitch
        {
            constexpr int BOTTOM_ID = 11;
            constexpr int TOP_ID = 12;
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

    namespace State {
        constexpr uint16_t maskDesiredPosition = 0xF;      // 0000 0000 0000 0000 0000 0000 0000 1111
        constexpr uint16_t maskCurrentPosition = 0xF << 4; // 0000 0000 0000 0000 0000 0000 1111 0000
        constexpr uint16_t maskMovingType = 0xF << 8;      // 0000 0000 0000 0000 0000 1111 0000 0000

        // constexpr uint16_t m_L1Desired =  0b0000;
        // constexpr uint16_t m_L2Desired =  0b0010;
        // constexpr uint16_t m_L3Desired =  0b0100;
        // constexpr uint16_t m_L4Desired =  0b0110;

        // constexpr uint16_t m_AtL1 =      0b0000   << 4;
        // constexpr uint16_t m_AtL1L2 =    0b0001   << 4;
        // constexpr uint16_t m_AtL2 =      0b0010   << 4;
        // constexpr uint16_t m_AtL2L3 =    0b0011   << 4;
        // constexpr uint16_t m_AtL3 =      0b0100   << 4;
        // constexpr uint16_t m_AtL3L4 =    0b0101   << 4;
        // constexpr uint16_t m_AtL4 =      0b0110   << 4;

        // constexpr uint16_t Rest =         0b0000   << 8;
        // constexpr uint16_t Up =           0b0001   << 8;
        // constexpr uint16_t Down =        0b0010    << 8;

        constexpr uint16_t L1 =      0b0000;
        constexpr uint16_t L1L2 =    0b0001;
        constexpr uint16_t L2 =      0b0010;
        constexpr uint16_t L2L3 =    0b0011;
        constexpr uint16_t L3 =      0b0100;
        constexpr uint16_t L3L4 =    0b0101;
        constexpr uint16_t L4 =      0b0110;

        constexpr uint16_t Rest =         0b0000;
        constexpr uint16_t Up =           0b0001;
        constexpr uint16_t Down =         0b0010;
    }
    #define MAKE_ELEVATOR_STATE(desiredPosition, currentPosition, movingType) (((movingType) <<8) | ((currentPosition) << 4) | (desiredPosition))
    
    // #define CLEAR_ELEVATOR_DESIRED_POSITION(val) (BITSGET(val, ~elevatorConstants::State::m_maskDesiredPosition))
    // #define CLEAR_ELEVATOR_CURRENT_POSITION(val) (BITSGET(val,  ~elevatorConstants::State::m_maskCurrentPosition))
    // #define CLEAR_ELEVATOR_MOVING_TYPE(val) (BITSGET(val, ~elevatorConstants::State::m_maskMovingType))

    #define GET_ELEVATOR_DESIRED_POSITION(bits) ((bits) &  (elevatorConstants::State::maskDesiredPosition))
    #define GET_ELEVATOR_CURRENT_POSITION(bits) (((bits) & (elevatorConstants::State::maskCurrentPosition)) >> 4)
    #define GET_ELEVATOR_MOVING_TYPE(bits) (((bits) &  (elevatorConstants::State::maskMovingType)) >> 8)

    #define SET_ELEVATOR_DESIRED_POSITION(bits, setValue) ((((bits) & (~elevatorConstants::State::maskDesiredPosition)) | (setValue)))
    #define SET_ELEVATOR_CURRENT_POSITION(bits, setValue) ((((bits) & (~elevatorConstants::State::maskCurrentPosition)) | ((setValue) <<4)))
    #define SET_ELEVATOR_MOVING_TYPE(bits, setValue) ((((bits) & (~elevatorConstants::State::maskMovingType)) | ((setValue) << 8)))
    #define SET_ELEVATOR_REST_AT_POSITION(setValue) (MAKE_ELEVATOR_STATE((setValue), (setValue), elevatorConstants::State::Rest))

    // inline double ELEVATOR_SLOWER(double value) {
    //     return 1.0 - ((value - elevatorConstants::Sensor::HallEffect::THRESHOLD) / 2.0);
    // }
    #define IS_TRANSITION_POSITION(val) ((val) & 1)
}

namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int XBOX_CONTROLLER_ID = 2;
    }
    namespace Button {
        constexpr int L2 = 1;
        constexpr int L3 = 2;
        constexpr int L4 = 4;
    }
}