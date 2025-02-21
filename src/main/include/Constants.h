#pragma once

#include "rev/SparkMax.h"

// Write ALL your robot-specific constants here
// Use namespacing to group constants together depending on the subsystems
// Example: namespace DriveConstants {}
// Example for variable : constexpr int kMOTOR_DRIVE = 0;
// Example for macro : #define SQUARE(x) x*x

namespace DeepClimbConstants {

    constexpr double REDUCTION = 10.0/3.0;

    namespace Motors {
            constexpr int ID_FRONT = 5;
            constexpr int ID_BACK = 6;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool INVERTED = false;
            constexpr int CURRENT_LIMIT = 40;
            constexpr double RAMP = 0.5;
            constexpr double VOLTAGE_COMPENSATION = 10.0; //12
    }
    namespace Encoder {
        constexpr int ENCODER_A_ID = 0;
        constexpr int ENCODER_B_ID = 1;
        namespace Settings {
            constexpr bool REVERSED = false;
            constexpr double DISTANCE_PER_PULSE = ( 360.0 / REDUCTION ) / 2048; //2048 is the resolution of the encoder
        }
    }
    namespace HallEffectSensor {
        constexpr int UP = 0;
        constexpr int DOWN = 1;
    }
    namespace PID {
        constexpr double SETPOINT = 0.0;
        constexpr double KP = 0.1;
        constexpr double KI = 0.0;
        constexpr double KD = 0.0;
        constexpr double TOLERANCE = 0.1;
    }
}
