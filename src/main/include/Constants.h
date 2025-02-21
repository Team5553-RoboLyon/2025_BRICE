#pragma once

#include "rev/SparkMax.h"

// Write ALL your robot-specific constants here
// Use namespacing to group constants together depending on the subsystems
// Example: namespace DriveConstants {}
// Example for variable : constexpr int kMOTOR_DRIVE = 0;
// Example for macro : #define SQUARE(x) x*x

namespace DeepClimbConstants {
    namespace Motors {
        namespace Front {
            constexpr int ID = 5;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool INVERTED = false;
            constexpr int CURRENT_LIMIT = 40;
            constexpr double RAMP = 0.5;
            constexpr double VOLTAGE_COMPENSATION = 10.0; //12
        }
        namespace Back {
            constexpr int ID = 6;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool INVERTED = false;
            constexpr int CURRENT_LIMIT = 40;
            constexpr double RAMP = 0.5;
            constexpr double VOLTAGE_COMPENSATION = 10.0; //12
        }
    }
    namespace Encoder {
        constexpr double ENCODER_A_ID = 0;
        constexpr double ENCODER_B_ID = 1;
        namespace Settings {
            constexpr bool REVERSED = false;
            constexpr double DISTANCE_PER_PULSE = 360.0 * 1; //check the reduction
        }
    }
    namespace HallEffectSensor {
        constexpr int UP = 0;
        constexpr int DOWN = 1;
    }
}
