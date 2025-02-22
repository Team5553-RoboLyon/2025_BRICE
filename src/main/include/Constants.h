#pragma once
#include "rev/SparkMax.h"

// Write ALL your robot-specific constants here
// Use namespacing to group constants together depending on the subsystems
// Example: namespace DriveConstants {}
// Example for variable : constexpr int kMOTOR_DRIVE = 0;
// Example for macro : #define SQUARE(x) x*x

namespace elevatorConstants {
    namespace Motors {
        constexpr int ID = 7;
        constexpr double VOLTAGE_COMP = 12.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.2;
        constexpr bool INVERTED = false;
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;

    }
    namespace Sensor {
        constexpr int ID_TOP = 1;
        constexpr int ID_MIDDLE = 3;
        constexpr int ID_BOTTOM = 4;
        constexpr double TOLERANCE = 3.0;
    }
    namespace Encoder {
        constexpr int ENCODER_A_ID = 6;
        constexpr int ENCODER_B_ID = 7;
        constexpr double REDUCTION = 30.0/26.0;
        constexpr double RADIUS = 1.0;
        constexpr double DISTANCE_PER_PULSE = 2.0 * 3.14159265358979323846 * RADIUS / 2048.0 / REDUCTION;
    }
}