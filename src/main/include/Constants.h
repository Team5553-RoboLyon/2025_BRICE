#pragma once

#include "rev/SparkMax.h"

// Write ALL your robot-specific constants here
// Use namespacing to group constants together depending on the subsystems
// Example: namespace DriveConstants {}
// Example for variable : constexpr int kMOTOR_DRIVE = 0;
// Example for macro : #define SQUARE(x) x*x

namespace DeepClimbConstants {
    //Front motor
        constexpr int FRONT_MOTOR_ID = 9;
        constexpr rev::spark::SparkBaseConfig::IdleMode FRONT_MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        constexpr bool FRONT_MOTOR_INVERTED = false;
        constexpr int FRONT_MOTOR_CURRENT_LIMIT = 40;
        constexpr double FRONT_MOTOR_RAMP = 0.5;
        constexpr double FRONT_MOTOR_VOLTAGE_COMPENSATION = 10.0;

    //Back motor
        constexpr int BACK_MOTOR_ID = 1;
        constexpr rev::spark::SparkBaseConfig::IdleMode BACK_MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        constexpr bool BACK_MOTOR_INVERTED = false;
        constexpr int BACK_MOTOR_CURRENT_LIMIT = 40;
        constexpr double BACK_MOTOR_RAMP = 0.5;
        constexpr double BACK_MOTOR_VOLTAGE_COMPENSATION = 10.0;
        constexpr bool BACK_MOTOR_FOLLOW = false;
    
    //Encoder
        constexpr double ENCODER_A_ID = 0;
        constexpr double ENCODER_B_ID = 1;
        constexpr double DISTANCE_PER_PULSE = 360.0 * 1; //check the reduction

}
