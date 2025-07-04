#pragma once
#include "rev/SparkMax.h"
#include "lib/UtilsRBL.h"
#include "lib/DebugUtils.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BRICE_COMPETITION 0
#define BRICE_TRAINING 1
#define BABY_BRICE 2
// #define T_NOR 3 TODO
#define SIMULATION 4

#define ROBOT_MODEL (BRICE_TRAINING) // Change this to the desired robot model
#if (ROBOT_MODEL != (BRICE_COMPETITION))
#define DEBUG_MODE
#endif

constexpr double ENCODER_TICKS_PER_REVOLUTION_K2X = 2048.0;
constexpr double TIME_PER_CYCLE = 0.02; // 20ms



enum class ControlMode {
    PROFILED_PID, //Motion profiling + PID sur l'output
    MOTION_PROFILING, 
    POSITION_PID, 
    VELOCITY, // RPM ou RPS
    VOLTAGE, // volts (-12V à 12V)
    DUTY_CYCLE, // percentage (-1 to 1)
    OPEN_LOOP // bypass the StateMachine
};
#define ALLOWS_STATE_MACHINE(mode) ((mode) != (ControlMode::OPEN_LOOP))



namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int COPILOT_CONTROLLER_ID = 2;
    }
    namespace Button {
        // FORWARD Joystick
        constexpr int REVERSED_DRIVE_BUTTON = 1;
        // ROTATION Joystick
        constexpr int SLOW_DRIVE_BUTTON = 1;
    }
    namespace Settings{
        constexpr double SLOW_RATE = 2.0;
        constexpr double DEADBAND = 0.05;
        constexpr double TIME_TO_REACH_FULL_FORWARD = 0.8;
        constexpr double TIME_TO_REACH_FULL_ROTATION = 0.5;
        constexpr double DEADBAND_OPEN_LOOP = 0.1;
        constexpr double MIN_MOVING_V = 0.07;
        constexpr double MIN_MOVING_W = 0.25;
    }
}