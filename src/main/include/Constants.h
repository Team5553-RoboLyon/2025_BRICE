#pragma once
#include "rev/SparkMax.h"
#include "lib/UtilsRBL.h"
#include "frc/DriverStation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BRICE_COMPETITION 0
#define BRICE_TRAINING 1
#define BABY_BRICE 2
// #define T_NOR 3 TODO
#define SIMULATION 4

#define ADAM 0
#define VICTOR 1
#define ALEXIS 2
#define TEST 3

#define ROBOT_MODEL (BRICE_TRAINING) // Change this to the desired robot model
#define PILOT (ALEXIS)
#define OPERATOR (ADAM)

#if (ROBOT_MODEL != (BRICE_COMPETITION))
#define DEBUG_MODE
#endif


constexpr double ENCODER_TICKS_PER_REVOLUTION_K2X = 2048.0;
constexpr double TIME_PER_CYCLE = 0.02; // 20ms


#define IS_RED_ALLIANCE(alliance) ((alliance) == (frc::DriverStation::Alliance::kRed))
#define IS_BLUE_ALLIANCE(alliance) ((alliance) == (frc::DriverStation::Alliance::kBlue))

namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int COPILOT_CONTROLLER_ID = 2;
    }
}