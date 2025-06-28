#pragma once

#include "Constants.h"
using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

namespace gripperConstants
{   
    constexpr ControlMode DefaultMode = ControlMode::DUTY_CYCLE;
    namespace IRbreaker
    {
        constexpr int DOWN_ID = 13;
        constexpr int UP_ID = 18;
        constexpr int UP2_ID = 19;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Counter 
    {
        constexpr int PRESHOOT = 10; //TUNEME
        constexpr int SHOOT = 20; //TUNEME
    }
    constexpr double OPEN_LOOP_REDUC = 2.0; //TUNEME

}

namespace feederConstants
{
    constexpr double GEAR_RATIO = (3.0*3.0);
    namespace Motor
    {
        constexpr int ID = 10;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.1; //TUNEME
        constexpr bool INVERTED = true;
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }
    namespace RPM 
    {
        constexpr double COLLECTING = 420; //TUNEME
        constexpr double REST = 0.0;
        constexpr double REJECTING_BACKWARD = -600; //TUNEME
        constexpr double REJECTING_FORWARD = 600; //TUNEME
    }
    namespace DutyCycle
    {
        constexpr double COLLECTING = 0.5553; //TUNEME
        constexpr double REST = 0.0;
        constexpr double REJECTING_BACKWARD = -0.3; //TUNEME
        constexpr double REJECTING_FORWARD = 0.3; //TUNEME
    }
    namespace VelocityPID
    {
        constexpr double KP = 0.5; //TUNEME
        constexpr double KI = 0.0; //TUNEME
        constexpr double KD = 0.0; //TUNEME
        constexpr double MAX = 1.0;
        constexpr double MIN = -1.0;
    }
}
namespace outtakeConstants
{
    constexpr double GEAR_RATIO = 3.0;
    namespace Motor
    {
        constexpr int ID = 9;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.1; //TUNEME
        constexpr bool INVERTED = false;
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }
    namespace RPM
    {
        constexpr double FEEDING_EMPTY = 600.0; //TUNEME
        constexpr double FEEDING_FORWARD = 450.0; //TUNEME
        constexpr double FEEDING_BACKWARD = -375.0; //TUNEME
        constexpr double SHY = 75.0; //TUNEME
        constexpr double PRESHOOT = -300.0; //TUNEME
        constexpr double REJECTING_BACKWARD = -450.0; //TUNEME
        constexpr double REJECTING_FORWARD = 450.0; //TUNEME
        constexpr double SHIFTING = 300.0; //TUNEME
        constexpr double REST = 0.0;
        constexpr double HIGH_SHOOTING = 600.0; //TUNEME
        constexpr double MIDDLE_SHOOTING = 645.0; //TUNEME
        constexpr double LOW_SHOOTING = 150.0; //TUNEME
    }
    namespace DutyCycle
    {
        constexpr double FEEDING_EMPTY = 0.4; //TUNEME
        constexpr double FEEDING_FORWARD = 0.3; //TUNEME
        constexpr double FEEDING_BACKWARD = -0.25; //TUNEME
        constexpr double SHY = 0.05; //TUNEME
        constexpr double PRESHOOT = -0.2; //TUNEME
        constexpr double REJECTING_BACKWARD = -0.3; //TUNEME
        constexpr double REJECTING_FORWARD = 0.3; //TUNEME
        constexpr double SHIFTING = 0.2; //TUNEME
        constexpr double REST = 0.0;
        constexpr double HIGH_SHOOTING = 0.4; //TUNEME
        constexpr double MIDDLE_SHOOTING = 0.43; //TUNEME
        constexpr double LOW_SHOOTING = 0.1; //TUNEME
    }
    namespace VelocityPID
    {
        constexpr double KP = 0.5; //TUNEME
        constexpr double KI = 0.0; //TUNEME
        constexpr double KD = 0.0; //TUNEME
        constexpr double MAX = 1.0;
        constexpr double MIN = -1.0;
    }
}