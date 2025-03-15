#pragma once
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

namespace ManipulatorConstants
{
    namespace Elevator {
        namespace Motors{
            constexpr int ID = 8;
            constexpr double VOLTAGE_COMPENSATION = 12.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.0;
            constexpr bool INVERTED = true;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
        namespace Sensor {
            namespace Encoder {
                constexpr int A_ID = 6;
                constexpr int B_ID = 7;
                constexpr bool REVERSED = true;
                constexpr double REDUCTION = 30.0/26.0;
                constexpr double RADIUS = (0.0360*3.0/M_PI)/2.0;
                constexpr double DISTANCE_PER_PULSE = (2.0 * M_PI * RADIUS) / REDUCTION / ENCODER_TICKS_PER_REVOLUTION;
            }
            namespace LimitSwitch {
                constexpr int BOTTOM_ID = 11;
                constexpr int TOP_ID = 12;
                constexpr bool IS_TRIGGERED = true;
            }
        }
        // namespace PID {
        //     constexpr double KP = 8.0;
        //     constexpr double KI = 0.0;
        //     constexpr double KD = 0.00;
        //     constexpr double TOLERANCE = 0.001;
        //     constexpr double SETPOINT = 0.0;
        // }
        namespace Speed {
            constexpr double UP = 0.4;
            constexpr double DOWN = -0.4;
            constexpr double CALIBRATION = -0.3;
            constexpr double REST = 0.0;
        }
        namespace State {
            constexpr u_int8_t L2 = 0;
            constexpr u_int8_t L2_CORALSTATION = 1;
            constexpr u_int8_t CORALSTATION = 2;
            constexpr u_int8_t CORALSTATION_L3 = 3;
            constexpr u_int8_t L3 = 4;
            constexpr u_int8_t L3_L4 = 5;
            constexpr u_int8_t L4 = 6;

            constexpr u_int8_t REST = 0;
            constexpr u_int8_t UP = 1;
            constexpr u_int8_t DOWN = 2;
            constexpr u_int8_t WAITING = 3;
        }

    }
    namespace Planetary {
        namespace Motors {
            constexpr int ID = 9;
            constexpr double VOLTAGE_COMPENSATION = 10.0;
            constexpr double CURRENT_LIMIT = 40.0;
            constexpr double RAMP_RATE = 0.1;
            constexpr bool INVERTED = true;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
        }
        namespace Sensor {
            namespace Encoder {
                constexpr int A_ID = 8;
                constexpr int B_ID = 9;
                constexpr bool REVERSED = false; 
                constexpr double REDUCTION = -4.0;
                constexpr double DISTANCE_PER_PULSE = NF64_2PI / REDUCTION / ENCODER_TICKS_PER_REVOLUTION;
            }
        }
        namespace PID {
            constexpr double KP = 1.25;
            constexpr double KI = 0.0;
            constexpr double KD = 0.00;
            constexpr double TOLERANCE = 0.02;
            constexpr double SETPOINT = 0.0;
        }
        namespace Speed {
            constexpr double MAX = 0.3;
            constexpr double MIN = -0.3;
            constexpr double REST = 0.0;
        }
        namespace State {
            constexpr u_int8_t HOME = 0;              
            constexpr u_int8_t HOME_CORALSTATION = 1;  
            constexpr u_int8_t CORALSTATION = 2;       
            constexpr u_int8_t CORALSTATION_L4 = 3;    
            constexpr u_int8_t L4 = 4;                 
            constexpr u_int8_t L4_L3 = 5;              
            constexpr u_int8_t L3 = 6;                 
            constexpr u_int8_t L3_L2 = 7;              
            constexpr u_int8_t L2 = 8;                 
            constexpr u_int8_t REST = 0;
            constexpr u_int8_t CLOCKWISE = 1;
            constexpr u_int8_t COUNTER_CLOCKWISE = 2;
            constexpr u_int8_t WAITING = 3;
        }
    }
    namespace mask {
        constexpr uint16_t maskElevatorPosition = 0xF;             // 0000 0000 0000 1111
        constexpr uint16_t maskElevatorMovingType = 0x30;          // 0000 0000 0011 0000
        constexpr uint16_t maskPlanetaryPosition = 0x3C0;          // 0000 0011 1100 0000
        constexpr uint16_t maskPlanetaryMovingType = 0xC00;        // 0000 1100 0000 0000
        constexpr uint16_t maskGripperState = 0xF000;              // 1111 0000 0000 0000
        constexpr uint16_t maskElevatorState = 0x3F;               // 0000 0000 0011 1111
        constexpr uint16_t maskPlanetaryState = 0xFC0;             // 0000 1111 1100 0000
        constexpr uint16_t maskState = 0xFFFF;                     // 1111 1111 1111 1111
    }
    enum class State {
        Home =0,
        L2 =1,
        CoralStation =2,
        L3 =3,
        L4 =4
    };
}

namespace GrippperConstants {
    namespace Motors {
        constexpr int ID_TOP = 10;
        constexpr int ID_BOTTOM = 11;
        constexpr double VOLTAGE_COMPENSATION = 12.0;   
        constexpr double CURRENT_LIMIT = 20.0;
        constexpr double RAMP_RATE = 0.1;
        constexpr bool INVERTED = true;
        constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
    }
    namespace Sensor {
        constexpr int IR_BREAKER_ID = 10;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Speed {
        constexpr double CATCH = -1.0;
        constexpr double DROP = 1.0;
        constexpr double REST = 0.0;
        // constexpr double MOVE_TO_BACK = -0.5;
        // constexpr double MOVE_TO_MIDDLE = 0.5;
    }
}

namespace DriveConstants {

    namespace LeftGearbox{
        namespace Motor{
            constexpr int FRONT_MOTOR_ID = 2;
            constexpr int BACK_MOTOR_ID = 3;
            constexpr rev::spark::SparkBaseConfig::IdleMode MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool MOTOR_INVERTED = false;
            constexpr int MOTOR_CURRENT_LIMIT = 40;
            constexpr double MOTOR_RAMP = 0.1;
            constexpr double MOTOR_VOLTAGE_COMPENSATION = 12.0;
        }

        namespace Encoder{
            constexpr int ID_ENCODER_A = 0;
            constexpr int ID_ENCODER_B = 1;
            constexpr bool REVERSE_ENCODER = true; 
            constexpr double DISTANCE_PER_PULSE = 1.0/ENCODER_TICKS_PER_REVOLUTION;
        }
        constexpr bool WHEEL_SIDE = true;
    }

    namespace RightGearbox{
        namespace Motor{
            constexpr int FRONT_MOTOR_ID = 4;
            constexpr int BACK_MOTOR_ID = 5;
            constexpr rev::spark::SparkBaseConfig::IdleMode MOTOR_IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool MOTOR_INVERTED = true;
            constexpr int MOTOR_CURRENT_LIMIT = 40;
            constexpr double MOTOR_RAMP = 0.1;
            constexpr double MOTOR_VOLTAGE_COMPENSATION = 12.0;
        }

        namespace Encoder{
            constexpr int ID_ENCODER_A = 2;
            constexpr int ID_ENCODER_B = 3;
            constexpr bool REVERSE_ENCODER = false;
            constexpr double DISTANCE_PER_PULSE = 1.0/ENCODER_TICKS_PER_REVOLUTION; // TODO : set appropriate distance
        }
        constexpr bool WHEEL_SIDE = false;
    }
}

namespace DeepClimbConstants {

    constexpr double REDUCTION = 10.0/3.0;

    namespace Motors {
            constexpr int ID_FRONT = 6;
            constexpr int ID_BACK = 7;
            constexpr rev::spark::SparkBaseConfig::IdleMode IDLE_MODE = rev::spark::SparkBaseConfig::IdleMode::kBrake;
            constexpr bool INVERTED = false;
            constexpr int CURRENT_LIMIT = 40;
            constexpr double RAMP = 0.5;
            constexpr double VOLTAGE_COMPENSATION = 10.0; //12
    }
    namespace Encoder {
        constexpr int ENCODER_A_ID = 2;
        constexpr int ENCODER_B_ID = 3;
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

namespace ControlPanelConstants {
    namespace Joystick{
        constexpr int FORWARD_ID = 0;
        constexpr int ROTATION_ID = 1;
        constexpr int XBOX_CONTROLLER_ID = 2;
    }
    namespace Button {
        // FORWARD Joystick
        constexpr int REVERSED_DRIVE_BUTTON = 1;
        // ROTATION Joystick
        constexpr int SLOW_DRIVE_BUTTON = 1;
        // XBOX_CONTROLLER 
        constexpr int DROP = 2;
        constexpr int CATCH = 1;
    }
    namespace Settings{
        constexpr double SLOW_RATE = 2.0;
        constexpr double DEADBAND = 0.17;
        constexpr double RATE_LIMITER_FOWARD = TIME_TO_REACH_MAX(0.45);
        constexpr double RATE_LIMITER_ROTATION = TIME_TO_REACH_MAX(0.5);
    }
}
