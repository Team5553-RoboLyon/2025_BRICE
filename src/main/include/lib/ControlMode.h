/*******************************************************************************
 * 
 * File        : ControlMode.h (v1.0)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Defines various control modes used in the robot's state machine 
 *               and manual control.
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

#pragma once
#include <string>

// It's recommended to use a StateMachine-enabled mode for Main Control Mode
// and a Bypass StateMachine mode for Emergency Mode
enum class ControlMode {
    // ----- StateMachine-enabled modes -----
    PROFILED_PID,       // Motion profiling + output PID
    MOTION_PROFILING,   // Motion profiling without PID 
    POSITION_PID,       // setpoint PID
    VELOCITY_PID,       // RPM or RPS
    VOLTAGE,            // Volts (-12V to 12V)
    DUTY_CYCLE,         // Pourcentage (-1.0 to 1.0)

    // ----- Manual / Bypass StateMachine -----
    MANUAL_SETPOINT,
    MANUAL_VOLTAGE,     
    MANUAL_VELOCITY,    
    MANUAL_DUTY_CYCLE

    // ----- Futur advanced modes -----
    // ENERGY_MODEL
};
#define ALLOWS_STATE_MACHINE(mode) ((mode) == ControlMode::PROFILED_PID || \
                                    (mode) == ControlMode::MOTION_PROFILING || \
                                    (mode) == ControlMode::POSITION_PID || \
                                    (mode) == ControlMode::VELOCITY_PID || \
                                    (mode) == ControlMode::VOLTAGE || \
                                    (mode) == ControlMode::DUTY_CYCLE)
#define BYPASS_STATE_MACHINE(mode) ((mode) == ControlMode::MANUAL_SETPOINT || \
                                    (mode) == ControlMode::MANUAL_VOLTAGE || \
                                    (mode) == ControlMode::MANUAL_VELOCITY || \
                                    (mode) == ControlMode::MANUAL_DUTY_CYCLE)

inline const char* ToString(ControlMode mode) {
    switch (mode) {
        case ControlMode::PROFILED_PID: return "PROFILED_PID";
        case ControlMode::MOTION_PROFILING: return "MOTION_PROFILING";
        case ControlMode::POSITION_PID: return "POSITION_PID";
        case ControlMode::VELOCITY_PID: return "VELOCITY_PID";
        case ControlMode::VOLTAGE: return "VOLTAGE";
        case ControlMode::DUTY_CYCLE: return "DUTY_CYCLE";
        case ControlMode::MANUAL_SETPOINT: return "MANUAL_SETPOINT";
        case ControlMode::MANUAL_VOLTAGE: return "MANUAL_VOLTAGE";
        case ControlMode::MANUAL_VELOCITY: return "MANUAL_VELOCITY";
        case ControlMode::MANUAL_DUTY_CYCLE: return "MANUAL_DUTY_CYCLE";
        default: return "UNKNOWN";
    }
}