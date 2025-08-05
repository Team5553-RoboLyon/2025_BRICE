/*******************************************************************************
 * 
 * File        : ControlMode.h (v1.2)
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
    // ----- StateMachine-enabled modes (PID loops or motion profiling) -----
    PROFILED_PID,               // Motion profiling combined with PID control
    MOTION_PROFILING,           // Basic motion profiling without PID

    POSITION_VOLTAGE_PID,       // Position PID controller with output applied as voltage (Volts)
    POSITION_DUTYCYCLE_PID,     // Position PID controller with output applied as duty cycle (%)

    VELOCITY_VOLTAGE_PID,       // Velocity PID controller with output applied as voltage (Volts)
    VELOCITY_DUTYCYCLE_PID,     // Velocity PID controller with output applied as duty cycle (%)

    VELOCITY_VOLTAGE,           // Open-loop velocity control based on a feedforward model: (velocity / maxVelocity) * maxVoltage
    VELOCITY_DUTY_CYCLE,        // Open-loop velocity control using feedforward: (velocity / maxVelocity) as a duty cycle
    
    VOLTAGE,                   // Direct voltage control (open loop)
    DUTY_CYCLE,                // Direct duty cycle control (open loop)

    // ----- Manual / Bypass StateMachine modes -----
    MANUAL_SETPOINT,           // Manual setpoint control (with PID)
    MANUAL_VOLTAGE,            // Manual direct voltage control
    MANUAL_VELOCITY,           // Manual velocity control (in volts with PID)
    MANUAL_DUTY_CYCLE,         // Manual direct duty cycle control

    // ----- Disabled mode -----
    DISABLED,                  // Controller disabled, no output

    // ----- Future advanced control modes -----
    // ENERGY_MODEL,
};

#define ALLOWS_STATE_MACHINE(mode) ((mode) == ControlMode::PROFILED_PID || \
                                    (mode) == ControlMode::MOTION_PROFILING || \
                                    (mode) == ControlMode::POSITION_VOLTAGE_PID || \
                                    (mode) == ControlMode::POSITION_DUTYCYCLE_PID || \
                                    (mode) == ControlMode::VELOCITY_VOLTAGE_PID || \
                                    (mode) == ControlMode::VELOCITY_DUTYCYCLE_PID || \
                                    (mode) == ControlMode::VELOCITY_DUTY_CYCLE || \
                                    (mode) == ControlMode::VELOCITY_VOLTAGE || \
                                    (mode) == ControlMode::VOLTAGE || \
                                    (mode) == ControlMode::DUTY_CYCLE)

#define BYPASS_STATE_MACHINE(mode) ((mode) == ControlMode::MANUAL_SETPOINT || \
                                    (mode) == ControlMode::MANUAL_VOLTAGE || \
                                    (mode) == ControlMode::MANUAL_VELOCITY || \
                                    (mode) == ControlMode::MANUAL_DUTY_CYCLE || \
                                    (mode) == ControlMode::DISABLED)

#define IS_PID(mode) ((mode) == ControlMode::PROFILED_PID || \
                        (mode) == ControlMode::POSITION_VOLTAGE_PID || \
                        (mode) == ControlMode::POSITION_DUTYCYCLE_PID || \
                        (mode) == ControlMode::VELOCITY_VOLTAGE_PID || \
                        (mode) == ControlMode::VELOCITY_DUTYCYCLE_PID || \
                        (mode) == ControlMode::MANUAL_SETPOINT || \
                        (mode) == ControlMode::MANUAL_VELOCITY)

#define IS_PROFILING(mode) ((mode) == ControlMode::PROFILED_PID || \
                            (mode) == ControlMode::MOTION_PROFILING)

#define IS_VOLTAGE_OUTPUT_MODE(mode) ((mode) == ControlMode::POSITION_VOLTAGE_PID || \
                                      (mode) == ControlMode::VELOCITY_VOLTAGE_PID || \
                                      (mode) == ControlMode::VOLTAGE || \
                                      (mode) == ControlMode::VELOCITY_VOLTAGE || \
                                      (mode) == ControlMode::MANUAL_VOLTAGE)
            
#define IS_DUTYCYCLE_OUTPUT_MODE(mode) ((mode) == ControlMode::POSITION_DUTYCYCLE_PID || \
                                        (mode) == ControlMode::VELOCITY_DUTYCYCLE_PID || \
                                        (mode) == ControlMode::VELOCITY_DUTY_CYCLE|| \
                                        (mode) == ControlMode::DUTY_CYCLE || \
                                        (mode) == ControlMode::MANUAL_DUTY_CYCLE)

#define IS_DISABLED_MODE(mode) ((mode) == ControlMode::DISABLED)

inline const char* ToString(const ControlMode mode) {
    switch (mode) {
        case ControlMode::PROFILED_PID:           return "PROFILED_PID";
        case ControlMode::MOTION_PROFILING:       return "MOTION_PROFILING";
        case ControlMode::POSITION_VOLTAGE_PID:   return "POSITION_VOLTAGE_PID";
        case ControlMode::POSITION_DUTYCYCLE_PID: return "POSITION_DUTYCYCLE_PID";
        case ControlMode::VELOCITY_VOLTAGE_PID:   return "VELOCITY_VOLTAGE_PID";
        case ControlMode::VELOCITY_DUTYCYCLE_PID: return "VELOCITY_DUTYCYCLE_PID";
        case ControlMode::VELOCITY_DUTY_CYCLE:    return "VELOCITY_DUTY_CYCLE";
        case ControlMode::VELOCITY_VOLTAGE:       return "VELOCITY_VOLTAGE";
        case ControlMode::VOLTAGE:                 return "VOLTAGE";
        case ControlMode::DUTY_CYCLE:              return "DUTY_CYCLE";
        case ControlMode::MANUAL_SETPOINT:         return "MANUAL_SETPOINT";
        case ControlMode::MANUAL_VOLTAGE:           return "MANUAL_VOLTAGE";
        case ControlMode::MANUAL_VELOCITY:          return "MANUAL_VELOCITY";
        case ControlMode::MANUAL_DUTY_CYCLE:        return "MANUAL_DUTY_CYCLE";
        case ControlMode::DISABLED:                  return "DISABLED";
        default:                                    return "UNKNOWN";
    }
}