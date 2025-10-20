/*******************************************************************************
 * 
 * File        : ControlMode.h (v1.3)
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
    // ----- High-level control -----
    PROFILED_PID,               // Motion profiling + PID (trapezoidal, S-curve)
    MOTION_PROFILING,           // Feedforward motion profiling only (open-loop trajectory)

    // ----- Closed-loop control (PID + optional feedforward) -----
    POSITION_VOLTAGE_PID,       // Position control (PID + volts output)
    POSITION_DUTYCYCLE_PID,     // Position control (PID + duty cycle output)
    VELOCITY_VOLTAGE_PID,       // Velocity control (PID + volts output)
    VELOCITY_DUTYCYCLE_PID,     // Velocity control (PID + duty cycle output)
    MODEL_CONTROLLED,           // Model-based control (dynamic system model + PID feedback) 
    // ----- Open-loop control (feedforward or direct) -----
    VELOCITY_VOLTAGE_FF,        // Open-loop velocity control (kS/kV/kA model, output in volts)
    VELOCITY_DUTYCYCLE_FF,      // Open-loop velocity control (kS/kV/kA model, duty cycle output)
    VOLTAGE,                    // Direct voltage control
    DUTY_CYCLE,                 // Direct duty cycle control
    CURRENT,                    // Current control (amps or % of max amps, if stable model available)
    TORQUE,                     // Torque control (if stable model available)

    // ----- Manual / Bypass modes (no state machine) -----
    MANUAL_POSITION,            // Manual position or velocity command with PID
    MANUAL_VOLTAGE,             // Manual voltage command
    MANUAL_VELOCITY,            // Manual velocity command (PID or open-loop)
    MANUAL_DUTY_CYCLE,          // Manual duty cycle command

    // ----- Disabled / Safe mode -----
    DISABLED                   // Controller output disabled

    // ----- Future advanced control modes -----
    // ENERGY_MODEL, 
    // STATE_SPACE,                // Full state-space or model-based control (with observers)
};

#define ALLOWS_STATE_MACHINE(mode) ((mode) == ControlMode::PROFILED_PID || \
                                    (mode) == ControlMode::MOTION_PROFILING || \
                                    (mode) == ControlMode::POSITION_VOLTAGE_PID || \
                                    (mode) == ControlMode::POSITION_DUTYCYCLE_PID || \
                                    (mode) == ControlMode::VELOCITY_VOLTAGE_PID || \
                                    (mode) == ControlMode::VELOCITY_DUTYCYCLE_PID || \
                                    (mode) == ControlMode::MODEL_CONTROLLED || \
                                    (mode) == ControlMode::VELOCITY_VOLTAGE_FF || \
                                    (mode) == ControlMode::VELOCITY_DUTYCYCLE_FF || \
                                    (mode) == ControlMode::VOLTAGE || \
                                    (mode) == ControlMode::CURRENT || \
                                    (mode) == ControlMode::TORQUE || \
                                    (mode) == ControlMode::DUTY_CYCLE)

#define BYPASS_STATE_MACHINE(mode) ((mode) == ControlMode::MANUAL_POSITION || \
                                    (mode) == ControlMode::MANUAL_VOLTAGE || \
                                    (mode) == ControlMode::MANUAL_VELOCITY || \
                                    (mode) == ControlMode::MANUAL_DUTY_CYCLE || \
                                    (mode) == ControlMode::DISABLED)

#define IS_PID(mode) ((mode) == ControlMode::PROFILED_PID || \
                        (mode) == ControlMode::POSITION_VOLTAGE_PID || \
                        (mode) == ControlMode::POSITION_DUTYCYCLE_PID || \
                        (mode) == ControlMode::VELOCITY_VOLTAGE_PID || \
                        (mode) == ControlMode::VELOCITY_DUTYCYCLE_PID || \
                        (mode) == ControlMode::MANUAL_POSITION || \
                        (mode) == ControlMode::MODEL_CONTROLLED || \
                        (mode) == ControlMode::MANUAL_VELOCITY)

#define IS_PROFILING(mode) ((mode) == ControlMode::PROFILED_PID || \
                            (mode) == ControlMode::MOTION_PROFILING)

#define IS_VOLTAGE_OUTPUT_MODE(mode) ((mode) == ControlMode::POSITION_VOLTAGE_PID || \
                                      (mode) == ControlMode::VELOCITY_VOLTAGE_PID || \
                                      (mode) == ControlMode::VOLTAGE || \
                                      (mode) == ControlMode::VELOCITY_VOLTAGE_FF || \
                                      (mode) == ControlMode::MANUAL_VOLTAGE)
            
#define IS_DUTYCYCLE_OUTPUT_MODE(mode) ((mode) == ControlMode::POSITION_DUTYCYCLE_PID || \
                                        (mode) == ControlMode::VELOCITY_DUTYCYCLE_PID || \
                                        (mode) == ControlMode::VELOCITY_DUTYCYCLE_FF|| \
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
        case ControlMode::MODEL_CONTROLLED:       return "MODEL_CONTROLLED";
        case ControlMode::VELOCITY_VOLTAGE_FF:    return "VELOCITY_VOLTAGE_FF";
        case ControlMode::VELOCITY_DUTYCYCLE_FF:  return "VELOCITY_DUTYCYCLE_FF";
        case ControlMode::VOLTAGE:                return "VOLTAGE";
        case ControlMode::DUTY_CYCLE:              return "DUTY_CYCLE";
        case ControlMode::CURRENT:                  return "CURRENT";
        case ControlMode::TORQUE:                   return "TORQUE";
        case ControlMode::MANUAL_POSITION:         return "MANUAL_POSITION";
        case ControlMode::MANUAL_VOLTAGE:           return "MANUAL_VOLTAGE";
        case ControlMode::MANUAL_VELOCITY:          return "MANUAL_VELOCITY";
        case ControlMode::MANUAL_DUTY_CYCLE:        return "MANUAL_DUTY_CYCLE";
        case ControlMode::DISABLED:                  return "DISABLED";
        default:                                    return "UNKNOWN";
    }
}