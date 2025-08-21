/*******************************************************************************
 * 
 * File        : DebugUtils.h (v1.2)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Debugging utilities for logging and assertions.
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <iostream>
#include "frc/Errors.h"
#include "Constants.h"

// write #define DEBUG_MODE to enable debug mode in Robot.h ( NEVER IN COMPETITION MODE )
#ifdef DEBUG_MODE
    #include <cassert>
    #define DEBUG_ASSERT(cond, msg) (assert((cond) && (msg))) // activate with false
    #define DEBUG_LOG(msg) \
        (std::cout << "[DEBUG] " << (msg) << "\n")
#else
    #define DEBUG_ASSERT(cond, msg) ( \
        (std::cout << "[ASSERT] " << (msg) << "\n"))
    #define DEBUG_LOG(msg) ((void)0)
#endif

#define ERROR_LOG(msg) FRC_ReportError(frc::err::Error, (msg))
#define WARNING_LOG(msg) FRC_ReportError(frc::warn::Warning, (msg))
#define INFO_LOG(msg) (std::cout << "[INFO] " << (msg) << "\n")