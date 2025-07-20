#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "StrafferIO.h"

class StrafferIOLogger {
public:
    StrafferIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const StrafferIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isMotorConnected;
    wpi::log::DoubleLogEntry appliedVoltage;
    wpi::log::DoubleLogEntry busVoltage;
    wpi::log::DoubleLogEntry current;
    wpi::log::DoubleLogEntry temperature;
    wpi::log::BooleanLogEntry limitSwitchLeft;
    wpi::log::BooleanLogEntry limitSwitchRight;
    wpi::log::DoubleLogEntry widthPosition;
};