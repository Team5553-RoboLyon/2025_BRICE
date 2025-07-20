#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "GripperIO.h"

class GripperIOLogger {
public:
    GripperIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const GripperIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isFeederMotorConnected;
    wpi::log::BooleanLogEntry isOuttakeMotorConnected;
    wpi::log::DoubleLogEntry feederAppliedVoltage;
    wpi::log::DoubleLogEntry feederBusVoltage;
    wpi::log::DoubleLogEntry feederCurrent;
    wpi::log::DoubleLogEntry feederTemperature;
    wpi::log::DoubleLogEntry feederRPM;
    wpi::log::DoubleLogEntry outtakeAppliedVoltage;
    wpi::log::DoubleLogEntry outtakeBusVoltage;
    wpi::log::DoubleLogEntry outtakeCurrent;
    wpi::log::DoubleLogEntry outtakeTemperature;
    wpi::log::DoubleLogEntry outtakeRPM;
    wpi::log::BooleanLogEntry IRBreakerUp;
    wpi::log::BooleanLogEntry IRBreakerUp2;
    wpi::log::BooleanLogEntry IRBreakerDown;
};