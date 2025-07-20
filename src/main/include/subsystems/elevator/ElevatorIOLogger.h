#pragma once

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "ElevatorIO.h"

class ElevatorIOLogger {
public:
    ElevatorIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const ElevatorIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isLeftMotorConnected;
    wpi::log::BooleanLogEntry isRightMotorConnected;
    wpi::log::DoubleLogEntry leftMotorAppliedVoltage;
    wpi::log::DoubleLogEntry leftMotorBusVoltage;
    wpi::log::DoubleLogEntry leftMotorCurrent;
    wpi::log::DoubleLogEntry leftMotorTemperature;
    wpi::log::DoubleLogEntry rightMotorAppliedVoltage;
    wpi::log::DoubleLogEntry rightMotorBusVoltage;
    wpi::log::DoubleLogEntry rightMotorCurrent;
    wpi::log::DoubleLogEntry rightMotorTemperature;
    wpi::log::BooleanLogEntry limitSwitchBottom;
    wpi::log::BooleanLogEntry limitSwitchBottom2;
    wpi::log::DoubleLogEntry heightPosition;
};