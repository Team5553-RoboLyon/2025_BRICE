#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpi/DataLog.h>
#include "DrivetrainIO.h"

class DrivetrainIOLogger {
public:
    DrivetrainIOLogger(wpi::log::DataLog& log, const std::string& path);
    void Log(const DrivetrainIOInputs& inputs);

private:
    wpi::log::BooleanLogEntry isFrontLeftMotorConnected;
    wpi::log::BooleanLogEntry isBackLeftMotorConnected;
    wpi::log::BooleanLogEntry isFrontRightMotorConnected;
    wpi::log::BooleanLogEntry isBackRightMotorConnected;
    wpi::log::DoubleLogEntry frontLeftMotorAppliedVoltage;
    wpi::log::DoubleLogEntry frontLeftMotorBusVoltage;
    wpi::log::DoubleLogEntry frontLeftMotorCurrent;
    wpi::log::DoubleLogEntry frontLeftMotorTemperature;
    wpi::log::DoubleLogEntry backLeftMotorAppliedVoltage;
    wpi::log::DoubleLogEntry backLeftMotorBusVoltage;
    wpi::log::DoubleLogEntry backLeftMotorCurrent;
    wpi::log::DoubleLogEntry backLeftMotorTemperature;
    wpi::log::DoubleLogEntry frontRightMotorAppliedVoltage;
    wpi::log::DoubleLogEntry frontRightMotorBusVoltage;
    wpi::log::DoubleLogEntry frontRightMotorCurrent;
    wpi::log::DoubleLogEntry frontRightMotorTemperature;
    wpi::log::DoubleLogEntry backRightMotorAppliedVoltage;
    wpi::log::DoubleLogEntry backRightMotorBusVoltage;
    wpi::log::DoubleLogEntry backRightMotorCurrent;
    wpi::log::DoubleLogEntry backRightMotorTemperature;
    wpi::log::DoubleLogEntry leftSideTraveledDistance;
    wpi::log::DoubleLogEntry leftSideVelocity;
    wpi::log::DoubleLogEntry rightSideTraveledDistance;
    wpi::log::DoubleLogEntry rightSideVelocity;

    wpi::log::StructLogEntry<frc::Pose2d> robotPosition;
};