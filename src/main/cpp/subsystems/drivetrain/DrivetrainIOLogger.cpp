#include "subsystems/drivetrain/DrivetrainIOLogger.h"

DrivetrainIOLogger::DrivetrainIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isFrontLeftMotorConnected(log, path + "/isFrontLeftMotorConnected"),
          isBackLeftMotorConnected(log, path + "/isBackLeftMotorConnected"),
          isFrontRightMotorConnected(log, path + "/isFrontRightMotorConnected"),
          isBackRightMotorConnected(log, path + "/isBackRightMotorConnected"),
          frontLeftMotorAppliedVoltage(log, path + "/frontLeftMotorAppliedVoltage"),
          frontLeftMotorBusVoltage(log, path + "/frontLeftMotorBusVoltage"),
          frontLeftMotorCurrent(log, path + "/frontLeftMotorCurrent"),
          frontLeftMotorTemperature(log, path + "/frontLeftMotorTemperature"),
          backLeftMotorAppliedVoltage(log, path + "/backLeftMotorAppliedVoltage"),
          backLeftMotorBusVoltage(log, path + "/backLeftMotorBusVoltage"),
          backLeftMotorCurrent(log, path + "/backLeftMotorCurrent"),
          backLeftMotorTemperature(log, path + "/backLeftMotorTemperature"),
          frontRightMotorAppliedVoltage(log, path + "/frontRightMotorAppliedVoltage"),
          frontRightMotorBusVoltage(log, path + "/frontRightMotorBusVoltage"),
          frontRightMotorCurrent(log, path + "/frontRightMotorCurrent"),
          frontRightMotorTemperature(log, path + "/frontRightMotorTemperature"),
          backRightMotorAppliedVoltage(log, path + "/backRightMotorAppliedVoltage"),
          backRightMotorBusVoltage(log, path + "/backRightMotorBusVoltage"),
          backRightMotorCurrent(log, path + "/backRightMotorCurrent"),
          backRightMotorTemperature(log, path + "/backRightMotorTemperature"),
          leftSideTraveledDistance(log, path + "/leftSideTraveledDistance"),
          leftSideVelocity(log, path + "/leftSideVelocity"),
          rightSideTraveledDistance(log, path + "/rightSideTraveledDistance"),
          rightSideVelocity(log, path + "/rightSideVelocity"),
          robotPosition(log, path + "/robotPosition")
{}

void DrivetrainIOLogger::Log(const DrivetrainIOInputs& inputs) {
    isFrontLeftMotorConnected.Append(inputs.isFrontLeftMotorConnected);
    isBackLeftMotorConnected.Append(inputs.isBackLeftMotorConnected);
    isFrontRightMotorConnected.Append(inputs.isFrontRightMotorConnected);
    isBackRightMotorConnected.Append(inputs.isBackRightMotorConnected);
    frontLeftMotorAppliedVoltage.Append(inputs.frontLeftMotorAppliedVoltage);
    frontLeftMotorBusVoltage.Append(inputs.frontLeftMotorBusVoltage);
    frontLeftMotorCurrent.Append(inputs.frontLeftMotorCurrent);
    frontLeftMotorTemperature.Append(inputs.frontLeftMotorTemperature);
    backLeftMotorAppliedVoltage.Append(inputs.backLeftMotorAppliedVoltage);
    backLeftMotorBusVoltage.Append(inputs.backLeftMotorBusVoltage);
    backLeftMotorCurrent.Append(inputs.backLeftMotorCurrent);
    backLeftMotorTemperature.Append(inputs.backLeftMotorTemperature);
    frontRightMotorAppliedVoltage.Append(inputs.frontRightMotorAppliedVoltage);
    frontRightMotorBusVoltage.Append(inputs.frontRightMotorBusVoltage);
    frontRightMotorCurrent.Append(inputs.frontRightMotorCurrent);
    frontRightMotorTemperature.Append(inputs.frontRightMotorTemperature);
    backRightMotorAppliedVoltage.Append(inputs.backRightMotorAppliedVoltage);
    backRightMotorBusVoltage.Append(inputs.backRightMotorBusVoltage);
    backRightMotorCurrent.Append(inputs.backRightMotorCurrent);
    backRightMotorTemperature.Append(inputs.backRightMotorTemperature);
    leftSideTraveledDistance.Append(inputs.leftSideTraveledDistance);
    leftSideVelocity.Append(inputs.leftSideVelocity);
    rightSideTraveledDistance.Append(inputs.rightSideTraveledDistance);
    rightSideVelocity.Append(inputs.rightSideVelocity);
    robotPosition.Append(inputs.robotPosition);
}