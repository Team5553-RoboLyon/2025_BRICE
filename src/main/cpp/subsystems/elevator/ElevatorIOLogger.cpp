#include "subsystems/elevator/ElevatorIOLogger.h"

ElevatorIOLogger::ElevatorIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isLeftMotorConnected(log, path + "/isLeftMotorConnected"),
          isRightMotorConnected(log, path + "/isRightMotorConnected"),
          leftMotorAppliedVoltage(log, path + "/leftMotorAppliedVoltage"),
          leftMotorBusVoltage(log, path + "/leftMotorBusVoltage"),
          leftMotorCurrent(log, path + "/leftMotorCurrent"),
          leftMotorTemperature(log, path + "/leftMotorTemperature"),
          rightMotorAppliedVoltage(log, path + "/rightMotorAppliedVoltage"),
          rightMotorBusVoltage(log, path + "/rightMotorBusVoltage"),
          rightMotorCurrent(log, path + "/rightMotorCurrent"),
          rightMotorTemperature(log, path + "/rightMotorTemperature"),
          limitSwitchBottom(log, path + "/limitSwitchBottom"),
          limitSwitchBottom2(log, path + "/limitSwitchBottom2"),
          heightPosition(log, path + "/heightPosition")
{}

void ElevatorIOLogger::Log(const ElevatorIOInputs& inputs) {
    isLeftMotorConnected.Append(inputs.isLeftMotorConnected);
    isRightMotorConnected.Append(inputs.isRightMotorConnected);
    leftMotorAppliedVoltage.Append(inputs.leftMotorAppliedVoltage);
    leftMotorBusVoltage.Append(inputs.leftMotorBusVoltage);
    leftMotorCurrent.Append(inputs.leftMotorCurrent);
    leftMotorTemperature.Append(inputs.leftMotorTemperature);
    rightMotorAppliedVoltage.Append(inputs.rightMotorAppliedVoltage);
    rightMotorBusVoltage.Append(inputs.rightMotorBusVoltage);
    rightMotorCurrent.Append(inputs.rightMotorCurrent);
    rightMotorTemperature.Append(inputs.rightMotorTemperature);
    limitSwitchBottom.Append(inputs.limitSwitchBottom);
    limitSwitchBottom2.Append(inputs.limitSwitchBottom2);
    heightPosition.Append(inputs.heightPosition);
}