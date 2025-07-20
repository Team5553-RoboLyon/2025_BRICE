#include "subsystems/straffer/StrafferIOLogger.h"

StrafferIOLogger::StrafferIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isMotorConnected(log, path + "/isMotorConnected"),
          appliedVoltage(log, path + "/appliedVoltage"),
          busVoltage(log, path + "/busVoltage"),
          current(log, path + "/current"),
          temperature(log, path + "/temperature"),
          limitSwitchLeft(log, path + "/limitSwitchLeft"),
          limitSwitchRight(log, path + "/limitSwitchRight"),
          widthPosition(log, path + "/widthPosition")
{}

void StrafferIOLogger::Log(const StrafferIOInputs& inputs) {
    isMotorConnected.Append(inputs.isMotorConnected);
    appliedVoltage.Append(inputs.appliedVoltage);
    busVoltage.Append(inputs.busVoltage);
    current.Append(inputs.current);
    temperature.Append(inputs.temperature);
    limitSwitchLeft.Append(inputs.limitSwitchLeft);
    limitSwitchRight.Append(inputs.limitSwitchRight);
    widthPosition.Append(inputs.widthPosition);
}