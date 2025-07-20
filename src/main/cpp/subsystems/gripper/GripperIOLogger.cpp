#include "subsystems/gripper/GripperIOLogger.h"

GripperIOLogger::GripperIOLogger(wpi::log::DataLog& log, const std::string& path)
    :     isFeederMotorConnected(log, path + "/isFeederMotorConnected"),
          isOuttakeMotorConnected(log, path + "/isOuttakeMotorConnected"),
          feederAppliedVoltage(log, path + "/feederAppliedVoltage"),
          feederBusVoltage(log, path + "/feederBusVoltage"),
          feederCurrent(log, path + "/feederCurrent"),
          feederTemperature(log, path + "/feederTemperature"),
          feederRPM(log, path + "/feederRPM"),
          outtakeAppliedVoltage(log, path + "/outtakeAppliedVoltage"),
          outtakeBusVoltage(log, path + "/outtakeBusVoltage"),
          outtakeCurrent(log, path + "/outtakeCurrent"),
          outtakeTemperature(log, path + "/outtakeTemperature"),
          outtakeRPM(log, path + "/outtakeRPM"),
          IRBreakerUp(log, path + "/IRBreakerUp"),
          IRBreakerUp2(log, path + "/IRBreakerUp2"),
          IRBreakerDown(log, path + "/IRBreakerDown")
{}

void GripperIOLogger::Log(const GripperIOInputs& inputs) {
    isFeederMotorConnected.Append(inputs.isFeederMotorConnected);
    isOuttakeMotorConnected.Append(inputs.isOuttakeMotorConnected);
    feederAppliedVoltage.Append(inputs.feederAppliedVoltage);
    feederBusVoltage.Append(inputs.feederBusVoltage);
    feederCurrent.Append(inputs.feederCurrent);
    feederTemperature.Append(inputs.feederTemperature);
    feederRPM.Append(inputs.feederRPM);
    outtakeAppliedVoltage.Append(inputs.outtakeAppliedVoltage);
    outtakeBusVoltage.Append(inputs.outtakeBusVoltage);
    outtakeCurrent.Append(inputs.outtakeCurrent);
    outtakeTemperature.Append(inputs.outtakeTemperature);
    outtakeRPM.Append(inputs.outtakeRPM);
    IRBreakerUp.Append(inputs.IRBreakerUp);
    IRBreakerUp2.Append(inputs.IRBreakerUp2);
    IRBreakerDown.Append(inputs.IRBreakerDown);
}