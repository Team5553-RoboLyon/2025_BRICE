#pragma once

struct ElevatorIOInputs
{
    bool isLeftMotorConnected = true;
    bool isRightMotorConnected = true;
    
    double leftMotorAppliedVoltage = 0.0;
    double leftMotorBusVoltage = 0.0;
    double leftMotorCurrent = 0.0;
    double leftMotorTemperature = 0.0;
    double rightMotorAppliedVoltage = 0.0;
    double rightMotorBusVoltage = 0.0;
    double rightMotorCurrent = 0.0;
    double rightMotorTemperature = 0.0;

    bool limitSwitchBottom = false;
    bool limitSwitchBottom2 = false;
    double heightPosition = 0.0;
};


class ElevatorIO {
public:
    virtual ~ElevatorIO() = default;

    virtual void UpdateInputs(ElevatorIOInputs& inputs) = 0; //COMMENTME

    virtual void SetVoltage(double voltage) = 0; //COMMENTME
    virtual void SetDutyCycle(double dutyCycle) = 0; //COMMENTME
    virtual void ResetPosition() = 0; //COMMENTME
};