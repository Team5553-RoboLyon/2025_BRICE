#pragma once


struct DrivetrainIOInputs
{
    bool isFrontLeftMotorConnected = true;
    bool isBackLeftMotorConnected = true;
    bool isFrontRightMotorConnected = true;
    bool isBackRightMotorConnected = true;

    double frontLeftMotorAppliedVoltage = 0.0;
    double frontLeftMotorBusVoltage = 0.0;
    double frontLeftMotorCurrent = 0.0;
    double frontLeftMotorTemperature = 0.0;
    double backLeftMotorAppliedVoltage = 0.0;
    double backLeftMotorBusVoltage = 0.0;
    double backLeftMotorCurrent = 0.0;
    double backLeftMotorTemperature = 0.0;

    double frontRightMotorAppliedVoltage = 0.0;
    double frontRightMotorBusVoltage = 0.0;
    double frontRightMotorCurrent = 0.0;
    double frontRightMotorTemperature = 0.0;
    double backRightMotorAppliedVoltage = 0.0;
    double backRightMotorBusVoltage = 0.0;
    double backRightMotorCurrent = 0.0;
    double backRightMotorTemperature = 0.0;

    double leftDistance = 0.0;
    double leftVelocity = 0.0;
    double rightDistance = 0.0;
    double rightVelocity = 0.0;
};


class DrivetrainIO {
public:
    virtual ~DrivetrainIO() = default;

    virtual void UpdateInputs(DrivetrainIOInputs& inputs) = 0; //COMMENTME

    virtual void SetVoltage(double leftSideVoltage, double rightSideVoltage) = 0; //COMMENTME
    virtual void SetDutyCycle(double leftSideDutyCycle, double rightSideDutyCycle) = 0; //COMMENTME
    virtual void ResetPosition() = 0; //COMMENTME
};