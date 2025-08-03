#pragma once
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/geometry/Pose2d.h"

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

    double leftSideTraveledDistance = 0.0;
    double leftSideVelocity = 0.0;
    double rightSideTraveledDistance = 0.0;
    double rightSideVelocity = 0.0;

    frc::Pose2d robotPosition;
};


class DrivetrainIO {
public:
    virtual ~DrivetrainIO() = default;

    virtual void UpdateInputs(DrivetrainIOInputs& inputs) = 0; //COMMENTME

    virtual void SetVoltage(const double leftSideVoltage, const double rightSideVoltage) = 0; //COMMENTME
    virtual void SetDutyCycle(const double leftSideDutyCycle, const double rightSideDutyCycle) = 0; //COMMENTME
    virtual void SetChassisSpeed(const frc::ChassisSpeeds &speeds) = 0;

    virtual void ResetPosition(const frc::Pose2d position) = 0; //COMMENTME
};