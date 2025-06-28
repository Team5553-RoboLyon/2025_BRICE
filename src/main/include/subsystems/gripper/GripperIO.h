#pragma once

struct GripperIOInputs
{
    bool isFeederMotorConnected = true;
    bool isOuttakeMotorConnected = true;
    
    double feederAppliedVoltage = 0.0;
    double feederBusVoltage = 0.0;
    double feederCurrent = 0.0;
    double feederTemperature = 0.0;
    double feederRPM = 0.0;
    double outtakeAppliedVoltage = 0.0;
    double outtakeBusVoltage = 0.0;
    double outtakeCurrent = 0.0;
    double outtakeTemperature = 0.0;
    double outtakeRPM = 0.0; 

    bool IRBreakerUp = false;
    bool IRBreakerUp2 = false;
    bool IRBreakerDown = false;
};


class GripperIO {
public:
    virtual ~GripperIO() = default;

    //COMMENTME
    virtual void UpdateInputs(GripperIOInputs& inputs) = 0;

    //COMMENTME
    virtual void SetFeederVoltage(const double voltage) = 0;
    //COMMENTME
    virtual void SetOuttakeVoltage(const double voltage) = 0;

    //COMMENTME
    virtual void SetFeederDutyCycle(const double dutyCycle) = 0;
    //COMMENTME
    virtual void SetOuttakeDutyCycle(const double dutyCycle) = 0;

    //COMMENTME
    virtual void SetFeederRPM(const double RPM) = 0;
    //COMMENTME
    virtual void SetOuttakeRPM(const double RPM) = 0;
};