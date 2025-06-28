#pragma once

struct StrafferIOInputs
{
    bool isMotorConnected = true;
    
    double appliedVoltage = 0.0;
    double busVoltage = 0.0;
    double current = 0.0;
    double temperature = 0.0;

    bool limitSwitchLeft = false;
    bool limitSwitchRight = false;
    double widthPosition = 0.0;
};


class StrafferIO {
  public:
    virtual ~StrafferIO() = default;

    /**
     * @brief Updates the input data structure with the current data of the Straffer subsystem.
     * 
     * This method retrieves sensor and motor data and updates the provided inputs structure.
     * 
     * @param inputs Reference to a StrafferIOInputs structure that will be populated with the current data.
     * 
     * Updates include:
     * - Motor connection status.
     * - Applied voltage to the motor.
     * - Bus voltage of the motor.
     * - Output current of the motor.
     * - Motor temperature.
     * - Bool of the left and right limit switches.
     * - Width position.
     */
    virtual void UpdateInputs(StrafferIOInputs& inputs) = 0;

    //COMMENTME
    virtual void SetVoltage(const double voltage) = 0;
    //COMMENTME
    virtual void SetDutyCycle(const double dutyCycle) = 0;

    virtual void ResetPosition() = 0;
    //COMMENTME
    virtual void ResetPositionLeft() = 0;
    //COMMENTME
    virtual void ResetPositionRight() = 0;   
};