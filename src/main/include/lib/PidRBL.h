/*******************************************************************************
 * 
 * File        : PidRBL.h (v3.4)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Advanced PID controller class implementing 
 *               Proportional-Integral-Derivative control with optional 
 *               Feedforward. Supports real-time update, input/output clamping, 
 *               tolerance checking, and continuous inputs (e.g. angle 
 *               wrap-around).
 * 
 * Authors     : Gaspard (2023), last update by AKA (2025) 
 *                                and inspired by Team 1678
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <string>
#define THEORETICAL_DT (0.02)

class PidRBL {
public :
  PidRBL();
  PidRBL(const double kp, const double ki, const double kd);
  PidRBL(double kp, double ki, double kd, double ff);


  /**
   * @brief Sets the PIDF (Proportional, Integral, Derivative, Feedforward) gains for the controller.
   * 
   * @note This function calls Reset() internally to clear the previous state of the controller.
   * 
   * @param kp The proportional gain, which determines the reaction to the current error.
   * @param ki The integral gain, which determines the reaction based on the accumulation of past errors.
   * @param kd The derivative gain, which determines the reaction based on the rate of change of the error.
   * @param ff The feedforward term, which provides a baseline output independent of the error (default is 0.0).
   */
  void SetGains(const double kp, const double ki, const double kd, const double ff = 0.0);
  /**
   * @brief Sets the feedforward gain for the PID controller.
   * 
   * @warning This function must be called before Calculate() to ensure that the 
   *          feedforward gain is applied correctly.
   * 
   * @param kf The feedforward term value to be set. This value is used to 
   *        directly scale the input to the controller, providing a baseline 
   *        output that is independent of the error.
   */
  void SetFeedforward(const double ff);
  /**
   * @brief Sets the desired setpoint for the PID controller, clamping it within the allowed input range.
   * 
   * @param setpoint The desired setpoint value for the PID controller.
   */
  void SetSetpoint(const double setpoint);
  /**
   * @brief Sets the tolerance for the PID controller.
   * 
   * @param tolerance The absolute error below which the controller is considered "at setpoint"
   */
  void SetTolerance(const double tolerance);
  /**
   * Sets the output limits for the PID controller.
   *
   * @param min The minimum output value that the PID controller can produce.
   * @param max The maximum output value that the PID controller can produce.
   */
  void SetOutputLimits(const double min, const double max);
  /**
   * @brief Sets the input limits (acceptable range for setpoint values) for the PID controller.
   * 
   * @param min The minimum allowable input value.
   * @param max The maximum allowable input value.
   */
  void SetInputLimits(const double min, const double max);
  /**
   * @brief Sets the input limits for the PID controller.
   * 
   * This function activates or deactivates the input limits for the PID controller
   * based on the provided boolean parameter. When input limits are active, the 
   * controller will constrain the input values within a predefined range.
   * 
   * @param isActive A boolean value indicating whether input limits should be active.
   */
  void SetInputLimits(const bool isActive);
  /**
   * @brief Sets whether the PID controller should handle inputs as continuous.
   * 
   * When enabled, the controller will treat the input range as circular, allowing
   * for seamless transitions between the minimum and maximum values.
   * 
   * @param isContinuous A boolean indicating whether circular input handling is enabled.
   */
  void SetContinuous(const bool isContinuous);

  double GetKP() const;
  double GetKI() const;
  double GetKD() const;
  double GetFF() const;
  double GetError() const;
  double GetSetpoint() const;
  /**
   * @brief Retrieves the current state of the PID controller as a formatted string.
   * 
   * @return A string containing the formatted state of the PID controller.
   */
  std::string GetState() const;

    /**
   * Calculates the PID output using theoretical delta time (0.02s in FRC).
   *    
   * @param measurement The current value of the system being controlled.
   * @return The PID output value.
   */
  double Calculate(const double measurement);
  /**
   * Calculates the PID output using theoretical delta time (0.02s in FRC).
   *    
   * @param setpoint The desired target value for the system.
   * @param measurement The current value of the system being controlled.
   * @return The PID output value.
   */
  double Calculate(const double setpoint, const double measurement);
    /**
   * Calculates the PID output using real timestamp.
   *
   * @param measurement The current value of the system being controlled.
   * @param timestamp The current time in seconds, used for real-time calculations.
   * @return The PID output value.
   */
  double CalculateWithRealTime(const double measurement, const double timestamp);
  /**
   * Calculates the PID output using real timestamp.
   *
   * @param setpoint The desired target value for the system.
   * @param measurement The current value of the system being controlled.
   * @param timestamp The current time in seconds, used for real-time calculations.
   * @return The PID output value.
   */
  double CalculateWithRealTime(const double setpoint, const double measurement, const double timestamp);
   /**
   * @brief Resets the PID controller state.
   * 
   * This function adjusts the setpoint to match the current measurement.
   * It also resets the previous error, current error, and output to zero.
   * Additionally, it clears the integrative component of the PID controller.
   */
  void Reset();
  /**
   * @brief Resets the PID controller state and updates the last timestamp.
   * 
   * This method resets the internal state of the PID controller and sets
   * the last timestamp to the provided value. Also it calls Reset() to clear the previous state.
   * 
   * @param timestamp The current timestamp to set as the last timestamp.
   */
  void Reset(const double timestamp);
  /**
   * @brief Resets the integrative term accumulator to zero.
   * 
   * This function clears the I-term accumulator to prevent integrative windup
   * in the PID controller. It does not affect other components of the PID
   * controller.
   */
  void ResetIntegrative();
  /**
   * @brief Determines if the current error with the setpoint is within the acceptable tolerance.
   * @return true if the current error is within the tolerance, false otherwise.
   */
  bool AtSetpoint() const;
private:

  double m_kp;  // factor for Proportional gain
  double m_ki;  // factor for Integral gain
  double m_kd;  // factor for Derivative gain
  double m_feedforward;  // Feedforward term

  double m_outputMin{-1.0};   // Min output value
  double m_outputMax{1.0};    // Max output value
  double m_inputMin;    // Min setpoint value allowed
  double m_inputMax;    // Max setpoint value allowed

  bool m_isContinuous{false}; // do the endpoints wrap around?
  bool m_isInputLimitsActive{false}; // When set to true, the inputs will be constrained within specified limits.

  double m_previousError{0.0};  // the prior error for derivative calculation
  double m_integrative{0.0};     // Total accumulated error for integral term
  double m_setpoint{0.0};       // Desired setpoint
  double m_currentError;        // Error between the setpoint and the measurement

  double m_output;              // Output of the PID controller.
  double m_tolerance{0.0};      // Tolerance for considering the measurement at the setpoint.
  
  double m_lastTimestamp{0.0};  // Last time the PID controller was updated
  double m_dt{THEORETICAL_DT}; // Time step for the PID controller, default in  FRC is 20ms
}; 