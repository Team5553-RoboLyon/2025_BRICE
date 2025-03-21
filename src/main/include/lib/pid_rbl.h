#include "lib/UtilsRBL.h"

class PidRBL {
    public :
  /**
   * @brief Constructs a PID controller with a specified setpoint and gains.
   * @param setpoint The desired setpoint.
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */
  PidRBL(double setpoint, double kp, double ki, double kd);
  /** 
   * @brief Constructs a PID controller with a specified gains.
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */
  PidRBL(double kp, double ki, double kd);
  /**
   * @brief Sets the gains of the PID controller.
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   */
  void SetGains(double kp, double ki, double kd);
  /**
   * @brief Sets the setpoint for the PID controller.
   * @param setpoint The desired setpoint.
   */
  void SetSetpoint(double setpoint);
  /**
   * @brief Calculates the output of the PID controller based on the current measurement.
   * @param measurement The current measurement.
   * @return The calculated output.
   */
  double Calculate(double measurement);
  /**
   * @brief Calculates the output of the PID controller based on the given setpoint and measurement.
   * @param setpoint The desired setpoint.
   * @param measurement The current measurement.
   * @return The calculated output.
   */
  double Calculate(double setpoint, double measurement);
  /**
   * @brief Calculates the output of the PID controller based on the error.
   * @param error The current error.
   * @return The calculated output.
   */
  double CalculateWithError(double error);
  /**
  * @brief Resets the PID controller with a new setpoint.
  *
  * This function resets the internal state of the PID controller, including
  * the setpoint, error terms, and output. It is useful when you need to start
  * the PID control process with a new target value.
  *
  * @param setpoint The new setpoint value to initialize the PID controller with.
  */
  void Reset(double setpoint);
  /**
   * @brief Sets the tolerance for considering the measurement at the setpoint.
   * @param tolerance The tolerance value.
   */
  void SetTolerance(double tolerance);
  /**
  * @brief Sets the output limits for the PID controller.
  * 
  * This function sets the minimum and maximum output values that the PID controller
  * can produce. These limits are used to constrain the output to a specific range.
  * 
  * @param min The minimum output value.
  * @param max The maximum output value.
  */
  void SetOutputLimits(double min, double max);
  /**
   * @brief Checks if the measurement is at the setpoint within the specified tolerance.
   * @return True if the measurement is at the setpoint, false otherwise.
   */
  bool AtSetpoint();

private:

  double m_setpoint;    // Desired setpoint.
  double m_tolerance;   // Tolerance for considering the measurement at the setpoint.
  double m_output;      // Output of the PID controller.
  double m_outputMin = 0.0;   // Min output value
  double m_outputMax = 1.0;   // Max output value
  double m_error;       // Error between the setpoint and the measurement.
  double m_lastError;   // Previous error.
  double m_integrative; // Integral of the error.
  double m_derivative;  // Derivative of the error.
  double m_kp;          // Proportional gain.
  double m_ki;          // Integral gain.
  double m_kd;          // Derivative gain.
  double m_dt;          // Time step.
}; 
