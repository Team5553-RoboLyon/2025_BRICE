#include <math.h>

class PidRBL {
    public :
    PidRBL(double setpoint, double kp, double ki, double kd);
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
   * @brief Resets the PID controller.
   * @param error The initial error value.
   * @param lastError The initial last error value.
   * @param integrative The initial integrative value.
   * @param derivative The initial derivative value.
   * @param output The initial output value.
   */
  void Reset(double error = 0.0, double lastError = 0.0, double integrative = 0.0, double derivative = 0.0, double output = 0.0);
  /**
   * @brief Sets the tolerance for considering the measurement at the setpoint.
   * @param tolerance The tolerance value.
   */
  void SetTolerance(double tolerance);
  /**
   * @brief Checks if the measurement is at the setpoint within the specified tolerance.
   * @return True if the measurement is at the setpoint, false otherwise.
   */
  bool AtSetpoint();

  double m_error;       // Error between the setpoint and the measurement.
  double m_output;      // Output of the PID controller.
  double m_setpoint;    // Desired setpoint.

private:
  double m_lastError;   // Previous error.
  double m_integrative; // Integral of the error.
  double m_derivative;  // Derivative of the error.
  double m_tolerance;   // Tolerance for considering the measurement at the setpoint.
  double m_kp;          // Proportional gain.
  double m_ki;          // Integral gain.
  double m_kd;          // Derivative gain.
}; 
