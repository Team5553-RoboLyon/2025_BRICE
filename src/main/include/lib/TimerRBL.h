/*******************************************************************************
 * 
 * File        : TimerRBL.h (v1.0)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : 
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <frc/DriverStation.h>
#include <frc/RobotController.h>


class TimerRBL {
public:
  static double GetFPGATimestampInSeconds();
  static uint64_t GetFPGATimestampInMicroSeconds();
  static units::time::second_t GetPeriodRemainingTime();

  TimerRBL();

/**
 * @brief  Get the total elapsed time since the timer started (in seconds).
 *
 * @return The total elapsed time in seconds as a double.
 */
  double GetElapsedTimeSeconds() const;

/**
     * @brief Resets the timer by clearing the accumulated time and setting the start time to the current system time.
     * 
     * This function sets the accumulated seconds to 0.0 and updates the start time
     * to the current system time in milliseconds. It effectively restarts the timer.
     */
  void Reset();

  /**
     * @brief Starts the timer if it is not already running.
     */
  void Start();

  /**
     * @brief Stops the timer and updates the accumulated time.
     */
  void Stop();

  /**
     * @brief Restarts the timer by stopping it if it is currently running, 
     *        resetting its value, and then starting it again.
     *
     * This method ensures that the timer begins counting from zero, regardless 
     * of its previous state. If the timer is already running, it will be stopped 
     * before being reset and started again.
     */
  void Restart();

  /**
     * @brief Checks if the timer is currently running.
     * 
     * @return true if the timer is running, false otherwise.
     */
  bool IsRunning() const;

private:

    /**
     * @brief give the current system time in milliseconds.
     * 
     * This method returns the system time as a double value representing
     * the number of milliseconds since the system started with the 
     * frc::RobotController::GetTime() function.
     * 
     * @return The current system time in milliseconds as a double.
     */
  double GetSystemMilliseconds() const;

  double m_startMilliseconds{0.0};
  double m_accumulatedSeconds{0.0};
  bool m_isRunning{false};
};