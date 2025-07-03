/*******************************************************************************
 * 
 * File        : RateLimiter.h (v2.0)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Rate Limiter class for controlling the rate of change of a value.
 *               Appropriate for applications where smooth transitions are required,
 *               such as voltage or velocity control.
 * 
 * Authors     : Gaspard (2023), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <string>

class RateLimiter
{

public:
  RateLimiter();
  RateLimiter(const double timeToReachMax);
  RateLimiter(const double timeToReachMaxUp, const double timeToReachMaxDown);
  /**
   * @brief Sets the delta time for the RateLimiter and updates the rate limits accordingly.
   * 
   * @param deltaTime The new delta time value in seconds. Must be greater than 0.0.
   */
  void SetDeltaTime(const double deltaTime);
  
  /**
   * @brief Sets the rate limit for the RateLimiter based on the time required to reach the maximum value.
   * 
   * This function calculates the rate limit for both upward and downward directions
   * based on the provided time to reach the maximum value.
   * 
   * @param timeToReachMax The time (in seconds) required to reach the maximum value. 
   *                       Must be greater than 0.0.
   * 
   * @warning If timeToReachMax is less than or equal to zero, the rate limits are set to 0,
   *          and an error message is logged.
   */
  void SetRateLimit(const double timeToReachMax);
  /**
   * @brief Sets the rate limits for the RateLimiter.
   * 
   * This function configures the rate at which the value can increase or decrease
   * based on the specified time to reach the maximum value in either direction.
   * 
   * @param timeToReachMaxUp The time (in seconds) required to reach the maximum value when increasing.
   *                         Must be greater than 0.0.
   * @param timeToReachMaxDown The time (in seconds) required to reach the maximum value when decreasing. 
   *                           Must be greater than 0.0.
   * 
   * @note If either of the input parameters is less than or equal to 0.0, the rate limits will default to 0.0,
   *       and an error will be logged.
   * 
   * @warning Ensure that both `timeToReachMaxUp` and `timeToReachMaxDown` are positive values to avoid invalid behavior.
   */
  void SetRateLimit(const double timeToReachMaxUp, const double timeToReachMaxDown);  
  /**
   * Sets the target speed value for the rate limiter.
   *
   * @param current The new target speed value to be set.
   */
  void SetTarget(const double target);
  /**
   * Sets the current speed value for the rate limiter.
   *
   * @param current The new current speed value to be set.
   */
  void SetCurrent(const double current);

  double GetCurrentSpeed() const;
  double GetTargetSpeed() const;
  double GetRateLimitUp() const;
  double GetRateLimitDown() const;
  double GetDeltaTime() const;
    /**
   * @brief Retrieves the current state of the Rate Limiter as a formatted string.
   * 
   * @return A string containing the formatted state of the Rate Limiter.
   */
  std::string GetState() const;

  /**
   * Updates the current speed towards the target speed, applying rate limits
   * for acceleration and deceleration.
   *
   * @return The updated current speed after applying rate limits.
   */
  double Update();
  /**
   * Updates the current speed towards the target speed, applying rate limits
   * for acceleration and deceleration.
   *
   * @param target The desired target speed.
   * @return The updated current speed after applying rate limits.
   */
  double Update(double target);
  /**
   * @brief Resets the RateLimiter's current and target speeds to zero.
   */
  void Reset();
  /**
   * Resets the rate limiter with a new target speed and current speed.
   *
   * @param target The desired target speed to set.
   * @param current The current speed to set.
   */
  void Reset(double target, double current);

private:
  double m_dt{0.02}; // Default to 20ms in FRC, can be set by SetDeltaTime
  double m_rateLimitUp; // Rate limit for increasing speed
  double m_rateLimitDown; // Rate limit for decreasing speed
  double m_currentSpeed; // Current speed of the rate limiter
  double m_targetSpeed; // Target speed to reach
};