/*******************************************************************************
 * 
 * File        : TunableValueLogger.h (v1.0)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Provides a class for managing tunable numeric values that can 
 *               be adjusted via the robot dashboard using NetworkTables. 
 *               Includes functionality for detecting changes and executing 
 *               actions based on updated values.
 * 
 * Authors     : AKA (2025), last update by AKA (2025) 
 *                           and inspired by Team 6328
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include <networktables/NetworkTableInstance.h>
#include <unordered_map>
#include <string>
#include <functional>
#include <initializer_list>

/**
 * TunableValueLogger class
 * 
 * This class represents a number that can be tuned from the robot dashboard.
 * - It stores a key to identify the value on the dashboard.
 * - It keeps a default value to use if tuning is not active.
 * - It detects if the value has changed since the last time it was checked.
 * - It supports running an action when one or more tunable numbers change.
 */
class TunableValueLogger {
public:
  /**
   * Constructor that takes a dashboard key (name of the tunable number).
   * The key will be used in NetworkTables under "/Tuning/" path.
   * @note The TunableValue won't work until a defaultValue is set.
   * 
   * @param dashboardKey The name of the tunable value on the dashboard.
   */
  explicit TunableValueLogger(const std::string& dashboardKey);

  /**
   * Constructor that takes a dashboard key (name of the tunable number).
   * The key will be used in NetworkTables under "/Tuning/" path.
   * 
   * @param dashboardKey The name of the tunable value.
   * @param defaultValue The default number used if dashboard value is not set.
   */
  TunableValueLogger(const std::string& dashboardKey, double defaultValue);

  /**
   * Initialize the default value.
   * This method only sets the default once.
   * It also creates the NetworkTable entry and publishes the default value to the dashboard.
   * 
   * @param defaultValue The default number to use.
   */
  void InitDefault(double defaultValue);

  /**
   * Get the current value of the tunable number.
   * It returns the dashboard value; otherwise returns the default.
   * 
   * @note It works only if the defaultValue is set.
   * 
   * @return The current tunable number value.
   */
  double Get() const;

  /**
   * Check if the tunable number has changed since the last time this function was called
   * with the same unique id.
   * 
   * @param id A unique identifier (usually related to where this function is called)
   * @return True if the value changed since last check, false otherwise.
   */
  bool HasChanged(int id);

  /**
   * Run an action if any of the given tunable numbers changed since last check.
   * Passes the current values of all tunables to the action.
   * 
   * @param id Unique identifier for the caller to keep track of last values.
   * @param action Function to call if any tunable changed. Receives all current values.
   * @param tunables List of pointers to tunable numbers to check.
   */
  static void IfChanged(int id,
                        std::function<void(const std::vector<double>&)> action,
                        std::initializer_list<TunableValueLogger*> tunables);

private:
  std::string m_key;                      // NetworkTables key like "/Tuning/KeyName"
  bool m_hasDefault;                      // Whether a default value was set
  double m_default;                       // Default value used when tuning is off or no value set
  nt::NetworkTableEntry m_entry;         // NetworkTables entry connected to the dashboard
  std::unordered_map<int, double> m_lastValues; // Stores last value seen for each id to detect changes
};