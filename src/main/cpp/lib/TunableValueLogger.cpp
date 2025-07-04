#include "lib/TunableValueLogger.h"

TunableValueLogger::TunableValueLogger(const std::string& dashboardKey)
    : m_key("/Tuning/" + dashboardKey), m_hasDefault(false) 
{}

TunableValueLogger::TunableValueLogger(const std::string& dashboardKey, double defaultValue)
    : TunableValueLogger(dashboardKey) 
{
  InitDefault(defaultValue);
}

void TunableValueLogger::InitDefault(double defaultValue) 
{
    if (!m_hasDefault) 
    {
      m_hasDefault = true;
      m_default = defaultValue;

      // Get the NetworkTable entry under "Tuning" table
      // The key is without the "/Tuning/" prefix because NetworkTables paths start from the table root
      m_entry = nt::NetworkTableInstance::GetDefault()
                  .GetTable("Tuning")
                  ->GetEntry(m_key.substr(8)); // remove leading "/Tuning/"

      // Set the default value in the NetworkTable entry (shows on dashboard)
      m_entry.SetDouble(m_default);
    }
}

double TunableValueLogger::Get() const 
{
    if (!m_hasDefault) 
        return 0.0;  // If no default set, return 0
    return m_entry.GetDouble(m_default); // Get value from NetworkTables or fallback to default
}

bool TunableValueLogger::HasChanged(int id) 
{
    // Get the current value from NetworkTables
    double currentValue = m_entry.GetDouble(m_default);

    // Check if the value has changed since last check
    if (currentValue != m_lastValues[id]) 
    {
        m_lastValues[id] = currentValue; // Update last value for this id
        return true; // Value has changed
    }
    
    return false; // No change detected
}

void TunableValueLogger::IfChanged(int id,
                        std::function<void(const std::vector<double>&)> action,
                        std::initializer_list<TunableValueLogger*> tunables) 
{
    bool changed = false;
    std::vector<double> values;
    values.reserve(tunables.size());  // Prepare vector to hold all values

    // Check each tunable number
    for (auto* t : tunables) {
      if (t->HasChanged(id)) 
        changed = true;  // Mark if changed
      values.push_back(t->Get()); // Add current value to list
    }

    // If any changed, run the provided action passing current values
    if (changed) 
        action(values);
}