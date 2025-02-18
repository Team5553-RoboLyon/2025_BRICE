#pragma once

typedef struct Dynamic Dynamic; 
struct Dynamic
{
    Dynamic() : m_current(0.0), m_delta(0.0){};
    Dynamic(double initialvalue) : m_current(initialvalue), m_delta(0.0){};
    ~Dynamic(){};
    double m_current;       // The current value.
    double m_delta;         // The change in value since the last update.

    /**
    * @brief Sets a new value and updates the delta.
    * @param value The new value to set.
    */
    void set(double value)
    {
        m_delta = (value - m_current);
        m_current = value;
    };

    /**
    * @brief Resets the current value to the specified initial value.
    * 
    * This function sets the current value to the provided initial value and 
    * resets the delta to 0.0.
    * 
    * @param initialvalue The value to set as the current value.
    */
    void reset(double initialvalue)
    {
        m_current = initialvalue;
        m_delta = 0.0;
    };
};