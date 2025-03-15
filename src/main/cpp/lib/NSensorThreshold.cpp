#include "lib/NSensorThreshold.h"

NSensorThreshold::NSensorThreshold(int id, u_int32_t initalPosition, u_int32_t nbPosition) : m_sensor(id) {
    m_position = initalPosition;
    m_nextPosition = initalPosition;
    m_numberOfPositions = nbPosition;
    m_state = State::kLow;
}

void NSensorThreshold::SetThresholds(double lowTrshld, double highTrshld) {
    m_low = lowTrshld;
    m_high = highTrshld;
}

void NSensorThreshold::ResetCounter(u_int32_t Index, int32_t sens, State state)
{   
    m_state = state;
    m_position = Index;

    if(sens == 1)
        MoveUp();
    else /* sens == -1*/
        MoveDown();
    // assert(m_position < m_numberOfPositions && "Hall Fx's Position is out of bounds");
    // assert(m_position >= 0 && "Hall Fx's Position is out of bounds");
}
void NSensorThreshold::MoveUp() 
{
    if(m_state == State::kLow)
    {
        if( m_sens == -1)
            m_nextPosition = m_position;
    }
    else // m_state == State::khigh
    {
        m_nextPosition = m_position + 1;
    }
    
    m_sens = 1;
}
void NSensorThreshold::MoveDown()
{

    if(m_state == State::kLow)
    {
        if( m_sens == 1)
            m_nextPosition = m_position;
    }
    else // m_state == State::khigh
    {
        m_nextPosition = m_position - 1;
    }
    m_sens = -1;
}
void NSensorThreshold::Update()
{
    m_sensorValue = m_sensor.GetVoltage();
    if(m_state == State::kLow)
    {
        if(m_sensorValue > m_high)
        {
            m_position = m_nextPosition;
            m_state = State::kHigh;
        }
    } 
    else // m_state == State::kHigh
    {
        if(m_sensorValue < m_low)
        {
            m_state = State::kLow;
            m_nextPosition = m_position + m_sens;
        }
    }
    // assert(m_position < m_numberOfPositions && "Hall Fx's Position is out of bounds");
    // assert(m_position >= 0 && "Hall Fx's Position is out of bounds");
}

bool NSensorThreshold::IsHigh() {
    if(m_state == State::kHigh) 
        return true;
    else 
        return false;
}