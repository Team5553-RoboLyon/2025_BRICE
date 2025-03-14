#include "lib/UtilsRBL.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>


class NSensorThreshold {
    public :
      enum class State {
        kLow,
        kHigh
    };
  NSensorThreshold(int id, u_int32_t initalPosition, u_int32_t nbPosition);
  void SetThresholds(double lowTrshld, double highTrshld);
  void ResetCounter(u_int32_t Index,int32_t sens, NSensorThreshold::State state);
  void Update();
  void MoveUp();
  void MoveDown();
  u_int32_t GetPosition() {return m_position;};
  double GetSensorValue() {return m_sensorValue;};
  int GetSens() {return m_sens;};
  bool IsHigh();
private:
  double m_low;  
  double m_high; 
  double m_sensorValue;
  frc::AnalogInput m_sensor;
  u_int32_t m_position;
  u_int32_t m_nextPosition;
  int32_t m_sens;
  u_int32_t m_numberOfPositions; // unnecessary now
  State m_state = State::kLow;
};