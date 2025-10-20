#pragma once

#include "Constants.h"

#include "lib/ControlMode.h"
#include "lib/DebugUtils.h"

using IdleMode = rev::spark::SparkBaseConfig::IdleMode;

#if (ROBOT_MODEL != (BABY_BRICE))
namespace gripperConstants
{   
    constexpr ControlMode MainControlMode = ControlMode::DUTY_CYCLE;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_DUTY_CYCLE;
    namespace IRbreaker
    {
        constexpr int DOWN_ID = 13;
        constexpr int UP_ID = 18;
        constexpr int UP2_ID = 19;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Counter 
    {
        constexpr int PRESCORE = 10; //TUNEME
        constexpr int SCORE = 20; //TUNEME
    }
    constexpr double OPEN_LOOP_REDUC = 2.0; //TUNEME

}

namespace feederConstants
{
    namespace Motor
    {
        constexpr int ID = 10;
        constexpr bool INVERTED = true;
    
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.1; //TUNEME
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (3.0 / 1.0) * (3.0 / 1.0); //ul
        constexpr int KV = 473; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * KV; //RPM
        constexpr double FEEDER_MAX_SPEED = MOTOR_FREE_SPEED / GEAR_RATIO;  //RPM
    }

    namespace DutyCycle
    {
        constexpr double COLLECTING = 0.5553; //TUNEME
        constexpr double REST = 0.0; //TUNEME
        constexpr double REJECTING_FORWARD = 0.3; //TUNEME
        constexpr double REJECTING_BACKWARD = -0.3; //TUNEME
    }
    namespace Velocity //in RPM after gear reduction
    {
        constexpr double COLLECTING = 291.8; //TUNEME
        constexpr double REST = 0.0;
        constexpr double REJECTING_FORWARD = 157.7; //TUNEME
        constexpr double REJECTING_BACKWARD = -157.7; //TUNEME
        constexpr double MAX = Specifications::FEEDER_MAX_SPEED; 
        constexpr double MIN = -MAX; 
    }

    namespace Gains
    {
        namespace VelocityPID
        {
            constexpr double KP = 5.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double MAX = Motor::VOLTAGE_COMPENSATION;
            constexpr double MIN = -Motor::VOLTAGE_COMPENSATION;
        }
    }
}
namespace outtakeConstants
{
    namespace Motor
    {
        constexpr int ID = 9;
        constexpr bool INVERTED = false;
    
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.1; //TUNEME
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (3.0 / 1.0); //ul
        constexpr int KV = 559; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * KV; //RPM
        constexpr double OUTTAKE_MAX_SPEED = MOTOR_FREE_SPEED / GEAR_RATIO;  //RPM
    }

    namespace DutyCycle
    {
        constexpr double FEEDING_EMPTY = 0.4; //TUNEME
        constexpr double FEEDING_FORWARD = 0.3; //TUNEME
        constexpr double FEEDING_BACKWARD = -0.25; //TUNEME
        constexpr double SHY = 0.05; //TUNEME
        constexpr double PRESCORE = -0.2; //TUNEME
        constexpr double REJECTING_BACKWARD = -0.3; //TUNEME
        constexpr double REJECTING_FORWARD = 0.3; //TUNEME
        constexpr double SHIFTING = 0.2; //TUNEME
        constexpr double REST = 0.0;
        constexpr double HIGH_SCORING = 0.4; //TUNEME
        constexpr double MIDDLE_SCORING = 0.43; //TUNEME
        constexpr double LOW_SCORING = 0.08; //TUNEME
    }
    namespace Velocity //in RPM after gear reduction
    {
        constexpr double FEEDING_EMPTY = 745.0; //TUNEME
        constexpr double FEEDING_FORWARD = 559.0; //TUNEME
        constexpr double FEEDING_BACKWARD = -465.0; //TUNEME
        constexpr double SHY = 93.0; //TUNEME
        constexpr double PRESCORE = -372.0; //TUNEME
        constexpr double REJECTING_BACKWARD = -559.0; //TUNEME
        constexpr double REJECTING_FORWARD = 559.0; //TUNEME
        constexpr double SHIFTING = 372.0; //TUNEME
        constexpr double REST = 0.0;
        constexpr double HIGH_SCORING = 745.0; //TUNEME
        constexpr double MIDDLE_SCORING = 800.0; //TUNEME
        constexpr double LOW_SCORING = 150.0; //TUNEME
        constexpr double MAX = Specifications::OUTTAKE_MAX_SPEED;
        constexpr double MIN = -MAX;
    }
    
    namespace Gains
    {
        namespace VelocityPID
        {
            constexpr double KP = 5.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double MAX = Motor::VOLTAGE_COMPENSATION;
            constexpr double MIN = -Motor::VOLTAGE_COMPENSATION;
        }
    }
}
#else
namespace gripperConstants
{   
    constexpr ControlMode MainControlMode = ControlMode::DUTY_CYCLE;
    constexpr ControlMode EmergencyControlMode = ControlMode::MANUAL_DUTY_CYCLE;
    namespace IRbreaker
    {
        constexpr int DOWN_ID = 13;
        constexpr int UP_ID = 18;
        constexpr int UP2_ID = 19;
        constexpr bool IS_TRIGGERED = false;
    }
    namespace Counter 
    {
        constexpr int PRESCORE = 10; //TUNEME
        constexpr int SCORE = 20; //TUNEME
    }
    constexpr double OPEN_LOOP_REDUC = 2.0; //TUNEME

}

namespace feederConstants
{
    namespace Motor
    {
        constexpr int ID = 10;
        constexpr bool INVERTED = true;
    
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.1; //TUNEME
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (3.0 / 1.0) * (3.0 / 1.0); //ul
        constexpr int KV = 473; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * KV; //RPM
    }

    namespace DutyCycle
    {
        constexpr double COLLECTING = 0.5553; //TUNEME
        constexpr double REST = 0.0; //TUNEME
        constexpr double REJECTING_FORWARD = 0.3; //TUNEME
        constexpr double REJECTING_BACKWARD = -0.3; //TUNEME
    }
    namespace Velocity //in RPM after gear reduction
    {
        constexpr double COLLECTING = 291.8; //TUNEME
        constexpr double REST = 0.0;
        constexpr double REJECTING_FORWARD = 157.7; //TUNEME
        constexpr double REJECTING_BACKWARD = -157.7; //TUNEME
        constexpr double MAX = 525; 
        constexpr double MIN = -MAX; 
    }

    namespace Gains
    {
        namespace VelocityPID
        {
            constexpr double KP = 5.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double MAX = Motor::VOLTAGE_COMPENSATION;
            constexpr double MIN = -Motor::VOLTAGE_COMPENSATION;
        }
    }
}
namespace outtakeConstants
{
    namespace Motor
    {
        constexpr int ID = 9;
        constexpr bool INVERTED = false;
    
        constexpr IdleMode IDLE_MODE = IdleMode::kBrake;
        constexpr double VOLTAGE_COMPENSATION = 10.0;
        constexpr double CURRENT_LIMIT = 40.0;
        constexpr double RAMP_RATE = 0.1; //TUNEME
        constexpr int HOT_THRESHOLD = 60; //TUNEME
        constexpr int OVERHEATING_THRESHOLD = 75; //TUNEME
    }

    namespace Specifications
    {
        constexpr double GEAR_RATIO = (3.0 / 1.0); //ul
        constexpr int KV = 559; // RPM.V-1
        constexpr double MOTOR_FREE_SPEED = Motor::VOLTAGE_COMPENSATION * KV; //RPM
    }

    namespace DutyCycle
    {
        constexpr double FEEDING_EMPTY = 0.4; //TUNEME
        constexpr double FEEDING_FORWARD = 0.3; //TUNEME
        constexpr double FEEDING_BACKWARD = -0.25; //TUNEME
        constexpr double SHY = 0.05; //TUNEME
        constexpr double PRESCORE = -0.2; //TUNEME
        constexpr double REJECTING_BACKWARD = -0.3; //TUNEME
        constexpr double REJECTING_FORWARD = 0.3; //TUNEME
        constexpr double SHIFTING = 0.2; //TUNEME
        constexpr double REST = 0.0;
        constexpr double HIGH_SCORING = 0.4; //TUNEME
        constexpr double MIDDLE_SCORING = 0.43; //TUNEME
        constexpr double LOW_SCORING = 0.08; //TUNEME
    }
    namespace Velocity //in RPM after gear reduction
    {
        constexpr double FEEDING_EMPTY = 745.0; //TUNEME
        constexpr double FEEDING_FORWARD = 559.0; //TUNEME
        constexpr double FEEDING_BACKWARD = -465.0; //TUNEME
        constexpr double SHY = 93.0; //TUNEME
        constexpr double PRESCORE = -372.0; //TUNEME
        constexpr double REJECTING_BACKWARD = -559.0; //TUNEME
        constexpr double REJECTING_FORWARD = 559.0; //TUNEME
        constexpr double SHIFTING = 372.0; //TUNEME
        constexpr double REST = 0.0;
        constexpr double HIGH_SCORING = 745.0; //TUNEME
        constexpr double MIDDLE_SCORING = 800.0; //TUNEME
        constexpr double LOW_SCORING = 150.0; //TUNEME
        constexpr double MAX = 1860.0; //TUNEME  
        constexpr double MIN = -MAX;
    }
    
    namespace Gains
    {
        namespace VelocityPID
        {
            constexpr double KP = 5.0; //TUNEME
            constexpr double KI = 0.0; //TUNEME
            constexpr double KD = 0.0; //TUNEME
            constexpr double KS = 0.0; //TUNEME
            constexpr double MAX = Motor::VOLTAGE_COMPENSATION;
            constexpr double MIN = -Motor::VOLTAGE_COMPENSATION;
        }
    }
}
#endif