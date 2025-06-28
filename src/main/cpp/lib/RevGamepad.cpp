#include "lib/RevGamepad.h" 

RevGamepad::RevGamepad(int port)
  : frc::GenericHID(port),
    m_thresholdAxisAsButton(0.5) {
}
RevGamepad::RevGamepad(int port, double threshold)
  : frc::GenericHID(port),
    m_thresholdAxisAsButton(threshold) {
}

double RevGamepad::GetLeftX() const 
{
  return GetRawAxis(Axis::kLeftX);
}
double RevGamepad::GetLeftY() const 
{
  return GetRawAxis(Axis::kLeftY);
}

double RevGamepad::GetRightX() const 
{
  return GetRawAxis(Axis::kRightX);
}
double RevGamepad::GetRightY() const 
{
  return GetRawAxis(Axis::kRightY);
}

double RevGamepad::GetL2Axis() const 
{
  return GetRawAxis(Axis::kL2);
}
double RevGamepad::GetR2Axis() const 
{
  return GetRawAxis(Axis::kR2);
}

bool RevGamepad::GetSquareButton() const 
{
    return GetRawButton(Button::kSquare);
}
bool RevGamepad::GetSquareButtonPressed() 
{
    return GetRawButtonPressed(Button::kSquare);
}
bool RevGamepad::GetSquareButtonReleased() 
{
    return GetRawButtonReleased(Button::kSquare);
}

bool RevGamepad::GetCrossButton() const 
{
    return GetRawButton(Button::kCross);
}
bool RevGamepad::GetCrossButtonPressed() 
{
    return GetRawButtonPressed(Button::kCross);
}
bool RevGamepad::GetCrossButtonReleased() 
{
    return GetRawButtonReleased(Button::kCross);
}

bool RevGamepad::GetCircleButton() const 
{
    return GetRawButton(Button::kCircle);
}
bool RevGamepad::GetCircleButtonPressed() 
{
    return GetRawButtonPressed(Button::kCircle);
}
bool RevGamepad::GetCircleButtonReleased() 
{
    return GetRawButtonReleased(Button::kCircle);
}

bool RevGamepad::GetTriangleButton() const 
{
    return GetRawButton(Button::kTriangle);
}
bool RevGamepad::GetTriangleButtonPressed() 
{
    return GetRawButtonPressed(Button::kTriangle);
}
bool RevGamepad::GetTriangleButtonReleased() 
{
    return GetRawButtonReleased(Button::kTriangle);
}

bool RevGamepad::GetL1Button() const 
{
    return GetRawButton(Button::kL1);
}
bool RevGamepad::GetL1ButtonPressed() 
{
    return GetRawButtonPressed(Button::kL1);
}
bool RevGamepad::GetL1ButtonReleased() 
{
    return GetRawButtonReleased(Button::kL1);
}

bool RevGamepad::GetR1Button() const 
{
    return GetRawButton(Button::kR1);
}
bool RevGamepad::GetR1ButtonPressed() 
{
    return GetRawButtonPressed(Button::kR1);
}
bool RevGamepad::GetR1ButtonReleased() 
{
    return GetRawButtonReleased(Button::kR1);
}

bool RevGamepad::GetL2AsButton() const 
{
    return GetRawAxis(Axis::kL2) > m_thresholdAxisAsButton;
}
bool RevGamepad::GetL2AsButtonPressed() 
{
    if(GetRawAxis(Axis::kL2) > m_thresholdAxisAsButton)
    {
        if(L2AsButtonPressed)
        {
            return false; // Already pressed
        }
        else
        {
            L2AsButtonPressed = true; // Mark as pressed for next call
            return true;
        }
    }
    else 
    {
        L2AsButtonPressed = false; // Reset pressed state
        return false;
    }
}
bool RevGamepad::GetL2AsButtonReleased() 
{
    if(GetRawAxis(Axis::kL2) <= m_thresholdAxisAsButton)
    {
        if(!L2AsButtonPressed)
        {
            return false; // Already released
        }
        else
        {
            L2AsButtonPressed = false; // Mark as released for next call
            return true;
        }
    }
    else 
    {
        L2AsButtonPressed = true; // Reset pressed state
        return false;
    }
}

bool RevGamepad::GetR2AsButton() const 
{
    return GetRawAxis(Axis::kR2) > m_thresholdAxisAsButton;
}
bool RevGamepad::GetR2AsButtonPressed() 
{
    if(GetRawAxis(Axis::kR2) > m_thresholdAxisAsButton)
    {
        if(R2AsButtonPressed)
        {
            return false; // Already pressed
        }
        else
        {
            R2AsButtonPressed = true; // Mark as pressed for next call
            return true;
        }
    }
    else 
    {
        R2AsButtonPressed = false; // Reset pressed state
        return false;
    }
}
bool RevGamepad::GetR2AsButtonReleased() 
{
    if(GetRawAxis(Axis::kR2) <= m_thresholdAxisAsButton)
    {
        if(!R2AsButtonPressed)
        {
            return false; // Already released
        }
        else
        {
            R2AsButtonPressed = false; // Mark as released for next call
            return true;
        }
    }
    else 
    {
        R2AsButtonPressed = true; // Reset pressed state
        return false;
    }
}

bool RevGamepad::GetL3AsButton() const 
{
    return GetRawButton(Button::kL3);
}
bool RevGamepad::GetL3AsButtonPressed() 
{
    return GetRawButtonPressed(Button::kL3);
}
bool RevGamepad::GetL3AsButtonReleased() 
{
    return GetRawButtonReleased(Button::kL3);
}

bool RevGamepad::GetR3AsButton() const 
{
    return GetRawButton(Button::kR3);
}
bool RevGamepad::GetR3AsButtonPressed() 
{
    return GetRawButtonPressed(Button::kR3);
}
bool RevGamepad::GetR3AsButtonReleased() 
{
    return GetRawButtonReleased(Button::kR3);
}

bool RevGamepad::GetShareButton() const 
{
    return GetRawButton(Button::kShare);
}
bool RevGamepad::GetShareButtonPressed() 
{
    return GetRawButtonPressed(Button::kShare);
}
bool RevGamepad::GetShareButtonReleased() 
{
    return GetRawButtonReleased(Button::kShare);
}

bool RevGamepad::GetOptionsButton() const 
{
    return GetRawButton(Button::kOptions);
}
bool RevGamepad::GetOptionsButtonPressed() 
{
    return GetRawButtonPressed(Button::kOptions);
}
bool RevGamepad::GetOptionsButtonReleased() 
{
    return GetRawButtonReleased(Button::kOptions);
}

int RevGamepad::GetPOV() 
{
    return GenericHID::GetPOV(0);
}

void RevGamepad::SetRumble(RumbleType type, double value) 
{
    GenericHID::SetRumble(type, value);
}

bool RevGamepad::IsRevGamepadConnected() 
{
    return  IsConnected() && 
            GetAxisCount() == 6 &&
            GetButtonCount() == 10 &&
            GetPOVCount() == 1;
}