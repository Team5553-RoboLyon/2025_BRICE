/*******************************************************************************
 * 
 * File        : RevGamepad.h (v1.1)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Handle input from REV Robotics Gamepad connected to the Driver Station.
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Github      : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/
#pragma once

#include "frc/GenericHID.h"
#include "frc2/command/button/Trigger.h"

//WARNING: all functions are not tested, please report any bug you find

/**
 * Handle input from REV Robotics Gamepad connected to the Driver Station.
 *
 * This class handles REV input that comes from the Driver Station. Each
 * time a value is requested the most recent value is returned. There is a
 * single class instance for each controller and the mapping of ports to
 * hardware buttons depends on the code in the Driver Station.
 */
class RevGamepad : public frc::GenericHID{
 public:
    /**
     * Construct an instance of a controller.
     *
     * The controller index is the USB port on the Driver Station.
     * The threshold for the axis to be considered pressed as a button is set to 0.5.
     *
     * @param port The port on the Driver Station that the controller is plugged
     *             into (0-5).
     */
    explicit RevGamepad(int port);

    /**
     * Construct an instance of a controller.
     *
     * The controller index is the USB port on the Driver Station.
     *
     * @param port The port on the Driver Station that the controller is plugged
     *             into (0-5).
     * @param threshold The threshold for the axis to be considered pressed as a button.
     */
    explicit RevGamepad(int port, double threshold);

    ~RevGamepad() override = default;

    RevGamepad(RevGamepad&&) = default;
    RevGamepad& operator=(RevGamepad&&) = default;

    /**
     * Get the X axis value of left side of the controller.
     * @return the axis value.
     */
    double GetLeftX() const;
    /**
     * Get the Y axis value of left side of the controller.
     * @return the axis value.
     */
    double GetLeftY() const;

    /**
     * Get the X axis value of right side of the controller.
     * @return the axis value.
     */
    double GetRightX() const;
    /**
     * Get the Y axis value of right side of the controller.
     * @return the axis value.
     */
    double GetRightY() const;

    /**
     * Get the left trigger 2 axis value of the controller. Note that this axis
     * is bound to the range of [0, 1].
     * @return the axis value.
     */
    double GetL2Axis() const;
    /**
     * Get the right trigger 2 axis value of the controller. Note that this axis
     * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
     * @return the axis value.
     */
    double GetR2Axis() const;

    /**
     * Read the value of the square button on the controller.
     * @return The state of the button.
     */
    bool GetSquareButton() const;
    /**
     * Whether the square button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetSquareButtonPressed();
    /**
     * Whether the square button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetSquareButtonReleased();

    /**
     * Read the value of the cross button on the controller.
     * @return The state of the button.
     */
    bool GetCrossButton() const;
    /**
     * Whether the cross button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetCrossButtonPressed();
    /**
     * Whether the cross button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetCrossButtonReleased();


    /**
     * Read the value of the circle button on the controller.
     * @return The state of the button.
     */
    bool GetCircleButton() const;
    /**
     * Whether the circle button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetCircleButtonPressed();
    /**
     * Whether the circle button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetCircleButtonReleased();

    /**
     * Read the value of the triangle button on the controller.
     * @return The state of the button.
     */
    bool GetTriangleButton() const;
    /**
     * Whether the triangle button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetTriangleButtonPressed();
    /**
     * Whether the triangle button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetTriangleButtonReleased();

    /**
     * Read the value of the left trigger 1 button on the controller.
     * @return The state of the button.
     */
    bool GetL1Button() const;
    /**
     * Whether the left trigger 1 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetL1ButtonPressed();
    /**
     * Whether the left trigger 1 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetL1ButtonReleased();

    /**
     * Read the value of the right trigger 1 button on the controller.
     * @return The state of the button.
     */
    bool GetR1Button() const;
    /**
     * Whether the right trigger 1 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetR1ButtonPressed();
    /**
     * Whether the right trigger 1 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetR1ButtonReleased();

    /**
     * Read the value of the left trigger 2 button on the controller.
     * @return The state of the button.
     */
    bool GetL2AsButton() const;
    /**
     * Whether the left trigger 2 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetL2AsButtonPressed();
    /**
     * Whether the left trigger 2 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetL2AsButtonReleased();

    /**
     * Read the value of the right trigger 2 button on the controller.
     * @return The state of the button.
     */
    bool GetR2AsButton() const;
    /**
     * Whether the right trigger 2 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetR2AsButtonPressed();
    /**
     * Whether the right trigger 2 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetR2AsButtonReleased();

    /**
     * Read the value of the left trigger 3 button on the controller.
     * @return The state of the button.
     */
    bool GetL3AsButton() const;
    /**
     * Whether the left trigger 3 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetL3AsButtonPressed();
    /**
     * Whether the left trigger 3 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetL3AsButtonReleased();

    /**
     * Read the value of the right trigger 3 button on the controller.
     * @return The state of the button.
     */
    bool GetR3AsButton() const;
    /**
     * Whether the right trigger 3 button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetR3AsButtonPressed();
    /**
     * Whether the right trigger 3 button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetR3AsButtonReleased();

    /**
     * Read the value of the options button on the controller.
     * @return The state of the button.
     */
    bool GetOptionsButton() const;
    /**
     * Whether the options button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetOptionsButtonPressed();
    /**
     * Whether the options button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetOptionsButtonReleased();


    /**
     * Read the value of the Share button on the controller.
     * @return The state of the button.
     */
    bool GetShareButton() const;
    /**
     * Whether the Share button was pressed since the last check.
     * @return Whether the button was pressed since the last check.
     */
    bool GetShareButtonPressed();
    /**
     * Whether the Share button was released since the last check.
     * @return Whether the button was released since the last check.
     */
    bool GetShareButtonReleased();

    /**
     * Get the angle in degrees of a POV.
     * The POV angles start at 0 in the up direction, and increase clockwise
     * (e.g. right is 90, upper-left is 315).
     * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
     */
    int GetPOV();

      /**
     * Set the rumble output for the HID.
     * 
     * @param type  Which rumble value to set
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    void SetRumble(RumbleType type, double value);

    /**
     * @brief Checks if the RevGamepad is connected and matches the expected specifications.
     * 
     * This function verifies the connection status of the RevGamepad and ensures that
     * it has the expected number of axes, buttons, and POVs. The conditions checked are:
     * - The gamepad is connected.
     * - The gamepad has 6 axes.
     * - The gamepad has 10 buttons.
     * - The gamepad has 1 POV.
     * 
     * @return true if the RevGamepad is connected and meets the specifications, false otherwise.
     */
    bool IsRevGamepadConnected();

  /** Represents a digital button on a REV gamepad. */
  struct Button {
    /// Square button.
    static constexpr int kSquare = 3;
    /// Cross button.
    static constexpr int kCross = 1;
    /// Circle button.
    static constexpr int kCircle = 2;
    /// Triangle button.
    static constexpr int kTriangle = 4;
    /// Left trigger 1 button.
    static constexpr int kL1 = 5;
    /// Right trigger 1 button.
    static constexpr int kR1 = 6;
    /// Share button.
    static constexpr int kShare = 7;
    /// Options button.
    static constexpr int kOptions = 8;
    /// L3 (left stick) button.
    static constexpr int kL3 = 9;
    /// R3 (right stick) button.
    static constexpr int kR3 = 10;
  };

  /** Represents an axis on a REV gamepad. */
  struct Axis {
    /// Left X axis.
    static constexpr int kLeftX = 0;
    /// Left Y axis.
    static constexpr int kLeftY = 1;
    /// Right X axis.
    static constexpr int kRightX = 4;
    /// Right Y axis.
    static constexpr int kRightY = 5;
    /// Left trigger 2.
    static constexpr int kL2 = 2;
    /// Right trigger 2.
    static constexpr int kR2 = 3;
  };

  static constexpr HIDType type = HIDType::kXInputGamepad;
  
//   void InitSendable(wpi::SendableBuilder& builder) override; useless for now
    private :
        double m_thresholdAxisAsButton;

        bool L2AsButtonPressed = false;
        bool R2AsButtonPressed = false;


    protected :
      frc2::Trigger _squareButton{[this] { return GetSquareButton(); }};
      frc2::Trigger _crossButton{[this] { return GetCrossButton(); }};
      frc2::Trigger _circleButton{[this] { return GetCircleButton(); }};
      frc2::Trigger _triangleButton{[this] { return GetTriangleButton(); }};
      frc2::Trigger _L1Button{[this] { return GetL1Button(); }};
      frc2::Trigger _R1Button{[this] { return GetR1Button(); }};
      frc2::Trigger _L2AsButton{[this] { return GetL2AsButton(); }};
      frc2::Trigger _R2AsButton{[this] { return GetR2AsButton(); }};
      frc2::Trigger _L3AsButton{[this] { return GetL3AsButton(); }};
      frc2::Trigger _R3AsButton{[this] { return GetR3AsButton(); }};
      frc2::Trigger _optionsButton{[this] { return GetOptionsButton(); }};
      frc2::Trigger _shareButton{[this] { return GetShareButton(); }};

      frc2::Trigger _upPOVButton{[this] { return GetPOV() == 0; }};
      frc2::Trigger _rightPOVButton{[this] { return GetPOV() == 90; }};
      frc2::Trigger _downPOVButton{[this] { return GetPOV() == 180; }};
      frc2::Trigger _leftPOVButton{[this] { return GetPOV() == 270; }};
      frc2::Trigger _upRightPOVButton{[this] { return GetPOV() == 45; }};
      frc2::Trigger _downRightPOVButton{[this] { return GetPOV() == 135; }};
      frc2::Trigger _downLeftPOVButton{[this] { return GetPOV() == 225; }};
      frc2::Trigger _upLeftPOVButton{[this] { return GetPOV() == 315; }};
};
