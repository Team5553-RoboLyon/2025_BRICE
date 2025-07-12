#include "lib/RevGamepad.h"

class Operator final : public RevGamepad
{

#if (OPERATOR == (ADAM))
private:

    frc2::Trigger m_advanceMode{_shareButton};
    frc2::Trigger m_leftReefButton{_L1Button && (!_R1Button) && (!m_advanceMode)};
    frc2::Trigger m_rightReefButton{_R1Button && (!_L1Button) && (!m_advanceMode)};
public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger stageCoralStationButton{(_downPOVButton || _downRightPOVButton || _downLeftPOVButton) && (!m_advanceMode)};
    frc2::Trigger stageHomeButton{_crossButton && (!m_advanceMode)};
    frc2::Trigger stageL1Button{_optionsButton && (!m_advanceMode)};
    frc2::Trigger stageL2Button{_circleButton && (!m_advanceMode)};
    frc2::Trigger stageL3Button{_squareButton && (!m_advanceMode)};
    frc2::Trigger stageL4Button{_triangleButton && (!m_advanceMode)};
    frc2::Trigger stageL2AButton{stageL2Button && m_leftReefButton};
    frc2::Trigger stageL2BButton{stageL2Button && m_rightReefButton};
    frc2::Trigger stageL3AButton{stageL3Button && m_leftReefButton};
    frc2::Trigger stageL3BButton{stageL3Button && m_rightReefButton};
    frc2::Trigger stageL4AButton{stageL4Button && m_leftReefButton};
    frc2::Trigger stageL4BButton{stageL4Button && m_rightReefButton};

    frc2::Trigger toggleAlignAssistButton{_L1Button && _R1Button && (!m_advanceMode)};
    frc2::Trigger toggleScoreAssistButton{_L2AsButton && _R2AsButton && (!m_advanceMode)};
    frc2::Trigger toggleAssistModeButton{_triangleButton && m_advanceMode};

    // frc2::Trigger rejectCoralFrontButton{_circleButton && m_advanceMode};
    // frc2::Trigger rejectCoralBackButton{_squareButton && m_advanceMode};
    // frc2::Trigger defenseButtonButton{_crossButton && m_advanceMode};
    // frc2::Trigger goodbyeAlgaeButton{_upPOVButton || _upRightPOVButton || _upLeftPOVButton};
    // frc2::Trigger leCasseDeBriceButton{(_downPOVButton || _downRightPOVButton || _downLeftPOVButton) && m_advanceMode}; 

    // frc2::Trigger toggleGripperManualControlButton{_optionsButton && m_advanceMode};
    // frc2::Trigger toggleElevatorManualControlButton{_L3AsButton};
    // frc2::Trigger toggleStrafferManualControlButton{_R3AsButton};

    frc2::Trigger scoreButton{_R2AsButton && (!_L2AsButton) && (!m_advanceMode)};
    frc2::Trigger intakeButton{_L2AsButton && (!_R2AsButton) && (!m_advanceMode)};
#else //elif (OPERATOR == (VICTOR))
private:

    frc2::Trigger m_advanceMode{_shareButton};
    frc2::Trigger m_leftReefButton{_L1Button && (!_R1Button) && (!m_advanceMode)};
    frc2::Trigger m_rightReefButton{_R1Button && (!_L1Button) && (!m_advanceMode)};
public:
    Operator(int port) : RevGamepad(port){};
    Operator(int port, double threshold) : RevGamepad(port, threshold){};
    ~Operator() override = default;

    frc2::Trigger stageCoralStationButton{_crossButton && (!m_advanceMode)};
    frc2::Trigger stageHomeButton{(_downPOVButton || _downRightPOVButton || _downLeftPOVButton) && (!m_advanceMode)}; 
    frc2::Trigger stageL1Button{_optionsButton && (!m_advanceMode)};
    frc2::Trigger stageL2Button{_circleButton && (!m_advanceMode)};
    frc2::Trigger stageL3Button{_squareButton && (!m_advanceMode)};
    frc2::Trigger stageL4Button{_triangleButton && (!m_advanceMode)};
    frc2::Trigger stageL2AButton{stageL2Button && m_leftReefButton};
    frc2::Trigger stageL2BButton{stageL2Button && m_rightReefButton};
    frc2::Trigger stageL3AButton{stageL3Button && m_leftReefButton};
    frc2::Trigger stageL3BButton{stageL3Button && m_rightReefButton};
    frc2::Trigger stageL4AButton{stageL4Button && m_leftReefButton};
    frc2::Trigger stageL4BButton{stageL4Button && m_rightReefButton};

    frc2::Trigger toggleAlignAssistButton{_L1Button && _R1Button && (!m_advanceMode)};
    frc2::Trigger toggleScoreAssistButton{_L2AsButton && _R2AsButton && (!m_advanceMode)};
    frc2::Trigger toggleAssistModeButton{_triangleButton && m_advanceMode};

    // frc2::Trigger rejectCoralFrontButton{_circleButton && m_advanceMode};
    // frc2::Trigger rejectCoralBackButton{_squareButton && m_advanceMode};
    // frc2::Trigger defenseButtonButton{_crossButton && m_advanceMode};
    // frc2::Trigger goodbyeAlgaeButton{_upPOVButton || _upRightPOVButton || _upLeftPOVButton};
    // frc2::Trigger leCasseDeBriceButton{(_downPOVButton || _downRightPOVButton || _downLeftPOVButton) && m_advanceMode}; 

    // frc2::Trigger toggleGripperManualControlButton{_optionsButton && m_advanceMode};
    // frc2::Trigger toggleElevatorManualControlButton{_L3AsButton};
    // frc2::Trigger toggleStrafferManualControlButton{_R3AsButton};

    frc2::Trigger scoreButton{_R2AsButton && (!_L2AsButton) && (!m_advanceMode)};
    frc2::Trigger intakeButton{_L2AsButton && (!_R2AsButton) && (!m_advanceMode)};
#endif


    void SetRumble(RumbleType type, double value)
    {
        RevGamepad::SetRumble(type, value);
    }
    void SetRumble(double value)
    {
        RevGamepad::SetRumble(RumbleType::kBothRumble, value);
    }
    void SetRumble(RumbleType type)
    {
        RevGamepad::SetRumble(type, 1.0);
    }

};