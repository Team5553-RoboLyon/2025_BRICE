/*******************************************************************************
 * 
 * File        : Alert.h (1.0)
 * Library     : LyonLib (from 2025_BRICE)
 * Description : Persistent alert system for displaying categorized runtime messages
 *               to the SmartDashboard using NetworkTables.
 * 
 * Authors     : AKA (2025), last update by AKA (2025)
 * Organization: Robo'Lyon - FRC Team 5553
 *               Lycée Notre-Dame-de-Bellegarde, France
 * Repository  : https://github.com/Team5553-RoboLyon
 * 
 *******************************************************************************/

#pragma once

#include <string>
#include <vector>
#include "wpi/sendable/Sendable.h"
#include "wpi/sendable/SendableBuilder.h"
#include <unordered_map>

class Alert {
public:
    enum class AlertType {
        ERROR,
        WARNING,
        INFO
    };

    Alert(const std::string& text, AlertType type);
    Alert(const std::string& group, const std::string& text, AlertType type);

    void Set(bool active);
    void SetText(const std::string& text);

private:
    class SendableAlerts : public wpi::Sendable {
    public:
        std::vector<Alert*> alerts;

        std::vector<std::string> GetStrings(AlertType type);
        void InitSendable(wpi::SendableBuilder& builder) override;
    };

    static std::unordered_map<std::string, std::shared_ptr<SendableAlerts>> groups;

    AlertType type;
    bool active = false;
    double activeStartTime = 0.0;
    std::string text;
};