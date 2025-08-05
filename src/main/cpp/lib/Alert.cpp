#include "lib/Alert.h"

#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Errors.h>

std::unordered_map<std::string, std::shared_ptr<Alert::SendableAlerts>> Alert::groups;

Alert::Alert(const std::string& text, AlertType type)
    : Alert("Alerts", text, type) {}

Alert::Alert(const std::string& group, const std::string& text, AlertType type)
    : type(type), text(text) {
    if (groups.find(group) == groups.end()) {
        std::shared_ptr<SendableAlerts> sendable = std::make_shared<SendableAlerts>();
        frc::SmartDashboard::PutData(group, sendable.get());
        groups[group] = sendable;
    }

    groups[group]->alerts.push_back(this);
}

void Alert::Set(bool newActive) {
    if (newActive && !active) {
        activeStartTime = frc::Timer::GetFPGATimestamp().value();
        switch (type) {
            case AlertType::ERROR:
                FRC_ReportError(frc::err::Error, text.c_str());
                break;
            case AlertType::WARNING:
                FRC_ReportError(frc::warn::Warning, text.c_str());
                break;
            case AlertType::INFO:
                // The INFO type is only available for read in NetworkTables.
                break;
            case AlertType::PRINT:
                std::cout << text << "\n";
                break;
        }
    }
    active = newActive;
}

void Alert::SetText(const std::string& newText) {
    if (active && newText != text) {
        switch (type) {
            case AlertType::ERROR:
                FRC_ReportError(frc::err::Error, newText.c_str());
                break;
            case AlertType::WARNING:
                FRC_ReportError(frc::warn::Warning, newText.c_str());
                break;
            case AlertType::INFO:
                // The INFO type is only available for read in NetworkTables.
                break;
            case AlertType::PRINT:
                std::cout << newText << "\n";
                break;
        }
    }
    text = newText;
}

std::vector<std::string> Alert::SendableAlerts::GetStrings(AlertType type) {
    std::vector<Alert*> filtered;

    for (std::size_t i = 0; i < alerts.size(); ++i) {
        if (alerts[i]->type == type && alerts[i]->active) {
            filtered.push_back(alerts[i]);
        }
    }

    std::sort(filtered.begin(), filtered.end(), [](Alert* a1, Alert* a2) -> bool {
        return a2->activeStartTime < a1->activeStartTime;
    });

    std::vector<std::string> result;
    for (std::size_t i = 0; i < filtered.size(); ++i) {
        result.push_back(filtered[i]->text);
    }
    return result;
}

void Alert::SendableAlerts::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("Alerts");

    builder.AddStringArrayProperty(
        "errors",
        [this]() -> std::vector<std::string> {
            return GetStrings(Alert::AlertType::ERROR);
        },
        nullptr
    );

    builder.AddStringArrayProperty(
        "warnings",
        [this]() -> std::vector<std::string> {
            return GetStrings(Alert::AlertType::WARNING);
        },
        nullptr
    );

    builder.AddStringArrayProperty(
        "infos",
        [this]() -> std::vector<std::string> {
            return GetStrings(Alert::AlertType::INFO);
        },
        nullptr
    );

    builder.AddStringArrayProperty(
        "prints",
        [this]() -> std::vector<std::string> {
            return GetStrings(Alert::AlertType::PRINT);
        },
        nullptr
    );
}