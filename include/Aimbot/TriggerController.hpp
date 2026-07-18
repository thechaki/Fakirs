#pragma once

#include "Tracking/AimStateMachine.hpp"
#include "Overlay/OverlaySettings.hpp"
#include "Serial/ArduinoController.hpp"
#include "WeaponTracking/WeaponTrackingService.hpp"

#include <chrono>
#include <cstdint>

namespace FakirBot::Aimbot
{
    struct TriggerSnapshot final
    {
        bool triggerActive{ false };
        bool triggerKeyPressed{ false };

        bool hasValidTarget{ false };
        float targetReliability{ 0.0F };

        std::string activeWeapon{ "UNKNOWN" };
        std::string fireMode{ "Unknown" };
        int rpm{ 0 };

        uint32_t burstCount{ 0 };
        uint32_t roundsFired{ 0 };
        uint32_t lastTriggerTimeMs{ 0 };
    };

    class TriggerController final
    {
    public:
        TriggerController() = default;
        ~TriggerController() = default;

        void Initialize(
            Serial::ArduinoController* arduinoController,
            WeaponTracking::WeaponTrackingService* weaponTrackingService
        ) noexcept;

        void Update(
            const Tracking::TargetDecision& targetDecision,
            const Overlay::OverlaySettings& overlaySettings,
            bool triggerKeyIsPressed
        ) noexcept;

        [[nodiscard]]
        TriggerSnapshot GetSnapshot() const noexcept;

        [[nodiscard]]
        bool IsInitialized() const noexcept;

    private:
        void ExecuteTrigger(
            const std::string& fireMode,
            int rpm
        ) noexcept;

        void SendMouseClick() noexcept;

        void SendMousePress() noexcept;

        void SendMouseRelease() noexcept;

        [[nodiscard]]
        bool ShouldTrigger(
            const Tracking::TargetDecision& targetDecision,
            const Overlay::OverlaySettings& overlaySettings,
            bool triggerKeyPressed
        ) const noexcept;

        [[nodiscard]]
        uint32_t CalculateShotIntervalMs(
            int rpm
        ) const noexcept;

    private:
        Serial::ArduinoController* m_arduinoController{ nullptr };
        WeaponTracking::WeaponTrackingService* m_weaponTrackingService{ nullptr };

        TriggerSnapshot m_snapshot{};

        std::chrono::steady_clock::time_point m_lastTriggerTime{};
        std::chrono::steady_clock::time_point m_reactionDelayStart{};
        std::chrono::steady_clock::time_point m_lastBurstTime{};

        bool m_triggerKeyWasPressed{ false };
        bool m_mouseButtonPressed{ false };
        bool m_reactionDelayActive{ false };
        uint32_t m_burstCounter{ 0 };

        static constexpr int kMouseLeftButton = 1;
    };
}
