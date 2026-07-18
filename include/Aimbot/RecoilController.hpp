#pragma once

#include "Tracking/AimStateMachine.hpp"
#include "Overlay/OverlaySettings.hpp"
#include "Serial/ArduinoController.hpp"
#include "WeaponTracking/WeaponTrackingService.hpp"

#include <chrono>
#include <cstdint>
#include <vector>

namespace FakirBot::Aimbot
{
    struct RecoilSnapshot final
    {
        bool recoilActive{ false };
        bool sessionActive{ false };

        std::string activeWeapon{ "UNKNOWN" };
        uint32_t patternPointCount{ 0 };
        uint32_t currentPatternIndex{ 0 };
        uint32_t totalPointsApplied{ 0 };

        float lastDeltaX{ 0.0F };
        float lastDeltaY{ 0.0F };
        uint32_t lastAppliedTimeMs{ 0 };

        float compensationStrength{ 0.0F };
        float horizontalStrength{ 0.0F };
        float verticalStrength{ 0.0F };
        float outputLimit{ 0.0F };
    };

    class RecoilController final
    {
    public:
        RecoilController() = default;
        ~RecoilController() = default;

        void Initialize(
            Serial::ArduinoController* arduinoController,
            WeaponTracking::WeaponTrackingService* weaponTrackingService
        ) noexcept;

        void Update(
            const Tracking::TargetDecision& targetDecision,
            const Overlay::OverlaySettings& overlaySettings,
            bool triggerKeyIsPressed,
            bool mouseButtonDown
        ) noexcept;

        [[nodiscard]]
        RecoilSnapshot GetSnapshot() const noexcept;

        [[nodiscard]]
        bool IsInitialized() const noexcept;

        void Reset() noexcept;

    private:
        void StartSession() noexcept;

        void EndSession() noexcept;

        void ApplyRecoilPattern(
            const Overlay::OverlaySettings& overlaySettings
        ) noexcept;

        void SendRecoilDelta(
            float deltaX,
            float deltaY
        ) noexcept;

        [[nodiscard]]
        std::pair<float, float> CalculateInversePattern(
            float patternX,
            float patternY,
            const Overlay::OverlaySettings& overlaySettings
        ) const noexcept;

        [[nodiscard]]
        uint32_t GetPatternIntervalMs() const noexcept;

    private:
        Serial::ArduinoController* m_arduinoController{ nullptr };
        WeaponTracking::WeaponTrackingService* m_weaponTrackingService{ nullptr };

        RecoilSnapshot m_snapshot{};

        std::chrono::steady_clock::time_point m_sessionStart{};
        std::chrono::steady_clock::time_point m_lastPatternApplyTime{};

        bool m_sessionActive{ false };
        bool m_mouseButtonWasDown{ false };
        bool m_triggerKeyWasPressed{ false };
        uint32_t m_currentPatternIndex{ 0 };
        uint32_t m_lastWeaponTimingMs{ 0 };

        static constexpr float kMinPatternValue = -10.0F;
        static constexpr float kMaxPatternValue = 10.0F;
    };
}
