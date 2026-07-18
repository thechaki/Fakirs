#pragma once

#include "Overlay/OverlaySettings.hpp"
#include "Tracking/AimStateMachine.hpp"
#include "Serial/ArduinoController.hpp"

#include <cstdint>
#include <optional>
#include <chrono>

#include <opencv2/core/types.hpp>

namespace FakirBot::Aimbot
{
    struct AimbotSnapshot final
    {
        bool aimLockActive{ false };
        bool aimLockKeyPressed{ false };
        float sensitivity{ 0.60F };

        int mouseDpi{ 800 };
        float inGameSensitivity{ 1.0F };

        cv::Point2f targetPoint{};
        cv::Point2f screenCenter{};

        bool hasValidTarget{ false };
        float reliability{ 0.0F };

        int captureWidth{ 640 };
        int captureHeight{ 384 };
    };

    class AimbotController final
    {
    public:
        AimbotController() = default;
        ~AimbotController() = default;

        void Initialize(
            Serial::ArduinoController* arduinoController
        ) noexcept;

        void Update(
            const Tracking::TargetDecision& targetDecision,
            const Overlay::OverlaySettings& overlaySettings,
            int captureWidth,
            int captureHeight
        ) noexcept;

        [[nodiscard]]
        AimbotSnapshot GetSnapshot() const noexcept;

        [[nodiscard]]
        bool IsInitialized() const noexcept;

    private:
        void SendMouseMoveToArduino(
            float deltaX,
            float deltaY,
            float sensitivity
        ) noexcept;

        [[nodiscard]]
        float CalculateMouseSensitivity(
            int dpi,
            float inGameSensitivity,
            float aimLockSensitivity
        ) const noexcept;

        [[nodiscard]]
        cv::Point2f CalculateScreenDelta(
            const cv::Point2f& targetPoint,
            const cv::Point2f& screenCenter
        ) const noexcept;

    private:
        Serial::ArduinoController* m_arduinoController{ nullptr };

        AimbotSnapshot m_snapshot{};

        std::chrono::steady_clock::time_point m_lastCommandTime{};

        cv::Point2f m_lastSentPoint{};

        static constexpr float kMinimumDeltaPixels = 0.5F;
        static constexpr std::uint32_t kMinimumCommandDelayMs = 1;
    };
}
