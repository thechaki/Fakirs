#include "Aimbot/AimbotController.hpp"

#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>

namespace FakirBot::Aimbot
{
    void AimbotController::Initialize(
        Serial::ArduinoController* arduinoController
    ) noexcept
    {
        m_arduinoController = arduinoController;
        m_lastCommandTime = std::chrono::steady_clock::now();

        spdlog::info(
            "AimbotController initialized with ArduinoController."
        );
    }

    void AimbotController::Update(
        const Tracking::TargetDecision& targetDecision,
        const Overlay::OverlaySettings& overlaySettings,
        const int captureWidth,
        const int captureHeight
    ) noexcept
    {
        if (!m_arduinoController)
        {
            m_snapshot.aimLockActive = false;
            return;
        }

        m_snapshot.captureWidth = captureWidth;
        m_snapshot.captureHeight = captureHeight;
        m_snapshot.mouseDpi = overlaySettings.mouseDpi;
        m_snapshot.inGameSensitivity = overlaySettings.inGameSensitivity;
        m_snapshot.sensitivity = overlaySettings.aimLockSensitivity;

        const cv::Point2f screenCenter(
            static_cast<float>(captureWidth) * 0.5F,
            static_cast<float>(captureHeight) * 0.5F
        );

        m_snapshot.screenCenter = screenCenter;

        const bool isLocked =
            targetDecision.state == Tracking::TargetState::Locked ||
            targetDecision.state == Tracking::TargetState::Tracking;

        const bool shouldAim =
            overlaySettings.aimLockEnabled &&
            isLocked &&
            targetDecision.hasTarget &&
            targetDecision.reliability >= 0.40F;

        spdlog::debug(
            "AIM_UPDATE: aimLockEnabled={}, isLocked={}, hasTarget={}, reliability={:.2f}, shouldAim={}",
            overlaySettings.aimLockEnabled,
            isLocked,
            targetDecision.hasTarget,
            targetDecision.reliability,
            shouldAim
        );

        m_snapshot.aimLockActive = shouldAim;
        m_snapshot.aimLockKeyPressed = false;
        m_snapshot.hasValidTarget = targetDecision.hasTarget;
        m_snapshot.reliability = targetDecision.reliability;

        if (!shouldAim)
        {
            m_lastSentPoint = screenCenter;
            return;
        }

        m_snapshot.targetPoint = targetDecision.filteredPoint;

        const cv::Point2f delta =
            CalculateScreenDelta(
                targetDecision.filteredPoint,
                screenCenter
            );

        spdlog::debug(
            "AIM_DELTA: filtered=({:.1f},{:.1f}) center=({:.1f},{:.1f}) delta=({:.1f},{:.1f})",
            targetDecision.filteredPoint.x,
            targetDecision.filteredPoint.y,
            screenCenter.x,
            screenCenter.y,
            delta.x,
            delta.y
        );

        const float deltaLength = std::sqrt(
            delta.x * delta.x +
            delta.y * delta.y
        );

        if (deltaLength < kMinimumDeltaPixels)
        {
            spdlog::debug("AIM: Delta length too small: {:.2f}", deltaLength);
            return;
        }

        const auto now =
            std::chrono::steady_clock::now();

        const auto elapsedMs =
            std::chrono::duration_cast<
            std::chrono::milliseconds
            >(now - m_lastCommandTime).count();

        if (elapsedMs < kMinimumCommandDelayMs)
        {
            return;
        }

        const float sensitivity =
            CalculateMouseSensitivity(
                overlaySettings.mouseDpi,
                overlaySettings.inGameSensitivity,
                overlaySettings.aimLockSensitivity
            );

        spdlog::debug(
            "AIM_SETTINGS - DPI: {}, InGame: {:.2f}, Overlay: {:.2f}, Sensitivity: {:.6f}",
            overlaySettings.mouseDpi,
            overlaySettings.inGameSensitivity,
            overlaySettings.aimLockSensitivity,
            sensitivity
        );

        SendMouseMoveToArduino(
            delta.x,
            delta.y,
            sensitivity
        );

        m_lastCommandTime = now;
        m_lastSentPoint = targetDecision.filteredPoint;
    }

    AimbotSnapshot AimbotController::GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    bool AimbotController::IsInitialized() const noexcept
    {
        return m_arduinoController != nullptr;
    }

    void AimbotController::SendMouseMoveToArduino(
        const float deltaX,
        const float deltaY,
        const float sensitivity
    ) noexcept
    {
        if (!m_arduinoController)
        {
            spdlog::error("Arduino controller is null");
            return;
        }

        spdlog::debug(
            "AIM_RAW_DELTA: X={:.2f}px, Y={:.2f}px",
            deltaX,
            deltaY
        );

        spdlog::debug(
            "AIM_SENSITIVITY: {:.6f}",
            sensitivity
        );

        // Apply sensitivity multiplier
        const int16_t mouseX =
            static_cast<int16_t>(
                std::clamp(
                    static_cast<int>(deltaX * sensitivity),
                    static_cast<int>(INT16_MIN),
                    static_cast<int>(INT16_MAX)
                )
            );

        const int16_t mouseY =
            static_cast<int16_t>(
                std::clamp(
                    static_cast<int>(deltaY * sensitivity),
                    static_cast<int>(INT16_MIN),
                    static_cast<int>(INT16_MAX)
                )
            );

        spdlog::info(
            "AIM_SEND_MOUSE: deltaX={} counts, deltaY={} counts (from {:.2f}px, {:.2f}px × {:.6f})",
            mouseX,
            mouseY,
            deltaX,
            deltaY,
            sensitivity
        );

        // Send to Arduino
        const bool sent = m_arduinoController->SendMouseMove(
            mouseX,
            mouseY
        );

        if (!sent)
        {
            spdlog::warn(
                "Failed to send mouse move command to Arduino"
            );
            return;
        }

        spdlog::debug("AIM: Mouse move command sent to Arduino");
    }

    float AimbotController::CalculateMouseSensitivity(
        const int dpi,
        const float inGameSensitivity,
        const float aimLockSensitivity
    ) const noexcept
    {
        constexpr float referenceInGameSensitivity = 1.0F;
        constexpr float referenceDpi = 800.0F;
        constexpr float referenceOverlaySensitivity = 1.0F;

        // DPI Scale
        const float dpiScale =
            static_cast<float>(dpi) / referenceDpi;

        spdlog::debug(
            "AIM_DPI_SCALE: {} DPI / {} = {:.4f}",
            dpi,
            static_cast<int>(referenceDpi),
            dpiScale
        );

        // In-game Sensitivity Scale
        const float ingameScale =
            inGameSensitivity / referenceInGameSensitivity;

        spdlog::debug(
            "AIM_INGAME_SCALE: {:.2f} / {:.2f} = {:.4f}",
            inGameSensitivity,
            referenceInGameSensitivity,
            ingameScale
        );

        // Overlay Sensitivity Multiplier
        const float overlayMultiplier =
            aimLockSensitivity / referenceOverlaySensitivity;

        spdlog::debug(
            "AIM_OVERLAY_MULT: {:.2f} / {:.2f} = {:.4f}",
            aimLockSensitivity,
            referenceOverlaySensitivity,
            overlayMultiplier
        );

        // Base Sensitivity
        const float baseSensitivity =
            dpiScale * ingameScale * overlayMultiplier;

        // DPI Dampening Factor
        constexpr float dpiDampeningFactor = 0.25F;

        // Final Sensitivity
        const float finalSensitivity =
            baseSensitivity * dpiDampeningFactor;

        spdlog::debug(
            "AIM_SENSITIVITY_CALC: DPI={:.4f} × InGame={:.4f} × Overlay={:.4f} × Dampening={:.4f} = FINAL={:.6f}",
            dpiScale,
            ingameScale,
            overlayMultiplier,
            dpiDampeningFactor,
            finalSensitivity
        );

        return finalSensitivity;
    }

    cv::Point2f AimbotController::CalculateScreenDelta(
        const cv::Point2f& targetPoint,
        const cv::Point2f& screenCenter
    ) const noexcept
    {
        return cv::Point2f(
            targetPoint.x - screenCenter.x,
            targetPoint.y - screenCenter.y
        );
    }
}
