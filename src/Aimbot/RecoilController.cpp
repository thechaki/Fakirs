#include "Aimbot/RecoilController.hpp"

#include <algorithm>
#include <cmath>
#include <spdlog/spdlog.h>

namespace FakirBot::Aimbot
{
    void RecoilController::Initialize(
        Serial::ArduinoController* arduinoController,
        WeaponTracking::WeaponTrackingService* weaponTrackingService
    ) noexcept
    {
        m_arduinoController = arduinoController;
        m_weaponTrackingService = weaponTrackingService;

        spdlog::info(
            "RecoilController initialized with Arduino and WeaponTracking."
        );
    }

    void RecoilController::Update(
        const Tracking::TargetDecision& targetDecision,
        const Overlay::OverlaySettings& overlaySettings,
        const bool triggerKeyIsPressed,
        const bool mouseButtonDown
    ) noexcept
    {
        if (!m_arduinoController || !m_weaponTrackingService)
        {
            m_snapshot.recoilActive = false;
            return;
        }

        const auto weaponSnapshot =
            m_weaponTrackingService->GetSnapshot();

        m_snapshot.activeWeapon = weaponSnapshot.active.displayName;
        m_snapshot.patternPointCount =
            static_cast<uint32_t>(
                weaponSnapshot.active.patternPointCount
            );
        m_snapshot.compensationStrength =
            overlaySettings.compensationStrengthPercent / 100.0F;
        m_snapshot.horizontalStrength =
            overlaySettings.horizontalStrengthPercent / 100.0F;
        m_snapshot.verticalStrength =
            overlaySettings.verticalStrengthPercent / 100.0F;
        m_snapshot.outputLimit = overlaySettings.outputLimitUnits;
        m_lastWeaponTimingMs =
            weaponSnapshot.active.timing.milliseconds;

        // Check if recoil should be active
        const bool shouldApplyRecoil =
            overlaySettings.recoilAssistEnabled &&
            targetDecision.hasTarget &&
            !weaponSnapshot.active.patternPoints.empty() &&
            (mouseButtonDown || triggerKeyIsPressed);

        // Handle session start
        if (shouldApplyRecoil && !m_sessionActive)
        {
            StartSession();
        }
        // Handle session end
        else if (!shouldApplyRecoil && m_sessionActive)
        {
            EndSession();
        }

        m_snapshot.recoilActive = shouldApplyRecoil;
        m_snapshot.sessionActive = m_sessionActive;

        // Apply recoil pattern if session is active
        if (m_sessionActive && shouldApplyRecoil)
        {
            ApplyRecoilPattern(overlaySettings);
        }

        // Update button state tracking
        m_mouseButtonWasDown = mouseButtonDown;
        m_triggerKeyWasPressed = triggerKeyIsPressed;
    }

    RecoilSnapshot RecoilController::GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    bool RecoilController::IsInitialized() const noexcept
    {
        return m_arduinoController != nullptr &&
               m_weaponTrackingService != nullptr;
    }

    void RecoilController::Reset() noexcept
    {
        EndSession();
        m_currentPatternIndex = 0;
        m_snapshot = RecoilSnapshot{};
    }

    void RecoilController::StartSession() noexcept
    {
        if (m_sessionActive)
        {
            return;
        }

        m_sessionActive = true;
        m_sessionStart = std::chrono::steady_clock::now();
        m_lastPatternApplyTime = m_sessionStart;
        m_currentPatternIndex = 0;
        m_snapshot.totalPointsApplied = 0;
        m_snapshot.currentPatternIndex = 0;

        spdlog::info(
            "RECOIL_SESSION: Started | Weapon: {}",
            m_snapshot.activeWeapon
        );
    }

    void RecoilController::EndSession() noexcept
    {
        if (!m_sessionActive)
        {
            return;
        }

        m_sessionActive = false;
        m_currentPatternIndex = 0;
        m_snapshot.currentPatternIndex = 0;
        m_snapshot.lastDeltaX = 0.0F;
        m_snapshot.lastDeltaY = 0.0F;

        spdlog::info(
            "RECOIL_SESSION: Ended | Total points applied: {}",
            m_snapshot.totalPointsApplied
        );
    }

    void RecoilController::ApplyRecoilPattern(
        const Overlay::OverlaySettings& overlaySettings
    ) noexcept
    {
        const auto weaponSnapshot =
            m_weaponTrackingService->GetSnapshot();

        if (weaponSnapshot.active.patternPoints.empty())
        {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - m_sessionStart
            ).count();

        // Check if start delay has passed
        if (elapsedMs < static_cast<long long>(
            overlaySettings.recoilStartDelayMs
        ))
        {
            return; // Still in start delay
        }

        const uint32_t patternIntervalMs =
            GetPatternIntervalMs();

        if (patternIntervalMs == 0)
        {
            return; // Invalid timing
        }

        // Calculate which pattern point should be applied
        const auto timeSinceStartDelay = elapsedMs -
            static_cast<long long>(
                overlaySettings.recoilStartDelayMs
            );

        const auto nextPatternIndex =
            static_cast<uint32_t>(
                timeSinceStartDelay / static_cast<long long>(patternIntervalMs)
            );

        // Check if we need to apply the next pattern point
        if (nextPatternIndex <= m_currentPatternIndex)
        {
            return; // Not time for next point yet
        }

        // Don't go beyond pattern points
        if (nextPatternIndex >= static_cast<uint32_t>(
            weaponSnapshot.active.patternPoints.size()
        ))
        {
            // Pattern exhausted, loop back to beginning
            m_currentPatternIndex =
                static_cast<uint32_t>(
                    weaponSnapshot.active.patternPoints.size() - 1
                );
            return;
        }

        m_currentPatternIndex = nextPatternIndex;
        m_snapshot.currentPatternIndex = m_currentPatternIndex;

        // Get the current pattern point
        const auto& patternPoint =
            weaponSnapshot.active.patternPoints[m_currentPatternIndex];

        // Calculate inverse pattern with compensation
        const auto [deltaX, deltaY] = CalculateInversePattern(
            static_cast<float>(patternPoint.x),
            static_cast<float>(patternPoint.y),
            overlaySettings
        );

        // Store for snapshot
        m_snapshot.lastDeltaX = deltaX;
        m_snapshot.lastDeltaY = deltaY;
        m_snapshot.lastAppliedTimeMs = static_cast<uint32_t>(
            elapsedMs
        );

        // Send to Arduino
        SendRecoilDelta(deltaX, deltaY);

        m_snapshot.totalPointsApplied++;

        spdlog::debug(
            "RECOIL_POINT: Index={} | Source=({},{}) | "
            "Inverse=({:.2f},{:.2f}) | Interval={}ms | "
            "ElapsedMs={}",
            m_currentPatternIndex,
            patternPoint.x,
            patternPoint.y,
            deltaX,
            deltaY,
            patternIntervalMs,
            elapsedMs
        );
    }

    void RecoilController::SendRecoilDelta(
        const float deltaX,
        const float deltaY
    ) noexcept
    {
        if (!m_arduinoController)
        {
            spdlog::error("Arduino controller is null");
            return;
        }

        const int moveX = static_cast<int>(std::round(deltaX));
        const int moveY = static_cast<int>(std::round(deltaY));

        const bool sent = m_arduinoController->SendMouseMove(
            moveX,
            moveY
        );

        if (!sent)
        {
            spdlog::warn(
                "Failed to send recoil delta to Arduino: ({}, {})",
                moveX,
                moveY
            );
        }
        else
        {
            spdlog::debug(
                "Recoil delta sent to Arduino: ({}, {})",
                moveX,
                moveY
            );
        }
    }

    std::pair<float, float> RecoilController::CalculateInversePattern(
        const float patternX,
        const float patternY,
        const Overlay::OverlaySettings& overlaySettings
    ) const noexcept
    {
        // Get compensation factors from overlay settings
        const float globalCompensation =
            overlaySettings.compensationStrengthPercent / 100.0F;

        const float horizontalMultiplier =
            overlaySettings.horizontalStrengthPercent / 100.0F;

        const float verticalMultiplier =
            overlaySettings.verticalStrengthPercent / 100.0F;

        // Calculate inverse pattern: negate and apply multipliers
        float inverseX =
            -patternX *
            globalCompensation *
            horizontalMultiplier;

        float inverseY =
            -patternY *
            globalCompensation *
            verticalMultiplier;

        // Apply output limit clamping
        const float outputLimit = overlaySettings.outputLimitUnits;

        inverseX = std::clamp(
            inverseX,
            -outputLimit,
            outputLimit
        );

        inverseY = std::clamp(
            inverseY,
            -outputLimit,
            outputLimit
        );

        return { inverseX, inverseY };
    }

    uint32_t RecoilController::GetPatternIntervalMs() const noexcept
    {
        if (m_lastWeaponTimingMs == 0)
        {
            spdlog::warn(
                "Invalid weapon timing: {} ms",
                m_lastWeaponTimingMs
            );
            return 100; // Default fallback
        }

        return m_lastWeaponTimingMs;
    }
}
