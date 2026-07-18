#include "Aimbot/TriggerController.hpp"

#include <algorithm>
#include <spdlog/spdlog.h>

namespace FakirBot::Aimbot
{
    void TriggerController::Initialize(
        Serial::ArduinoController* arduinoController,
        WeaponTracking::WeaponTrackingService* weaponTrackingService
    ) noexcept
    {
        m_arduinoController = arduinoController;
        m_weaponTrackingService = weaponTrackingService;
        m_lastTriggerTime = std::chrono::steady_clock::now();

        spdlog::info(
            "TriggerController initialized with Arduino and WeaponTracking."
        );
    }

    void TriggerController::Update(
        const Tracking::TargetDecision& targetDecision,
        const Overlay::OverlaySettings& overlaySettings,
        const bool triggerKeyIsPressed
    ) noexcept
    {
        if (!m_arduinoController || !m_weaponTrackingService)
        {
            m_snapshot.triggerActive = false;
            return;
        }

        // Get current weapon info
        const auto weaponSnapshot =
            m_weaponTrackingService->GetSnapshot();

        m_snapshot.activeWeapon = weaponSnapshot.active.displayName;
        m_snapshot.fireMode = weaponSnapshot.active.timing.kind;
        m_snapshot.rpm = weaponSnapshot.active.timing.rpm;

        // Check if we should trigger
        const bool shouldTrigger =
            ShouldTrigger(
                targetDecision,
                overlaySettings,
                triggerKeyIsPressed
            );

        m_snapshot.triggerActive = shouldTrigger;
        m_snapshot.triggerKeyPressed = triggerKeyIsPressed;
        m_snapshot.hasValidTarget = targetDecision.hasTarget;
        m_snapshot.targetReliability = targetDecision.reliability;

        spdlog::debug(
            "TRIGGER_UPDATE: keyPressed={}, shouldTrigger={}, weapon={}, fireMode={}, rpm={}",
            triggerKeyIsPressed,
            shouldTrigger,
            m_snapshot.activeWeapon,
            m_snapshot.fireMode,
            m_snapshot.rpm
        );

        if (!shouldTrigger)
        {
            // Key released - release mouse button if pressed
            if (m_triggerKeyWasPressed && m_mouseButtonPressed)
            {
                SendMouseRelease();
                m_mouseButtonPressed = false;
                spdlog::info("TRIGGER: Mouse button released (key released)");
            }

            m_triggerKeyWasPressed = triggerKeyIsPressed;
            m_reactionDelayActive = false;
            m_burstCounter = 0;
            return;
        }

        // Trigger key just pressed
        if (triggerKeyIsPressed && !m_triggerKeyWasPressed)
        {
            m_reactionDelayActive = true;
            m_reactionDelayStart = std::chrono::steady_clock::now();
            m_burstCounter = 0;

            spdlog::info(
                "TRIGGER: Key pressed, reaction delay started: {}ms",
                overlaySettings.triggerReactionDelayMs
            );
        }

        m_triggerKeyWasPressed = triggerKeyIsPressed;

        // Handle reaction delay
        if (m_reactionDelayActive)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedMs =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - m_reactionDelayStart
                ).count();

            if (elapsedMs < overlaySettings.triggerReactionDelayMs)
            {
                return; // Still waiting
            }

            m_reactionDelayActive = false;
            spdlog::debug("TRIGGER: Reaction delay complete");
        }

        // Execute trigger based on fire mode
        ExecuteTrigger(
            weaponSnapshot.active.timing.kind,
            weaponSnapshot.active.timing.rpm
        );
    }

    TriggerSnapshot TriggerController::GetSnapshot() const noexcept
    {
        return m_snapshot;
    }

    bool TriggerController::IsInitialized() const noexcept
    {
        return m_arduinoController != nullptr &&
               m_weaponTrackingService != nullptr;
    }

    void TriggerController::ExecuteTrigger(
        const std::string& fireMode,
        const int rpm
    ) noexcept
    {
        const auto now = std::chrono::steady_clock::now();
        const auto elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - m_lastTriggerTime
            ).count();

        // Semi-automatic: Single click per key press
        if (fireMode == "semi-automatic")
        {
            if (!m_mouseButtonPressed)
            {
                SendMouseClick();
                m_snapshot.roundsFired++;
                m_lastTriggerTime = now;
                m_mouseButtonPressed = true;

                spdlog::info(
                    "TRIGGER_SEMI: Click sent (rounds_fired: {})",
                    m_snapshot.roundsFired
                );
            }

            return;
        }

        // Bolt-action: Click with reload time
        if (fireMode == "bolt-action")
        {
            const uint32_t shotIntervalMs = CalculateShotIntervalMs(rpm);

            if (elapsedMs >= shotIntervalMs)
            {
                SendMouseClick();
                m_snapshot.roundsFired++;
                m_lastTriggerTime = now;

                spdlog::info(
                    "TRIGGER_BOLT: Click sent (shot_interval: {}ms, rounds_fired: {})",
                    shotIntervalMs,
                    m_snapshot.roundsFired
                );
            }

            return;
        }

        // Automatic: Hold button down
        if (fireMode == "automatic")
        {
            const uint32_t shotIntervalMs = CalculateShotIntervalMs(rpm);

            // Press button on first shot
            if (!m_mouseButtonPressed)
            {
                SendMousePress();
                m_mouseButtonPressed = true;
                m_snapshot.burstCount = 1;
                m_snapshot.roundsFired++;
                m_lastTriggerTime = now;
                m_lastBurstTime = now;

                spdlog::info(
                    "TRIGGER_AUTO: Mouse pressed (shot_interval: {}ms)",
                    shotIntervalMs
                );

                return;
            }

            // Continue firing if interval passed
            if (elapsedMs >= shotIntervalMs)
            {
                m_snapshot.roundsFired++;
                m_snapshot.burstCount++;
                m_lastTriggerTime = now;

                spdlog::debug(
                    "TRIGGER_AUTO: Burst continue (burst: {}, rounds_fired: {})",
                    m_snapshot.burstCount,
                    m_snapshot.roundsFired
                );
            }

            return;
        }

        spdlog::warn(
            "TRIGGER: Unknown fire mode: '{}'",
            fireMode
        );
    }

    void TriggerController::SendMouseClick() noexcept
    {
        if (!m_arduinoController)
        {
            spdlog::error("Arduino controller is null");
            return;
        }

        const bool sent = m_arduinoController->SendMouseClick(
            kMouseLeftButton
        );

        if (!sent)
        {
            spdlog::warn("Failed to send mouse click to Arduino");
        }
        else
        {
            spdlog::debug("Mouse click sent to Arduino");
        }
    }

    void TriggerController::SendMousePress() noexcept
    {
        if (!m_arduinoController)
        {
            spdlog::error("Arduino controller is null");
            return;
        }

        const bool sent = m_arduinoController->SendTriggerPress(
            kMouseLeftButton
        );

        if (!sent)
        {
            spdlog::warn("Failed to send mouse press to Arduino");
        }
        else
        {
            spdlog::debug("Mouse press sent to Arduino");
        }
    }

    void TriggerController::SendMouseRelease() noexcept
    {
        if (!m_arduinoController)
        {
            spdlog::error("Arduino controller is null");
            return;
        }

        const bool sent = m_arduinoController->SendTriggerRelease(
            kMouseLeftButton
        );

        if (!sent)
        {
            spdlog::warn("Failed to send mouse release to Arduino");
        }
        else
        {
            spdlog::debug("Mouse release sent to Arduino");
        }
    }

    bool TriggerController::ShouldTrigger(
        const Tracking::TargetDecision& targetDecision,
        const Overlay::OverlaySettings& overlaySettings,
        const bool triggerKeyPressed
    ) const noexcept
    {
        // Trigger button not pressed
        if (!triggerKeyPressed)
        {
            return false;
        }

        // Trigger disabled
        if (!overlaySettings.triggerEnabled)
        {
            return false;
        }

        // No target
        if (!targetDecision.hasTarget)
        {
            return false;
        }

        // Target not locked or tracking
        const bool isLocked =
            targetDecision.state == Tracking::TargetState::Locked ||
            targetDecision.state == Tracking::TargetState::Tracking;

        if (!isLocked)
        {
            return false;
        }

        // Reliability too low
        if (targetDecision.reliability < overlaySettings.triggerMinimumConfidence)
        {
            return false;
        }

        return true;
    }

    uint32_t TriggerController::CalculateShotIntervalMs(
        const int rpm
    ) const noexcept
    {
        if (rpm <= 0)
        {
            return 1000; // Default 1000ms if invalid
        }

        // RPM to milliseconds: 60000 / RPM
        const uint32_t intervalMs = 60000 / rpm;

        return std::max(1U, intervalMs);
    }
}
