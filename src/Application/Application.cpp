// File: src/Application/Application.cpp
#include "Application/Application.hpp"

#include "Capture/LatestFrameBuffer.hpp"
#include "Config/ConfigManager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <Windows.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/utils/logger.hpp>

#include <spdlog/spdlog.h>

namespace FakirBot
{
    namespace
    {
        [[nodiscard]]
        std::uint32_t GetLogicalProcessorCount() noexcept
        {
            const DWORD processorCount =
                GetActiveProcessorCount(
                    ALL_PROCESSOR_GROUPS
                );

            return processorCount > 0
                ? static_cast<std::uint32_t>(
                    processorCount
                    )
                : 1U;
        }

        [[nodiscard]]
        DWORD_PTR CreateProcessorMask(
            const std::uint32_t processorIndex
        ) noexcept
        {
            constexpr std::uint32_t maskBitCount =
                static_cast<std::uint32_t>(
                    sizeof(DWORD_PTR) * 8U
                    );

            if (processorIndex >= maskBitCount)
            {
                return 0;
            }

            return
                static_cast<DWORD_PTR>(1)
                << processorIndex;
        }

        void SetCurrentThreadAffinity(
            const std::uint32_t processorIndex,
            const char* threadName
        )
        {
            const DWORD_PTR affinityMask =
                CreateProcessorMask(
                    processorIndex
                );

            if (affinityMask == 0 ||
                SetThreadAffinityMask(
                    GetCurrentThread(),
                    affinityMask
                ) == 0)
            {
                spdlog::warn(
                    "{} affinity could not be configured. "
                    "Processor: {}, Win32 error: {}",
                    threadName,
                    processorIndex,
                    GetLastError()
                );

                return;
            }

            spdlog::info(
                "{} affinity configured. "
                "Logical processor: {}",
                threadName,
                processorIndex
            );
        }

        void SetCurrentThreadPriorityLevel(
            const int priority,
            const char* threadName
        )
        {
            if (!SetThreadPriority(
                GetCurrentThread(),
                priority
            ))
            {
                spdlog::warn(
                    "{} priority could not be configured. "
                    "Win32 error: {}",
                    threadName,
                    GetLastError()
                );

                return;
            }

            spdlog::info(
                "{} priority configured: {}",
                threadName,
                priority
            );
        }

        void ConfigureProcessPriority()
        {
            if (!SetPriorityClass(
                GetCurrentProcess(),
                HIGH_PRIORITY_CLASS
            ))
            {
                spdlog::warn(
                    "Process priority could not be configured. "
                    "Win32 error: {}",
                    GetLastError()
                );

                return;
            }

            spdlog::info(
                "Process priority configured: "
                "HIGH_PRIORITY_CLASS"
            );
        }
    }

    bool Application::Initialize()
    {
        try
        {
            cv::utils::logging::setLogLevel(
                cv::utils::logging::LOG_LEVEL_WARNING
            );

            if (!ConfigManager::Load(
                "config/settings.json"
            ))
            {
                return false;
            }

            m_config =
                ConfigManager::Get();

            m_playerTeamController.SetTeam(
                Runtime::PlayerTeamController::Parse(
                    m_config.localTeam
                )
            );

            m_runtimeStatusStore =
                std::make_shared<
                Runtime::RuntimeStatusStore
                >();

            m_arduinoController =
                std::make_unique<
                Serial::ArduinoController
                >();

            if (!m_arduinoController->Start())
            {
                spdlog::error(
                    "Arduino controller could not be started."
                );
            }
            else
            {
                spdlog::info(
                    "Arduino controller started."
                );
            }

            ConfigureProcessPriority();

            m_classMap =
                std::make_shared<
                Tracking::ClassMap
                >();

            m_classMap->SetNames(
                m_config.classNames
            );

            m_classMap->SetFusionGroups(
                m_config.classFusionGroups
            );

            m_performanceController.SetEnabled(
                m_config.enablePerformanceController
            );

            m_performanceController.SetHighLatencyThresholdMs(
                m_config.performanceHighLatencyMs
            );

            m_performanceController.SetCriticalLatencyThresholdMs(
                m_config.performanceCriticalLatencyMs
            );

            m_performanceController.SetRecoveryLatencyThresholdMs(
                m_config.performanceRecoveryLatencyMs
            );

            m_model =
                std::make_unique<
                Inference::OnnxModel
                >(
                    m_config.modelPath.string(),
                    m_config.cpuFallbackModelPath.string(),
                    m_config.executionProvider,
                    m_config.cpuIntraOpThreads,
                    m_config.cpuInterOpThreads,
                    m_config.confidenceThreshold,
                    m_config.nmsThreshold
                );

            m_model->WarmUp(
                m_config.warmupRuns
            );

            m_capture =
                std::make_unique<
                Capture::DxgiCapture
                >();

            if (!m_capture->Initialize(
                static_cast<std::uint32_t>(
                    m_config.adapterIndex
                    ),
                static_cast<std::uint32_t>(
                    m_config.outputIndex
                    )
            ))
            {
                spdlog::error(
                    "DXGI capture could not be initialized."
                );

                m_capture.reset();
                m_model.reset();

                return false;
            }

            if (m_config.enableTracking)
            {
                m_tracker =
                    std::make_unique<
                    Tracking::KalmanTracker
                    >(
                        m_config.trackingMaxLostFrames,
                        m_config.trackingAssociationDistance,
                        m_config.trackingMinimumIou,
                        m_config.trackingDistanceWeight,
                        m_config.trackingIouWeight,
                        m_config.trackingSizeWeight,
                        m_config.trackingConfidenceSmoothing,
                        m_config.trackingVelocitySmoothing
                    );

                m_tracker->SetClassMap(
                    m_classMap
                );

                m_tracker->SetConfirmationFrames(
                    m_config.trackingConfirmationFrames
                );

                m_tracker->SetTentativeMaximumLostFrames(
                    m_config.trackingTentativeMaximumLostFrames
                );

                m_tracker->SetReidentificationFrames(
                    m_config.trackingReidentificationFrames
                );

                m_tracker->SetReidentificationDistance(
                    m_config.trackingReidentificationDistance
                );

                m_tracker->SetBoxSmoothing(
                    m_config.trackingBoxSmoothing
                );

                m_tracker->SetInvalidAssignmentCost(
                    m_config.trackingInvalidAssignmentCost
                );
            }

            if (m_config.enableTargetSelection)
            {
                m_targetSelector =
                    std::make_unique<
                    Tracking::TargetSelector
                    >();

                m_targetSelector->SetEnabled(
                    m_config.enableTargetSelection
                );

                m_targetSelector->SetOnlyConfirmedTracks(
                    m_config.targetOnlyConfirmedTracks
                );

                m_targetSelector->SetFovRadius(
                    m_config.targetFovRadius
                );

                m_targetSelector->SetStickyTargetBonus(
                    m_config.targetStickyBonus
                );

                m_targetSelector->SetConfidenceWeight(
                    m_config.targetConfidenceWeight
                );

                m_targetSelector->SetDistanceWeight(
                    m_config.targetDistanceWeight
                );

                m_targetSelector->SetMaximumLostFrames(
                    m_config.targetMaximumLostFrames
                );

                m_targetSelector->SetAimPointVerticalRatio(
                    m_config.targetAimPointVerticalRatio
                );

                m_targetSelector->SetMotionWeight(
                    m_config.targetMotionWeight
                );

                m_targetSelector->SetMinimumTrackAge(
                    m_config.targetMinimumTrackAge
                );

                m_targetSelector->SetMinimumConfidence(
                    m_config.targetMinimumConfidence
                );

                m_targetSelector->SetDynamicAimPointEnabled(
                    m_config.targetDynamicAimPoint
                );

                m_targetSelector->SetPredictionHorizonMs(
                    m_config.targetPredictionHorizonMs
                );

                m_targetSelector->SetMaximumPredictionPixels(
                    m_config.targetMaximumPredictionPixels
                );

                m_targetSelector->SetSwitchScoreMargin(
                    m_config.targetSwitchScoreMargin
                );

                ApplyPlayerTeamRules(
                    false
                );

                m_aimStateMachine =
                    std::make_unique<
                    Tracking::AimStateMachine
                    >();

                m_aimStateMachine->SetCandidateConfirmationFrames(
                    m_config.stateCandidateConfirmationFrames
                );

                m_aimStateMachine->SetLostGraceFrames(
                    m_config.stateLostGraceFrames
                );

                m_aimStateMachine->SetMinimumLockReliability(
                    m_config.stateMinimumLockReliability
                );

                m_aimStateMachine->SetPointSmoothing(
                    m_config.statePointSmoothing
                );

                m_aimStateMachine->SetDeadZonePixels(
                    m_config.stateDeadZonePixels
                );

                m_aimStateMachine->SetMaximumPointStepPixels(
                    m_config.stateMaximumPointStepPixels
                );
            }

            ApplyPlayerTeamRules(
                false
            );

            m_weaponTrackingService =
                std::make_unique<
                WeaponTracking::WeaponTrackingService
                >();

            const std::filesystem::path
                weaponTrackingAssets =
                std::filesystem::current_path() /
                "assets" /
                "weapon_tracking";

            if (!m_weaponTrackingService->Initialize(
                weaponTrackingAssets
            ))
            {
                spdlog::error(
                    "Weapon tracking could not be initialized. "
                    "Assets: {}",
                    weaponTrackingAssets.string()
                );

                m_weaponTrackingService.reset();
            }
            else
            {
                spdlog::info(
                    "Weapon tracking initialized. "
                    "Assets: {}",
                    weaponTrackingAssets.string()
                );
            }

            m_overlayManager =
                std::make_unique<
                Overlay::OverlayManager
                >();

            const std::filesystem::path logoPath =
                std::filesystem::current_path() /
                "assets" /
                "ui" /
                "fakirbot_logo.png";

            m_overlayManager->Start(
                m_runtimeStatusStore,
                logoPath
            );

            // ===== AIMBOT INITIALIZATION (DEFERRED) =====
            m_aimbotController =
                std::make_unique<Aimbot::AimbotController>();

            spdlog::info(
                "Aimbot controller created (waiting for Arduino connection)."
            );

            spdlog::info(
                "Application initialized. "
                "Desktop: {}x{} | Capture: {}x{} | "
                "Adapter: {} | Output: {} | "
                "Provider: {}",
                m_capture->GetWidth(),
                m_capture->GetHeight(),
                m_config.captureWidth,
                m_config.captureHeight,
                m_config.adapterIndex,
                m_config.outputIndex,
                m_model->GetExecutionProvider()
            );

            return true;
        }
        catch (const std::exception& exception)
        {
            spdlog::error(
                "Application initialization failed: {}",
                exception.what()
            );

            Shutdown();

            return false;
        }
    }

    void Application::Run()
    {
        if (!m_model ||
            !m_capture ||
            !m_capture->IsInitialized())
        {
            spdlog::error(
                "Application is not initialized."
            );

            return;
        }

        using Clock =
            std::chrono::steady_clock;

        constexpr auto frameWaitTimeout =
            std::chrono::milliseconds(100);

        const std::uint32_t logicalProcessorCount =
            GetLogicalProcessorCount();

        const std::uint32_t inferenceProcessorIndex =
            logicalProcessorCount > 1
            ? logicalProcessorCount - 1
            : 0;

        const std::uint32_t captureProcessorIndex =
            logicalProcessorCount > 2
            ? logicalProcessorCount - 2
            : 0;

        SetCurrentThreadPriorityLevel(
            THREAD_PRIORITY_HIGHEST,
            "Inference thread"
        );

        SetCurrentThreadAffinity(
            inferenceProcessorIndex,
            "Inference thread"
        );

        const auto& inputShape =
            m_model->GetInputShape();

        if (inputShape.size() != 4)
        {
            spdlog::error(
                "Invalid model input shape."
            );

            return;
        }

        const int modelHeight =
            static_cast<int>(
                inputShape.at(2)
                );

        const int modelWidth =
            static_cast<int>(
                inputShape.at(3)
                );

        if (m_config.captureWidth != modelWidth ||
            m_config.captureHeight != modelHeight)
        {
            spdlog::error(
                "Capture size must match model input. "
                "Capture: {}x{}, Model: {}x{}",
                m_config.captureWidth,
                m_config.captureHeight,
                modelWidth,
                modelHeight
            );

            return;
        }

        const int desktopWidth =
            m_capture->GetWidth();

        const int desktopHeight =
            m_capture->GetHeight();

        const int captureX =
            (desktopWidth -
                m_config.captureWidth) / 2;

        const int captureY =
            (desktopHeight -
                m_config.captureHeight) / 2;

        constexpr double weaponOriginalRoiX = 0.86;
        constexpr double weaponOriginalRoiY = 0.78;
        constexpr double weaponOriginalRoiHeight = 0.15;
        constexpr int weaponReferenceWidth = 1920;
        constexpr int weaponReferenceHeight = 1080;
        constexpr int weaponOldReferenceHeight = 350;
        constexpr int weaponReferenceWidthPixels = 250;
        constexpr int weaponReferenceHeightPixels = 220;
        constexpr double weaponLeftShiftRatio = 1.0 / 3.0;

        const double weaponScaleX =
            static_cast<double>(desktopWidth) /
            static_cast<double>(weaponReferenceWidth);

        const double weaponScaleY =
            static_cast<double>(desktopHeight) /
            static_cast<double>(weaponReferenceHeight);

        int weaponHudWidth =
            std::max(
                1,
                static_cast<int>(
                    std::lround(
                        weaponReferenceWidthPixels *
                        weaponScaleX
                    )
                    )
            );

        const int weaponOldHeight =
            std::max(
                1,
                static_cast<int>(
                    std::lround(
                        weaponOldReferenceHeight *
                        weaponScaleY
                    )
                    )
            );

        int weaponHudHeight =
            std::max(
                1,
                static_cast<int>(
                    std::lround(
                        weaponReferenceHeightPixels *
                        weaponScaleY
                    )
                    )
            );

        int weaponHudX =
            static_cast<int>(
                std::lround(
                    desktopWidth *
                    weaponOriginalRoiX
                )
                ) -
            static_cast<int>(
                std::lround(
                    weaponHudWidth *
                    weaponLeftShiftRatio
                )
                );

        int weaponHudY =
            static_cast<int>(
                std::lround(
                    desktopHeight *
                    (
                        weaponOriginalRoiY +
                        weaponOriginalRoiHeight
                        )
                )
                ) -
            weaponOldHeight;

        weaponHudX =
            std::clamp(
                weaponHudX,
                0,
                std::max(
                    0,
                    desktopWidth - 1
                )
            );

        weaponHudY =
            std::clamp(
                weaponHudY,
                0,
                std::max(
                    0,
                    desktopHeight - 1
                )
            );

        weaponHudWidth =
            std::clamp(
                weaponHudWidth,
                1,
                desktopWidth - weaponHudX
            );

        weaponHudHeight =
            std::clamp(
                weaponHudHeight,
                1,
                desktopHeight - weaponHudY
            );

        Capture::LatestFrameBuffer frameBuffer;
        Capture::LatestFrameBuffer weaponFrameBuffer;

        std::atomic<bool>
            captureError{ false };

        std::atomic<std::uint64_t>
            totalPublishedFrames{ 0 };

        std::atomic<std::uint64_t>
            totalCaptureTimeouts{ 0 };

        std::jthread captureThread(
            [
                this,
                &frameBuffer,
                &weaponFrameBuffer,
                &captureError,
                &totalPublishedFrames,
                &totalCaptureTimeouts,
                captureX,
                captureY,
                weaponHudX,
                weaponHudY,
                weaponHudWidth,
                weaponHudHeight,
                captureProcessorIndex
            ](
                const std::stop_token stopToken
                )
        {
            SetCurrentThreadPriorityLevel(
                THREAD_PRIORITY_ABOVE_NORMAL,
                "Capture thread"
            );

            SetCurrentThreadAffinity(
                captureProcessorIndex,
                "Capture thread"
            );

            cv::Mat capturedFrame;
            cv::Mat weaponHudFrame;

            while (!stopToken.stop_requested())
            {
                const auto captureEndBefore =
                    Clock::now();

                const bool captured =
                    m_capture->CaptureFrameRegions(
                        capturedFrame,
                        captureX,
                        captureY,
                        m_config.captureWidth,
                        m_config.captureHeight,
                        weaponHudFrame,
                        weaponHudX,
                        weaponHudY,
                        weaponHudWidth,
                        weaponHudHeight,
                        static_cast<std::uint32_t>(
                            m_config.captureTimeoutMs
                            )
                    );

                const auto captureEnd =
                    Clock::now();

                const auto& captureStatistics =
                    m_capture->GetLastStatistics();

                if (!captured)
                {
                    if (captureStatistics.timedOut)
                    {
                        totalCaptureTimeouts.fetch_add(
                            1,
                            std::memory_order_relaxed
                        );
                    }

                    if (!m_capture->IsInitialized())
                    {
                        captureError.store(
                            true,
                            std::memory_order_release
                        );

                        break;
                    }

                    continue;
                }

                if (capturedFrame.empty())
                {
                    continue;
                }

                totalPublishedFrames.fetch_add(
                    1,
                    std::memory_order_relaxed
                );

                frameBuffer.Publish(
                    capturedFrame,
                    captureStatistics.totalMs,
                    captureEnd
                );

                if (!weaponHudFrame.empty())
                {
                    weaponFrameBuffer.Publish(
                        weaponHudFrame,
                        captureStatistics.totalMs,
                        captureEnd
                    );
                }

                (void)captureEndBefore;
            }

            frameBuffer.Close();
            weaponFrameBuffer.Close();
        }
        );

        std::jthread weaponTrackingThread(
            [
                this,
                &weaponFrameBuffer
            ](
                const std::stop_token stopToken
                )
            {
                if (!m_weaponTrackingService)
                {
                    return;
                }

                cv::Mat weaponFrame;
                Capture::FrameMetadata
                    weaponMetadata{};

                std::uint64_t
                    weaponSequence = 0;

                std::string
                    lastPrimaryWeapon;

                std::string
                    lastPistolWeapon;

                std::string
                    lastActiveWeapon;

                std::uint64_t
                    lastActiveSequence = 0;

                constexpr auto
                    weaponWaitTimeout =
                    std::chrono::milliseconds(
                        250
                    );

                while (!stopToken.stop_requested())
                {
                    const bool available =
                        weaponFrameBuffer.WaitForNext(
                            weaponSequence,
                            weaponWaitTimeout,
                            weaponFrame,
                            weaponMetadata
                        );

                    if (!available)
                    {
                        if (weaponFrameBuffer.IsClosed())
                        {
                            break;
                        }

                        continue;
                    }

                    weaponSequence =
                        weaponMetadata.sequence;

                    if (weaponFrame.empty())
                    {
                        continue;
                    }

                    if (!m_weaponTrackingService
                        ->ProcessHudFrame(
                            weaponFrame
                        ))
                    {
                        continue;
                    }

                    const auto snapshot =
                        m_weaponTrackingService
                        ->GetSnapshot();

                    if (snapshot.primary.weaponId !=
                        lastPrimaryWeapon)
                    {
                        lastPrimaryWeapon =
                            snapshot.primary.weaponId;

                        spdlog::info(
                            "Weapon primary changed: {} | "
                            "Pattern: {} | Points: {} | "
                            "Timing: {} ms | Kind: {} | RPM: {}",
                            snapshot.primary.displayName,
                            snapshot.primary.patternId,
                            snapshot.primary.patternPointCount,
                            snapshot.primary.timing.milliseconds,
                            snapshot.primary.timing.kind,
                            snapshot.primary.timing.rpm
                        );
                    }

                    if (snapshot.pistol.weaponId !=
                        lastPistolWeapon)
                    {
                        lastPistolWeapon =
                            snapshot.pistol.weaponId;

                        spdlog::info(
                            "Weapon pistol changed: {} | "
                            "Pattern: {} | Points: {} | "
                            "Timing: {} ms | Kind: {} | RPM: {}",
                            snapshot.pistol.displayName,
                            snapshot.pistol.patternId,
                            snapshot.pistol.patternPointCount,
                            snapshot.pistol.timing.milliseconds,
                            snapshot.pistol.timing.kind,
                            snapshot.pistol.timing.rpm
                        );
                    }

                    if (snapshot.activeSequence !=
                        lastActiveSequence &&
                        snapshot.active.weaponId !=
                        "UNKNOWN")
                    {
                        lastActiveSequence =
                            snapshot.activeSequence;

                        lastActiveWeapon =
                            snapshot.active.weaponId;

                        spdlog::info(
                            "ACTIVE WEAPON: {} | Slot: {} | "
                            "WeaponId: {} | Pattern: {} | "
                            "Points: {} | Coordinates: {} | Timing: {} ms | "
                            "Kind: {} | RPM: {}",
                            snapshot.active.displayName,
                            WeaponTracking::ToString(
                                snapshot.activeSlot
                            ),
                            snapshot.active.weaponId,
                            snapshot.active.patternId,
                            snapshot.active.patternPointCount,
                            snapshot.active.patternPoints.size(),
                            snapshot.active.timing.milliseconds,
                            snapshot.active.timing.kind,
                            snapshot.active.timing.rpm
                        );
                    }
                }
            }
        );

        cv::Mat inferenceFrame;
        Capture::FrameMetadata frameMetadata{};

        std::uint64_t lastSequence = 0;
        std::uint64_t processedFrames = 0;
        std::uint64_t skippedFrames = 0;

        std::uint64_t reportProcessedFrames = 0;
        std::uint64_t reportSkippedFrames = 0;
        std::uint64_t previousPublishedFrames = 0;
        std::uint64_t previousTimeouts = 0;

        std::size_t lastDetectionCount = 0;
        std::size_t lastTrackCount = 0;

        std::uint64_t lastSelectedTrackId = 0;
        int lastSelectedClassId = -1;
        float lastSelectedScore = 0.0F;

        Tracking::TargetState lastTargetState =
            Tracking::TargetState::Searching;

        float lastTargetReliability = 0.0F;
        float lastTargetSpeed = 0.0F;

        cv::Point2f lastPredictedPoint{};
        cv::Point2f lastFilteredPoint{};

        double accumulatedPreprocessMs = 0.0;
        double accumulatedInferenceMs = 0.0;
        double accumulatedPostprocessMs = 0.0;
        double accumulatedModelTotalMs = 0.0;
        double accumulatedQueueAgeMs = 0.0;
        double accumulatedEndToEndMs = 0.0;

        auto previousFrameTime =
            Clock::now();

        bool teamToggleKeyWasDown =
            false;

        bool aimLockKeyWasDown =
            false;

        bool killSwitchKeyWasDown =
            false;

        bool killSwitchActive =
            false;

        auto lastTeamToggleTime =
            Clock::now() -
            std::chrono::milliseconds(
                m_config.teamToggleDebounceMs
            );

        auto reportWindowStart =
            Clock::now();

        std::filesystem::path activeModelPath =
            m_config.modelPath;

        std::filesystem::path activeCpuFallbackModelPath =
            m_config.cpuFallbackModelPath;

        std::string activeExecutionProvider =
            m_config.executionProvider;

        auto nextHotReloadCheck =
            Clock::now() +
            std::chrono::milliseconds(
                m_config.hotReloadIntervalMs
            );

        m_benchmark.Clear();

        spdlog::info(
            "Pipeline started. "
            "Region: x={}, y={}, width={}, height={}. "
            "Weapon HUD: x={}, y={}, width={}, height={}. "
            "Press END to stop.",
            captureX,
            captureY,
            m_config.captureWidth,
            m_config.captureHeight,
            weaponHudX,
            weaponHudY,
            weaponHudWidth,
            weaponHudHeight
        );

        try
        {
            while (true)
            {
                if (
                    (
                        GetAsyncKeyState(
                            VK_END
                        ) &
                        0x8000
                        ) != 0
                    )
                {
                    spdlog::info(
                        "Stop key received."
                    );

                    break;
                }

                if (
                    m_arduinoController &&
                    m_runtimeStatusStore
                    )
                {
                    const auto arduinoStatus =
                        m_arduinoController
                        ->GetSnapshot();

                    m_runtimeStatusStore
                        ->SetArduinoStatus(
                            arduinoStatus.connected,
                            arduinoStatus.portName,
                            arduinoStatus.status,
                            arduinoStatus.firmware,
                            arduinoStatus.latencyMilliseconds,
                            arduinoStatus.packetsSent,
                            arduinoStatus.packetsReceived,
                            arduinoStatus.crcErrors,
                            arduinoStatus.protocolErrors,
                            arduinoStatus.deviceUptimeMilliseconds
                        );
                }

                if (
                    m_overlayManager &&
                    m_runtimeStatusStore
                    )
                {
                    const auto& overlaySettings =
                        m_overlayManager
                        ->GetSettings();

                    const int aimLockVirtualKey =
                        overlaySettings
                        .aimLockButton
                        .virtualKey;

                    const bool aimLockKeyIsDown =
                        aimLockVirtualKey != 0 &&
                        (
                            GetAsyncKeyState(
                                aimLockVirtualKey
                            ) &
                            0x8000
                            ) != 0;

                    const bool aimLockPressed =
                        aimLockKeyIsDown &&
                        !aimLockKeyWasDown;

                    const bool aimLockReleased =
                        !aimLockKeyIsDown &&
                        aimLockKeyWasDown;

                    const int killSwitchVirtualKey =
                        overlaySettings
                        .killSwitchBind
                        .virtualKey;

                    const bool killSwitchKeyIsDown =
                        killSwitchVirtualKey != 0 &&
                        (
                            GetAsyncKeyState(
                                killSwitchVirtualKey
                            ) &
                            0x8000
                            ) != 0;

                    if (
                        killSwitchKeyIsDown &&
                        !killSwitchKeyWasDown
                        )
                    {
                        killSwitchActive =
                            !killSwitchActive;

                        spdlog::info(
                            "KillSwitch changed: {}",
                            killSwitchActive
                            ? "ACTIVE"
                            : "INACTIVE"
                        );
                    }

                    const bool effectiveAimLockEnabled =
                        overlaySettings.aimLockEnabled &&
                        !killSwitchActive;

                    m_runtimeStatusStore
                        ->SetAimLockStatus(
                            effectiveAimLockEnabled,
                            aimLockKeyIsDown,
                            aimLockPressed,
                            aimLockReleased,
                            overlaySettings
                            .aimLockSensitivity,
                            killSwitchActive,
                            overlaySettings
                            .aimLockButton
                            .displayName
                        );

                    aimLockKeyWasDown =
                        aimLockKeyIsDown;

                    killSwitchKeyWasDown =
                        killSwitchKeyIsDown;
                }

                const bool teamToggleKeyIsDown =
                    (
                        GetAsyncKeyState(
                            m_config.teamToggleVirtualKey
                        ) &
                        0x8000
                        ) != 0;

                const auto teamToggleNow =
                    Clock::now();

                const bool debouncePassed =
                    teamToggleNow -
                    lastTeamToggleTime >=
                    std::chrono::milliseconds(
                        m_config.teamToggleDebounceMs
                    );

                if (
                    teamToggleKeyIsDown &&
                    !teamToggleKeyWasDown &&
                    debouncePassed
                    )
                {
                    m_playerTeamController.Toggle();

                    lastTeamToggleTime =
                        teamToggleNow;

                    ApplyPlayerTeamRules(
                        true
                    );

                    spdlog::info(
                        "Player team changed: {} | {}",
                        m_playerTeamController
                        .GetPlayerLabel(),
                        m_playerTeamController
                        .GetTargetModeLabel()
                    );
                }

                teamToggleKeyWasDown =
                    teamToggleKeyIsDown;

                if (captureError.load(
                    std::memory_order_acquire
                ))
                {
                    if (!m_config.enableCaptureRecovery)
                    {
                        spdlog::error(
                            "Capture thread stopped "
                            "because DXGI capture was lost."
                        );

                        break;
                    }

                    spdlog::warn(
                        "DXGI capture lost. "
                        "Recovery requires pipeline restart."
                    );

                    break;
                }

                const auto now =
                    Clock::now();

                if (now >= nextHotReloadCheck)
                {
                    if (ConfigManager::ReloadIfChanged())
                    {
                        const AppConfig& newConfig =
                            ConfigManager::Get();

                        const bool modelSettingsChanged =
                            newConfig.modelPath !=
                            activeModelPath ||
                            newConfig.cpuFallbackModelPath !=
                            activeCpuFallbackModelPath ||
                            newConfig.executionProvider !=
                            activeExecutionProvider;

                        if (
                            m_config.enableModelHotReload &&
                            modelSettingsChanged
                            )
                        {
                            auto replacementModel =
                                std::make_unique<
                                Inference::OnnxModel
                                >(
                                    newConfig.modelPath.string(),
                                    newConfig.cpuFallbackModelPath.string(),
                                    newConfig.executionProvider,
                                    newConfig.cpuIntraOpThreads,
                                    newConfig.cpuInterOpThreads,
                                    newConfig.confidenceThreshold,
                                    newConfig.nmsThreshold
                                );

                            replacementModel->WarmUp(
                                newConfig.warmupRuns
                            );

                            m_model =
                                std::move(
                                    replacementModel
                                );

                            activeModelPath =
                                newConfig.modelPath;

                            activeCpuFallbackModelPath =
                                newConfig.cpuFallbackModelPath;

                            activeExecutionProvider =
                                newConfig.executionProvider;

                            if (m_tracker)
                            {
                                m_tracker->Reset();
                            }

                            if (m_targetSelector)
                            {
                                m_targetSelector->Reset();
                            }

                            if (m_aimStateMachine)
                            {
                                m_aimStateMachine->Reset();
                            }

                            spdlog::info(
                                "Model hot reload completed."
                            );
                        }

                        const bool configuredTeamChanged =
                            newConfig.localTeam !=
                            m_config.localTeam;

                        m_config.teamToggleVirtualKey =
                            newConfig.teamToggleVirtualKey;

                        m_config.teamToggleDebounceMs =
                            newConfig.teamToggleDebounceMs;

                        m_config.tPlayerAllowedTargetClasses =
                            newConfig.tPlayerAllowedTargetClasses;

                        m_config.tPlayerTargetPriorities =
                            newConfig.tPlayerTargetPriorities;

                        m_config.ctPlayerAllowedTargetClasses =
                            newConfig.ctPlayerAllowedTargetClasses;

                        m_config.ctPlayerTargetPriorities =
                            newConfig.ctPlayerTargetPriorities;

                        if (configuredTeamChanged)
                        {
                            m_config.localTeam =
                                newConfig.localTeam;

                            m_playerTeamController.SetTeam(
                                Runtime::PlayerTeamController::Parse(
                                    newConfig.localTeam
                                )
                            );
                        }

                        if (m_classMap)
                        {
                            m_classMap->SetNames(
                                newConfig.classNames
                            );

                            m_classMap->SetFusionGroups(
                                newConfig.classFusionGroups
                            );
                        }

                        m_performanceController.SetEnabled(
                            newConfig.enablePerformanceController
                        );

                        m_performanceController.SetHighLatencyThresholdMs(
                            newConfig.performanceHighLatencyMs
                        );

                        m_performanceController.SetCriticalLatencyThresholdMs(
                            newConfig.performanceCriticalLatencyMs
                        );

                        m_performanceController.SetRecoveryLatencyThresholdMs(
                            newConfig.performanceRecoveryLatencyMs
                        );

                        m_model->SetConfidenceThreshold(
                            newConfig.confidenceThreshold
                        );

                        m_model->SetNmsThreshold(
                            newConfig.nmsThreshold
                        );

                        if (m_tracker)
                        {
                            m_tracker->SetMaximumLostFrames(
                                newConfig.trackingMaxLostFrames
                            );

                            m_tracker->SetAssociationDistance(
                                newConfig.trackingAssociationDistance
                            );

                            m_tracker->SetMinimumIou(
                                newConfig.trackingMinimumIou
                            );

                            m_tracker->SetDistanceWeight(
                                newConfig.trackingDistanceWeight
                            );

                            m_tracker->SetIouWeight(
                                newConfig.trackingIouWeight
                            );

                            m_tracker->SetSizeWeight(
                                newConfig.trackingSizeWeight
                            );

                            m_tracker->SetConfidenceSmoothing(
                                newConfig.trackingConfidenceSmoothing
                            );

                            m_tracker->SetVelocitySmoothing(
                                newConfig.trackingVelocitySmoothing
                            );

                            m_tracker->SetConfirmationFrames(
                                newConfig.trackingConfirmationFrames
                            );

                            m_tracker->SetTentativeMaximumLostFrames(
                                newConfig.trackingTentativeMaximumLostFrames
                            );

                            m_tracker->SetReidentificationFrames(
                                newConfig.trackingReidentificationFrames
                            );

                            m_tracker->SetReidentificationDistance(
                                newConfig.trackingReidentificationDistance
                            );

                            m_tracker->SetBoxSmoothing(
                                newConfig.trackingBoxSmoothing
                            );

                            m_tracker->SetInvalidAssignmentCost(
                                newConfig.trackingInvalidAssignmentCost
                            );
                        }

                        if (m_targetSelector)
                        {
                            m_targetSelector->SetEnabled(
                                newConfig.enableTargetSelection
                            );

                            m_targetSelector->SetOnlyConfirmedTracks(
                                newConfig.targetOnlyConfirmedTracks
                            );

                            m_targetSelector->SetFovRadius(
                                newConfig.targetFovRadius
                            );

                            m_targetSelector->SetStickyTargetBonus(
                                newConfig.targetStickyBonus
                            );

                            m_targetSelector->SetConfidenceWeight(
                                newConfig.targetConfidenceWeight
                            );

                            m_targetSelector->SetDistanceWeight(
                                newConfig.targetDistanceWeight
                            );

                            m_targetSelector->SetMaximumLostFrames(
                                newConfig.targetMaximumLostFrames
                            );

                            m_targetSelector->SetAimPointVerticalRatio(
                                newConfig.targetAimPointVerticalRatio
                            );

                            m_targetSelector->SetMotionWeight(
                                newConfig.targetMotionWeight
                            );

                            m_targetSelector->SetMinimumTrackAge(
                                newConfig.targetMinimumTrackAge
                            );

                            m_targetSelector->SetMinimumConfidence(
                                newConfig.targetMinimumConfidence
                            );

                            m_targetSelector->SetDynamicAimPointEnabled(
                                newConfig.targetDynamicAimPoint
                            );

                            m_targetSelector->SetPredictionHorizonMs(
                                newConfig.targetPredictionHorizonMs
                            );

                            m_targetSelector->SetMaximumPredictionPixels(
                                newConfig.targetMaximumPredictionPixels
                            );

                            m_targetSelector->SetSwitchScoreMargin(
                                newConfig.targetSwitchScoreMargin
                            );

                            ApplyPlayerTeamRules(
                                configuredTeamChanged
                            );
                        }

                        if (m_aimStateMachine)
                        {
                            m_aimStateMachine->SetCandidateConfirmationFrames(
                                newConfig.stateCandidateConfirmationFrames
                            );

                            m_aimStateMachine->SetLostGraceFrames(
                                newConfig.stateLostGraceFrames
                            );

                            m_aimStateMachine->SetMinimumLockReliability(
                                newConfig.stateMinimumLockReliability
                            );

                            m_aimStateMachine->SetPointSmoothing(
                                newConfig.statePointSmoothing
                            );

                            m_aimStateMachine->SetDeadZonePixels(
                                newConfig.stateDeadZonePixels
                            );

                            m_aimStateMachine->SetMaximumPointStepPixels(
                                newConfig.stateMaximumPointStepPixels
                            );
                        }

                        m_config.confidenceThreshold =
                            newConfig.confidenceThreshold;

                        m_config.nmsThreshold =
                            newConfig.nmsThreshold;

                        m_config.trackingMaxLostFrames =
                            newConfig.trackingMaxLostFrames;

                        m_config.trackingAssociationDistance =
                            newConfig.trackingAssociationDistance;

                        m_config.trackingMinimumIou =
                            newConfig.trackingMinimumIou;

                        m_config.trackingDistanceWeight =
                            newConfig.trackingDistanceWeight;

                        m_config.trackingIouWeight =
                            newConfig.trackingIouWeight;

                        m_config.trackingSizeWeight =
                            newConfig.trackingSizeWeight;

                        m_config.trackingConfidenceSmoothing =
                            newConfig.trackingConfidenceSmoothing;

                        spdlog::info(
                            "Runtime settings applied. "
                            "Confidence: {:.2f}, NMS: {:.2f}",
                            m_config.confidenceThreshold,
                            m_config.nmsThreshold
                        );
                    }

                    nextHotReloadCheck =
                        now +
                        std::chrono::milliseconds(
                            m_config.hotReloadIntervalMs
                        );
                }

                const bool frameAvailable =
                    frameBuffer.WaitForNext(
                        lastSequence,
                        frameWaitTimeout,
                        inferenceFrame,
                        frameMetadata
                    );

                if (!frameAvailable)
                {
                    if (frameBuffer.IsClosed())
                    {
                        break;
                    }

                    continue;
                }

                if (
                    lastSequence > 0 &&
                    frameMetadata.sequence >
                    lastSequence + 1
                    )
                {
                    const std::uint64_t skipped =
                        frameMetadata.sequence -
                        lastSequence -
                        1;

                    skippedFrames += skipped;
                    reportSkippedFrames += skipped;
                }

                lastSequence =
                    frameMetadata.sequence;

                if (inferenceFrame.empty())
                {
                    continue;
                }

                const auto modelStart =
                    Clock::now();

                const double queueAgeMs =
                    std::chrono::duration<
                    double,
                    std::milli
                    >(
                        modelStart -
                        frameMetadata.capturedAt
                    ).count();

                const auto result =
                    m_model->Run(
                        inferenceFrame
                    );

                const auto modelEnd =
                    Clock::now();

                const double endToEndMs =
                    std::chrono::duration<
                    double,
                    std::milli
                    >(
                        modelEnd -
                        frameMetadata.capturedAt
                    ).count();

                const double deltaSeconds =
                    std::chrono::duration<double>(
                        modelEnd -
                        previousFrameTime
                    ).count();

                previousFrameTime =
                    modelEnd;

                std::vector<
                    Tracking::TrackedObject
                > tracks;

                if (m_tracker)
                {
                    tracks =
                        m_tracker->Update(
                            result.detections,
                            deltaSeconds
                        );

                    lastTrackCount =
                        tracks.size();
                }
                else
                {
                    lastTrackCount = 0;
                }

                if (m_overlayManager)
                {
                    const auto& overlaySettings =
                        m_overlayManager
                        ->GetSettings();

                    if (m_targetSelector)
                    {
                        m_targetSelector->SetFovRadius(
                            overlaySettings.fovRadius
                        );
                    }
                }

                std::optional<
                    Tracking::SelectedTarget
                > selectedTarget;

                if (m_targetSelector)
                {
                    selectedTarget =
                        m_targetSelector->Select(
                            tracks,
                            cv::Size(
                                m_config.captureWidth,
                                m_config.captureHeight
                            )
                        );
                }

                if (m_aimStateMachine)
                {
                    const auto decision =
                        m_aimStateMachine->Update(
                            selectedTarget,
                            deltaSeconds
                        );

                    lastTargetState =
                        decision.state;

                    lastTargetReliability =
                        decision.reliability;

                    lastTargetSpeed =
                        decision.speed;

                    lastPredictedPoint =
                        decision.predictedPoint;

                    lastFilteredPoint =
                        decision.filteredPoint;

                    lastSelectedTrackId =
                        decision.hasTarget
                        ? decision.trackId
                        : 0;

                    lastSelectedClassId =
                        decision.hasTarget
                        ? decision.classId
                        : -1;

                    lastSelectedScore =
                        decision.hasTarget
                        ? decision.targetScore
                        : 0.0F;

                    // ===== DEFERRED AIMBOT INITIALIZATION CHECK =====
                    if (m_aimbotController &&
                        m_arduinoController &&
                        m_arduinoController->GetSerialConnection() &&
                        m_arduinoController->GetSerialConnection()->IsOpen() &&
                        !m_aimbotController->IsInitialized())
                    {
                        m_aimbotController->Initialize(
                            m_arduinoController->GetSerialConnection()
                        );

                        spdlog::info(
                            "Aimbot controller initialized with Arduino connection."
                        );
                    }

                    // ===== AIMBOT UPDATE =====
                    if (m_aimbotController &&
                        m_aimbotController->IsInitialized())
                    {
                        const auto& overlaySettings =
                            m_overlayManager->GetSettings();

                        m_aimbotController->Update(
                            decision,
                            overlaySettings,
                            m_config.captureWidth,
                            m_config.captureHeight
                        );
                    }
                }

                m_benchmark.AddSample(
                    result.statistics
                );

                lastDetectionCount =
                    result.detections.size();

                accumulatedPreprocessMs +=
                    result.statistics.preprocessMs;

                accumulatedInferenceMs +=
                    result.statistics.inferenceMs;

                accumulatedPostprocessMs +=
                    result.statistics.outputCopyMs;

                accumulatedModelTotalMs +=
                    result.statistics.totalMs;

                accumulatedQueueAgeMs +=
                    queueAgeMs;

                accumulatedEndToEndMs +=
                    endToEndMs;

                ++processedFrames;
                ++reportProcessedFrames;

                const double elapsedSeconds =
                    std::chrono::duration<double>(
                        modelEnd -
                        reportWindowStart
                    ).count();

                if (elapsedSeconds < 1.0)
                {
                    continue;
                }

                const std::uint64_t currentPublishedFrames =
                    totalPublishedFrames.load(
                        std::memory_order_relaxed
                    );

                const std::uint64_t currentTimeouts =
                    totalCaptureTimeouts.load(
                        std::memory_order_relaxed
                    );

                const std::uint64_t reportPublishedFrames =
                    currentPublishedFrames -
                    previousPublishedFrames;

                const std::uint64_t reportTimeouts =
                    currentTimeouts -
                    previousTimeouts;

                const double captureFps =
                    static_cast<double>(
                        reportPublishedFrames
                        ) /
                    elapsedSeconds;

                const double processingFps =
                    static_cast<double>(
                        reportProcessedFrames
                        ) /
                    elapsedSeconds;

                const double divisor =
                    reportProcessedFrames > 0
                    ? static_cast<double>(
                        reportProcessedFrames
                        )
                    : 1.0;

                const std::uint64_t totalWindowFrames =
                    reportProcessedFrames +
                    reportSkippedFrames;

                const double skippedPercentage =
                    totalWindowFrames > 0
                    ? (
                        static_cast<double>(
                            reportSkippedFrames
                            ) /
                        static_cast<double>(
                            totalWindowFrames
                            )
                        ) *
                    100.0
                    : 0.0;

                const auto performanceDecision =
                    m_performanceController.Update(
                        Runtime::PerformanceSnapshot{
                            accumulatedInferenceMs / divisor,
                            accumulatedQueueAgeMs / divisor,
                            accumulatedEndToEndMs / divisor,
                            processingFps,
                            reportSkippedFrames,
                            reportProcessedFrames
                        }
                    );

                if (m_targetSelector)
                {
                    m_targetSelector->SetRuntimePredictionScale(
                        performanceDecision.predictionScale
                    );

                    m_targetSelector->SetRuntimeFovScale(
                        performanceDecision.fovScale
                    );

                    m_targetSelector->SetMinimumTrackAge(
                        performanceDecision.targetMinimumTrackAge
                    );

                    m_targetSelector->SetMaximumLostFrames(
                        performanceDecision.targetMaximumLostFrames
                    );
                }

                const std::string selectedClassName =
                    m_classMap
                    ? m_classMap->GetName(
                        lastSelectedClassId
                    )
                    : std::string(
                        "Unknown"
                    );

                const std::string targetStateText(
                    Tracking::AimStateMachine::ToString(
                        lastTargetState
                    )
                );

                const std::string performanceModeText(
                    Runtime::PerformanceController::ToString(
                        performanceDecision.mode
                    )
                );

                if (m_runtimeStatusStore)
                {
                    m_runtimeStatusStore->SetPipelineStatus(
                        captureFps,
                        processingFps,
                        accumulatedModelTotalMs /
                        divisor,
                        accumulatedEndToEndMs /
                        divisor,
                        m_benchmark.GetP99Ms(),
                        static_cast<int>(
                            lastDetectionCount
                            ),
                        static_cast<int>(
                            lastTrackCount
                            ),
                        lastSelectedTrackId,
                        lastSelectedClassId,
                        selectedClassName,
                        lastSelectedScore,
                        lastTargetReliability,
                        targetStateText,
                        performanceModeText
                    );
                }

                spdlog::info(
                    "Capture FPS: {:.1f} | "
                    "Processing FPS: {:.1f} | "
                    "Pre: {:.3f} ms | "
                    "Infer: {:.3f} ms | "
                    "Post: {:.3f} ms | "
                    "Model Total: {:.3f} ms | "
                    "Queue: {:.3f} ms | "
                    "End-to-End: {:.3f} ms | "
                    "P99: {:.3f} ms | "
                    "Skipped: {} ({:.1f}%) | "
                    "Timeouts: {} | "
                    "Detections: {} | Tracks: {} | "
                    "Selected Track: {} | Class: {} | "
                    "Target Score: {:.3f} | State: {} | "
                    "Reliability: {:.3f} | Speed: {:.1f} | "
                    "Predicted: ({:.1f},{:.1f}) | "
                    "Filtered: ({:.1f},{:.1f}) | "
                    "Class Name: {} | Perf Mode: {}",
                    captureFps,
                    processingFps,
                    accumulatedPreprocessMs / divisor,
                    accumulatedInferenceMs / divisor,
                    accumulatedPostprocessMs / divisor,
                    accumulatedModelTotalMs / divisor,
                    accumulatedQueueAgeMs / divisor,
                    accumulatedEndToEndMs / divisor,
                    m_benchmark.GetP99Ms(),
                    reportSkippedFrames,
                    skippedPercentage,
                    reportTimeouts,
                    lastDetectionCount,
                    lastTrackCount,
                    lastSelectedTrackId,
                    lastSelectedClassId,
                    lastSelectedScore,
                    Tracking::AimStateMachine::ToString(
                        lastTargetState
                    ),
                    lastTargetReliability,
                    lastTargetSpeed,
                    lastPredictedPoint.x,
                    lastPredictedPoint.y,
                    lastFilteredPoint.x,
                    lastFilteredPoint.y,
                    selectedClassName,
                    performanceModeText
                );

                previousPublishedFrames =
                    currentPublishedFrames;

                previousTimeouts =
                    currentTimeouts;

                reportProcessedFrames = 0;
                reportSkippedFrames = 0;

                accumulatedPreprocessMs = 0.0;
                accumulatedInferenceMs = 0.0;
                accumulatedPostprocessMs = 0.0;
                accumulatedModelTotalMs = 0.0;
                accumulatedQueueAgeMs = 0.0;
                accumulatedEndToEndMs = 0.0;

                m_benchmark.Clear();

                reportWindowStart =
                    modelEnd;
            }
        }
        catch (const std::exception& exception)
        {
            spdlog::error(
                "Inference pipeline failed: {}",
                exception.what()
            );
        }

        captureThread.request_stop();
        weaponTrackingThread.request_stop();

        frameBuffer.Close();
        weaponFrameBuffer.Close();

        if (captureThread.joinable())
        {
            captureThread.join();
        }

        if (weaponTrackingThread.joinable())
        {
            weaponTrackingThread.join();
        }

        spdlog::info(
            "Pipeline stopped. "
            "Published: {}, Processed: {}, "
            "Skipped: {}, Capture timeouts: {}",
            totalPublishedFrames.load(
                std::memory_order_relaxed
            ),
            processedFrames,
            skippedFrames,
            totalCaptureTimeouts.load(
                std::memory_order_relaxed
            )
        );
    }


    std::shared_ptr<
        Runtime::RuntimeStatusStore
    > Application::GetRuntimeStatusStore() const noexcept
    {
        return m_runtimeStatusStore;
    }

    void Application::ApplyPlayerTeamRules(
        const bool resetTargetState
    )
    {
        if (!m_targetSelector)
        {
            return;
        }

        if (
            m_playerTeamController.GetTeam() ==
            Runtime::PlayerTeam::T
            )
        {
            m_targetSelector->SetAllowedClasses(
                m_config.tPlayerAllowedTargetClasses
            );

            m_targetSelector->SetClassPriorities(
                m_config.tPlayerTargetPriorities
            );
        }
        else
        {
            m_targetSelector->SetAllowedClasses(
                m_config.ctPlayerAllowedTargetClasses
            );

            m_targetSelector->SetClassPriorities(
                m_config.ctPlayerTargetPriorities
            );
        }

        if (resetTargetState)
        {
            m_targetSelector->Reset();

            if (m_aimStateMachine)
            {
                m_aimStateMachine->Reset();
            }
        }

        if (m_runtimeStatusStore)
        {
            m_runtimeStatusStore->SetTeamStatus(
                std::string(
                    m_playerTeamController
                    .GetPlayerLabel()
                ),
                std::string(
                    m_playerTeamController
                    .GetTargetModeLabel()
                ),
                m_config.teamToggleVirtualKey
            );
        }
    }

    void Application::Shutdown()
    {
        if (m_aimbotController)
        {
            m_aimbotController.reset();
        }

        if (m_weaponTrackingService)
        {
            m_weaponTrackingService->Shutdown();
        }

        m_weaponTrackingService.reset();

        if (m_arduinoController)
        {
            m_arduinoController->Stop();
        }

        m_arduinoController.reset();

        if (m_overlayManager)
        {
            m_overlayManager->Stop();
        }

        m_overlayManager.reset();

        if (m_capture)
        {
            m_capture->Shutdown();
        }

        if (m_aimStateMachine)
        {
            m_aimStateMachine->Reset();
        }

        if (m_targetSelector)
        {
            m_targetSelector->Reset();
        }

        if (m_tracker)
        {
            m_tracker->Reset();
        }

        m_aimStateMachine.reset();
        m_targetSelector.reset();
        m_tracker.reset();
        m_classMap.reset();
        m_runtimeStatusStore.reset();

        m_performanceController.Reset();
        m_capture.reset();
        m_model.reset();

        spdlog::info(
            "Application shutdown."
        );
    }
}
