#pragma once

#include "Serial/ArduinoDeviceFinder.hpp"
#include "Serial/SerialConnection.hpp"
#include "Serial/SerialProtocol.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <thread>

namespace FakirBot::Serial
{
    struct ArduinoStatusSnapshot final
    {
        bool connected{ false };

        std::string portName{
            "Searching"
        };

        std::string status{
            "Disconnected"
        };

        std::string firmware{
            "-"
        };

        std::uint16_t vendorId{ 0 };
        std::uint16_t productId{ 0 };

        double latencyMilliseconds{ 0.0 };

        std::uint64_t packetsSent{ 0 };
        std::uint64_t packetsReceived{ 0 };

        std::uint64_t crcErrors{ 0 };
        std::uint64_t protocolErrors{ 0 };

        std::uint32_t deviceUptimeMilliseconds{ 0 };
    };

    class ArduinoController final
    {
    public:
        ArduinoController() = default;
        ~ArduinoController();

        bool Start();
        void Stop();

        [[nodiscard]]
        Serial::SerialConnection* GetSerialConnection() noexcept;

        [[nodiscard]]
        ArduinoStatusSnapshot
            GetSnapshot() const;

        // ===== NEW: Active command methods =====

        /**
         * Send mouse movement command to Arduino.
         * @param deltaX Horizontal delta in mouse units
         * @param deltaY Vertical delta in mouse units
         * @return true if sent successfully
         */
        bool SendMouseMove(
            std::int16_t deltaX,
            std::int16_t deltaY
        ) noexcept;

        /**
         * Send mouse button press command.
         * @param button 1=left, 2=right, 3=middle
         * @return true if sent successfully
         */
        bool SendMouseClick(
            std::uint8_t button
        ) noexcept;

        /**
         * Send mouse button release command.
         * @param button 1=left, 2=right, 3=middle
         * @return true if sent successfully
         */
        bool SendMouseRelease(
            std::uint8_t button
        ) noexcept;

        /**
         * Send trigger button press (used for trigger simulation).
         * @param button Mouse button to trigger (usually 1 for left click)
         * @return true if sent successfully
         */
        bool SendTriggerPress(
            std::uint8_t button
        ) noexcept;

        /**
         * Send trigger button release.
         * @param button Mouse button to release
         * @return true if sent successfully
         */
        bool SendTriggerRelease(
            std::uint8_t button
        ) noexcept;

        /**
         * Send recoil compensation step.
         * Arduino will apply this delta with timing control.
         * @param deltaX Horizontal recoil delta (usually negative for upward compensation)
         * @param deltaY Vertical recoil delta
         * @param delayMs Milliseconds to wait before applying this step
         * @return true if sent successfully
         */
        bool SendRecoilStep(
            std::int16_t deltaX,
            std::int16_t deltaY,
            std::uint16_t delayMs
        ) noexcept;

    private:
        void WorkerLoop();

        [[nodiscard]]
        bool TryConnect();

        [[nodiscard]]
        bool PerformHandshake();

        [[nodiscard]]
        bool SendRequest(
            Command command,
            std::span<
            const std::uint8_t
            > payload = {}
        );

        void PollSerial();

        void HandlePacket(
            const Packet& packet
        );

        void MarkDisconnected(
            const char* reason
        );

        void PublishSnapshot(
            const ArduinoStatusSnapshot& snapshot
        );

    private:
        std::atomic<bool>
            m_running{ false };

        std::jthread m_worker;

        ArduinoDeviceFinder m_deviceFinder;
        SerialConnection m_connection;
        PacketParser m_parser;

        mutable std::mutex
            m_snapshotMutex;

        ArduinoStatusSnapshot
            m_snapshot;

        std::uint8_t m_nextSequence{ 1 };
        std::uint8_t m_pendingPingSequence{ 0 };

        std::chrono::steady_clock::time_point
            m_pendingPingTime{};

        std::chrono::steady_clock::time_point
            m_nextScanTime{};

        std::chrono::steady_clock::time_point
            m_nextPingTime{};

        std::chrono::steady_clock::time_point
            m_nextStatusTime{};

        std::chrono::steady_clock::time_point
            m_lastPacketTime{};

    };
}
