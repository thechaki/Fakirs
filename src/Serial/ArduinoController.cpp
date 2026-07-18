#include "Serial/ArduinoController.hpp"

#include <spdlog/spdlog.h>

#include <array>
#include <thread>

namespace FakirBot::Serial
{
    namespace
    {
        constexpr std::chrono::milliseconds
            kReconnectInterval{
                1000
        };

        constexpr std::chrono::milliseconds
            kLeonardoResetWait{
                2500
        };

        constexpr std::chrono::milliseconds
            kHandshakeTimeout{
                5000
        };

        constexpr std::chrono::milliseconds
            kHandshakeRetryInterval{
                300
        };

        constexpr std::chrono::milliseconds
            kPingInterval{
                1000
        };

        constexpr std::chrono::milliseconds
            kStatusInterval{
                2000
        };

        constexpr std::chrono::milliseconds
            kConnectionTimeout{
                5000
        };

        constexpr std::uint16_t
            kExpectedVendorId =
            0x4490;

        constexpr std::uint16_t
            kExpectedProductId =
            0x2146;
    }

    ArduinoController::~ArduinoController()
    {
        Stop();
    }

    bool ArduinoController::Start()
    {
        spdlog::info(
            "ArduinoController::Start called."
        );

        if (m_running.exchange(true))
        {
            return true;
        }

        m_worker =
            std::jthread(
                [this]()
                {
                    WorkerLoop();
                }
            );

        return true;
    }

    void ArduinoController::Stop()
    {
        if (!m_running.exchange(false))
        {
            return;
        }

        if (m_worker.joinable())
        {
            m_worker.request_stop();
            m_worker.join();
        }

        m_connection.Close();

        MarkDisconnected(
            "Controller stopped"
        );
    }

    Serial::SerialConnection* ArduinoController::GetSerialConnection() noexcept
    {
        return &m_connection;
    }

    ArduinoStatusSnapshot
        ArduinoController::GetSnapshot() const
    {
        std::scoped_lock lock(
            m_snapshotMutex
        );

        return m_snapshot;
    }

    // ===== NEW: Active command methods =====

    bool ArduinoController::SendMouseMove(
        const std::int16_t deltaX,
        const std::int16_t deltaY
    ) noexcept
    {
        std::array<std::uint8_t, 4> payload{};
        WriteInt16(payload.data(), 0, deltaX);
        WriteInt16(payload.data(), 2, deltaY);

        return SendRequest(
            Command::MouseMove,
            payload
        );
    }

    bool ArduinoController::SendMouseClick(
        const std::uint8_t button
    ) noexcept
    {
        std::array<std::uint8_t, 1> payload{ button };

        return SendRequest(
            Command::MouseClick,
            payload
        );
    }

    bool ArduinoController::SendMouseRelease(
        const std::uint8_t button
    ) noexcept
    {
        std::array<std::uint8_t, 1> payload{ button };

        return SendRequest(
            Command::MouseRelease,
            payload
        );
    }

    bool ArduinoController::SendTriggerPress(
        const std::uint8_t button
    ) noexcept
    {
        std::array<std::uint8_t, 1> payload{ button };

        return SendRequest(
            Command::TriggerPress,
            payload
        );
    }

    bool ArduinoController::SendTriggerRelease(
        const std::uint8_t button
    ) noexcept
    {
        std::array<std::uint8_t, 1> payload{ button };

        return SendRequest(
            Command::TriggerRelease,
            payload
        );
    }

    bool ArduinoController::SendRecoilStep(
        const std::int16_t deltaX,
        const std::int16_t deltaY,
        const std::uint16_t delayMs
    ) noexcept
    {
        std::array<std::uint8_t, 6> payload{};
        WriteInt16(payload.data(), 0, deltaX);
        WriteInt16(payload.data(), 2, deltaY);
        WriteUint16(payload.data(), 4, delayMs);

        return SendRequest(
            Command::RecoilStep,
            payload
        );
    }

    void ArduinoController::WorkerLoop()
    {
        spdlog::info(
            "Arduino worker thread started."
        );

        m_nextScanTime =
            std::chrono::steady_clock::now();

        while (m_running.load())
        {
            const auto now =
                std::chrono::steady_clock::now();

            if (!m_connection.IsOpen())
            {
                if (now >= m_nextScanTime)
                {
                    if (!TryConnect())
                    {
                        m_nextScanTime =
                            now +
                            kReconnectInterval;
                    }
                }

                std::this_thread::sleep_for(
                    std::chrono::milliseconds(
                        10
                    )
                );

                continue;
            }

            PollSerial();

            if (!m_connection.IsOpen())
            {
                MarkDisconnected(
                    "Serial read failed"
                );

                m_nextScanTime =
                    now +
                    kReconnectInterval;

                continue;
            }

            if (now >= m_nextPingTime)
            {
                std::array<std::uint8_t, 4>
                    timestamp{};

                const auto tick =
                    static_cast<std::uint32_t>(
                        std::chrono::duration_cast<
                        std::chrono::milliseconds
                        >(
                            now.time_since_epoch()
                        ).count()
                        );

                WriteUint32(
                    timestamp.data(),
                    0,
                    tick
                );

                m_pendingPingSequence =
                    m_nextSequence;

                m_pendingPingTime =
                    now;

                const bool pingSent =
                    SendRequest(
                        Command::PingRequest,
                        timestamp
                    );

                if (!pingSent)
                {
                    MarkDisconnected(
                        "Ping write failed"
                    );

                    m_nextScanTime =
                        now +
                        kReconnectInterval;

                    continue;
                }

                m_nextPingTime =
                    now +
                    kPingInterval;
            }

            if (now >= m_nextStatusTime)
            {
                const bool statusSent =
                    SendRequest(
                        Command::StatusRequest
                    );

                if (!statusSent)
                {
                    MarkDisconnected(
                        "Status write failed"
                    );

                    m_nextScanTime =
                        now +
                        kReconnectInterval;

                    continue;
                }

                m_nextStatusTime =
                    now +
                    kStatusInterval;
            }

            if (
                m_lastPacketTime.time_since_epoch().count() != 0 &&
                now - m_lastPacketTime >
                kConnectionTimeout
                )
            {
                m_connection.Close();

                MarkDisconnected(
                    "Connection timeout"
                );

                m_nextScanTime =
                    now +
                    kReconnectInterval;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    1
                )
            );
        }

        spdlog::info(
            "Arduino worker thread stopped."
        );
    }

    bool ArduinoController::TryConnect()
    {
        spdlog::info(
            "Searching for Arduino device VID_4490&PID_2146..."
        );

        ArduinoStatusSnapshot searching =
            GetSnapshot();

        searching.connected = false;
        searching.portName = "Searching";
        searching.status = "Searching";

        PublishSnapshot(
            searching
        );

        const auto devices =
            m_deviceFinder.FindMatchingDevices();

        for (const auto& device :
            devices)
        {
            spdlog::info(
                "Arduino candidate found: {} | {}",
                device.portName,
                device.hardwareId
            );

            if (!m_connection.Open(
                device.portName,
                115200
            ))
            {
                spdlog::warn(
                    "Arduino port could not be opened: {} | Win32 error: {}",
                    device.portName,
                    m_connection.GetLastErrorCode()
                );

                continue;
            }

            ArduinoStatusSnapshot opening =
                GetSnapshot();

            opening.portName =
                device.portName;

            opening.status =
                "Opening";

            PublishSnapshot(
                opening
            );

            m_connection.Purge();
            m_parser.Reset();

            std::this_thread::sleep_for(
                kLeonardoResetWait
            );

            if (PerformHandshake())
            {
                spdlog::info(
                    "Arduino handshake successful: {}",
                    device.portName
                );

                const auto now =
                    std::chrono::steady_clock::now();

                m_nextPingTime = now;
                m_nextStatusTime = now;
                m_lastPacketTime = now;

                return true;
            }

            spdlog::warn(
                "Arduino handshake failed: {}",
                device.portName
            );

            m_connection.Close();
        }

        MarkDisconnected(
            "No verified controller"
        );

        return false;
    }

    bool ArduinoController::PerformHandshake()
    {
        const auto deadline =
            std::chrono::steady_clock::now() +
            kHandshakeTimeout;

        auto nextHelloTime =
            std::chrono::steady_clock::now();

        while (
            m_running.load() &&
            m_connection.IsOpen() &&
            std::chrono::steady_clock::now() <
            deadline
            )
        {
            const auto now =
                std::chrono::steady_clock::now();

            if (now >= nextHelloTime)
            {
                const bool helloSent =
                    SendRequest(
                        Command::HelloRequest
                    );

                if (!helloSent)
                {
                    return false;
                }

                nextHelloTime =
                    now +
                    kHandshakeRetryInterval;
            }

            std::array<std::uint8_t, 512>
                buffer{};

            const std::size_t count =
                m_connection.ReadAvailable(
                    buffer
                );

            for (std::size_t index = 0;
                index < count;
                ++index)
            {
                const auto packet =
                    m_parser.Feed(
                        buffer[index]
                    );

                if (!packet)
                {
                    continue;
                }

                if (
                    packet->command !=
                    Command::HelloResponse ||
                    packet->payload.size() <
                    11
                    )
                {
                    continue;
                }

                const bool magicMatches =
                    packet->payload[0] == 'F' &&
                    packet->payload[1] == 'B' &&
                    packet->payload[2] == '3' &&
                    packet->payload[3] == '2';

                const std::uint16_t vendorId =
                    ReadUint16(
                        packet->payload,
                        7
                    );

                const std::uint16_t productId =
                    ReadUint16(
                        packet->payload,
                        9
                    );

                if (
                    !magicMatches ||
                    packet->payload[4] !=
                    kProtocolVersion ||
                    vendorId !=
                    kExpectedVendorId ||
                    productId !=
                    kExpectedProductId
                    )
                {
                    spdlog::warn(
                        "Arduino handshake identity mismatch."
                    );

                    return false;
                }

                ArduinoStatusSnapshot connected =
                    GetSnapshot();

                connected.connected = true;
                connected.portName =
                    m_connection.GetPortName();

                connected.status =
                    "Connected";

                connected.vendorId =
                    vendorId;

                connected.productId =
                    productId;

                connected.firmware =
                    std::to_string(
                        packet->payload[5]
                    ) +
                    "." +
                    std::to_string(
                        packet->payload[6]
                    );

                connected.packetsReceived += 1;

                connected.crcErrors =
                    m_parser.GetCrcErrorCount();

                connected.protocolErrors =
                    m_parser.GetProtocolErrorCount();

                PublishSnapshot(
                    connected
                );

                return true;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(
                    1
                )
            );
        }

        return false;
    }

    bool ArduinoController::SendRequest(
        const Command command,
        const std::span<
        const std::uint8_t
        > payload
    )
    {
        if (!m_connection.IsOpen())
        {
            return false;
        }

        const std::uint8_t sequence =
            m_nextSequence++;

        const auto packet =
            EncodePacket(
                command,
                sequence,
                payload
            );

        if (!m_connection.Write(
            packet
        ))
        {
            return false;
        }

        ArduinoStatusSnapshot snapshot =
            GetSnapshot();

        snapshot.packetsSent += 1;

        PublishSnapshot(
            snapshot
        );

        return true;
    }

    void ArduinoController::PollSerial()
    {
        std::array<std::uint8_t, 1024>
            buffer{};

        for (;;)
        {
            const std::size_t count =
                m_connection.ReadAvailable(
                    buffer
                );

            if (count == 0)
            {
                break;
            }

            for (std::size_t index = 0;
                index < count;
                ++index)
            {
                const auto packet =
                    m_parser.Feed(
                        buffer[index]
                    );

                if (packet)
                {
                    HandlePacket(
                        *packet
                    );
                }
            }
        }
    }

    void ArduinoController::HandlePacket(
        const Packet& packet
    )
    {
        m_lastPacketTime =
            std::chrono::steady_clock::now();

        ArduinoStatusSnapshot snapshot =
            GetSnapshot();

        snapshot.connected = true;
        snapshot.status = "Connected";
        snapshot.portName =
            m_connection.GetPortName();

        snapshot.packetsReceived += 1;

        snapshot.crcErrors =
            m_parser.GetCrcErrorCount();

        snapshot.protocolErrors =
            m_parser.GetProtocolErrorCount();

        switch (packet.command)
        {
        case Command::PongResponse:
            if (packet.sequence ==
                m_pendingPingSequence)
            {
                snapshot.latencyMilliseconds =
                    std::chrono::duration<
                    double,
                    std::milli
                    >(
                        m_lastPacketTime -
                        m_pendingPingTime
                    ).count();
            }
            break;

        case Command::StatusResponse:
            if (packet.payload.size() >= 21)
            {
                snapshot.deviceUptimeMilliseconds =
                    ReadUint32(
                        packet.payload,
                        0
                    );
            }
            break;

        case Command::ErrorResponse:
            snapshot.protocolErrors += 1;
            break;

        default:
            break;
        }

        PublishSnapshot(
            snapshot
        );
    }

    void ArduinoController::MarkDisconnected(
        const char* reason
    )
    {
        ArduinoStatusSnapshot snapshot =
            GetSnapshot();

        if (snapshot.connected)
        {
            spdlog::warn(
                "Arduino disconnected from {}. Reason: {}",
                snapshot.portName,
                reason
            );
        }
        else
        {
            spdlog::debug(
                "Arduino remains disconnected. Reason: {}",
                reason
            );
        }

        snapshot.connected = false;
        snapshot.portName = "Searching";
        snapshot.status = "Disconnected";
        snapshot.latencyMilliseconds = 0.0;

        PublishSnapshot(
            snapshot
        );
    }

    void ArduinoController::PublishSnapshot(
        const ArduinoStatusSnapshot& snapshot
    )
    {
        std::scoped_lock lock(
            m_snapshotMutex
        );

        m_snapshot =
            snapshot;
    }
}
