#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace FakirBot::Serial
{
    inline constexpr std::uint8_t kSof1 = 0xFA;
    inline constexpr std::uint8_t kSof2 = 0x4B;
    inline constexpr std::uint8_t kProtocolVersion = 1;
    inline constexpr std::size_t kMaximumPayload = 32;

    enum class Command : std::uint8_t
    {
        // Legacy (unused now)
        HelloRequest = 0x01,
        PingRequest = 0x02,
        StatusRequest = 0x03,

        HelloResponse = 0x81,
        PongResponse = 0x82,
        StatusResponse = 0x83,

        // Active commands
        MouseMove = 0x10,      // payload: int16_t deltaX, int16_t deltaY
        MouseClick = 0x11,     // payload: uint8_t button (1=left, 2=right, 3=middle)
        MouseRelease = 0x12,   // payload: uint8_t button

        // Trigger & Fire
        TriggerPress = 0x13,   // payload: uint8_t button (trigger as mouse button)
        TriggerRelease = 0x14, // payload: uint8_t button

        // Recoil compensation
        RecoilStep = 0x20,     // payload: int16_t deltaX, int16_t deltaY, uint16_t delayMs

        // Errors
        ErrorResponse = 0xFF
    };

    struct Packet final
    {
        Command command{
            Command::ErrorResponse
        };

        std::uint8_t sequence{ 0 };

        std::vector<
            std::uint8_t
        > payload;
    };

    [[nodiscard]]
    std::uint8_t UpdateCrc8(
        std::uint8_t crc,
        std::uint8_t value
    ) noexcept;

    [[nodiscard]]
    std::vector<std::uint8_t>
        EncodePacket(
            Command command,
            std::uint8_t sequence,
            std::span<
            const std::uint8_t
            > payload = {}
        );

    class PacketParser final
    {
    public:
        [[nodiscard]]
        std::optional<Packet>
            Feed(
                std::uint8_t value
            );

        void Reset() noexcept;

        [[nodiscard]]
        std::uint64_t GetCrcErrorCount() const noexcept;

        [[nodiscard]]
        std::uint64_t GetProtocolErrorCount() const noexcept;

    private:
        enum class State : std::uint8_t
        {
            WaitSof1,
            WaitSof2,
            ReadVersion,
            ReadCommand,
            ReadSequence,
            ReadLength,
            ReadPayload,
            ReadCrc
        };

        State m_state{
            State::WaitSof1
        };

        Packet m_packet;

        std::uint8_t m_expectedLength{ 0 };
        std::uint8_t m_crc{ 0 };

        std::uint64_t m_crcErrors{ 0 };
        std::uint64_t m_protocolErrors{ 0 };
    };

    // ===== HELPER FUNCTIONS =====

    [[nodiscard]]
    std::uint16_t ReadUint16(
        std::span<
        const std::uint8_t
        > bytes,
        std::size_t offset
    );

    [[nodiscard]]
    std::uint32_t ReadUint32(
        std::span<
        const std::uint8_t
        > bytes,
        std::size_t offset
    );

    // Write helpers for creating payloads
    inline void WriteInt16(
        std::uint8_t* dest,
        std::size_t offset,
        std::int16_t value
    ) noexcept
    {
        dest[offset] = static_cast<std::uint8_t>(value & 0xFFU);
        dest[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    }

    inline void WriteUint16(
        std::uint8_t* dest,
        std::size_t offset,
        std::uint16_t value
    ) noexcept
    {
        dest[offset] = static_cast<std::uint8_t>(value & 0xFFU);
        dest[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    }

    inline void WriteUint32(
        std::uint8_t* dest,
        std::size_t offset,
        std::uint32_t value
    ) noexcept
    {
        dest[offset] = static_cast<std::uint8_t>(value & 0xFFU);
        dest[offset + 1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
        dest[offset + 2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
        dest[offset + 3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
    }
}
