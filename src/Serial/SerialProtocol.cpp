#include "Serial/SerialProtocol.hpp"

#include <stdexcept>

namespace FakirBot::Serial
{
    std::uint8_t UpdateCrc8(
        std::uint8_t crc,
        const std::uint8_t value
    ) noexcept
    {
        crc ^= value;

        for (std::uint8_t bit = 0;
             bit < 8;
             ++bit)
        {
            if ((crc & 0x80U) != 0U)
            {
                crc =
                    static_cast<std::uint8_t>(
                        (crc << 1U) ^ 0x07U
                    );
            }
            else
            {
                crc <<= 1U;
            }
        }

        return crc;
    }

    std::vector<std::uint8_t>
    EncodePacket(
        const Command command,
        const std::uint8_t sequence,
        const std::span<
            const std::uint8_t
        > payload
    )
    {
        if (payload.size() >
            kMaximumPayload)
        {
            throw std::invalid_argument(
                "Serial payload exceeds protocol limit."
            );
        }

        std::vector<std::uint8_t> output;

        output.reserve(
            7 +
            payload.size()
        );

        output.push_back(kSof1);
        output.push_back(kSof2);

        std::uint8_t crc = 0;

        output.push_back(
            kProtocolVersion
        );

        crc =
            UpdateCrc8(
                crc,
                kProtocolVersion
            );

        const auto commandValue =
            static_cast<std::uint8_t>(
                command
            );

        output.push_back(
            commandValue
        );

        crc =
            UpdateCrc8(
                crc,
                commandValue
            );

        output.push_back(
            sequence
        );

        crc =
            UpdateCrc8(
                crc,
                sequence
            );

        const auto length =
            static_cast<std::uint8_t>(
                payload.size()
            );

        output.push_back(
            length
        );

        crc =
            UpdateCrc8(
                crc,
                length
            );

        for (const std::uint8_t value :
             payload)
        {
            output.push_back(
                value
            );

            crc =
                UpdateCrc8(
                    crc,
                    value
                );
        }

        output.push_back(
            crc
        );

        return output;
    }

    std::optional<Packet>
    PacketParser::Feed(
        const std::uint8_t value
    )
    {
        switch (m_state)
        {
            case State::WaitSof1:
                if (value == kSof1)
                {
                    m_state =
                        State::WaitSof2;
                }
                return std::nullopt;

            case State::WaitSof2:
                if (value == kSof2)
                {
                    m_state =
                        State::ReadVersion;

                    m_crc = 0;
                }
                else
                {
                    m_state =
                        value == kSof1
                            ? State::WaitSof2
                            : State::WaitSof1;
                }
                return std::nullopt;

            case State::ReadVersion:
                if (value !=
                    kProtocolVersion)
                {
                    ++m_protocolErrors;
                    Reset();
                    return std::nullopt;
                }

                m_crc =
                    UpdateCrc8(
                        m_crc,
                        value
                    );

                m_state =
                    State::ReadCommand;

                return std::nullopt;

            case State::ReadCommand:
                m_packet.command =
                    static_cast<Command>(
                        value
                    );

                m_crc =
                    UpdateCrc8(
                        m_crc,
                        value
                    );

                m_state =
                    State::ReadSequence;

                return std::nullopt;

            case State::ReadSequence:
                m_packet.sequence =
                    value;

                m_crc =
                    UpdateCrc8(
                        m_crc,
                        value
                    );

                m_state =
                    State::ReadLength;

                return std::nullopt;

            case State::ReadLength:
                if (value >
                    kMaximumPayload)
                {
                    ++m_protocolErrors;
                    Reset();
                    return std::nullopt;
                }

                m_expectedLength =
                    value;

                m_packet.payload.clear();

                m_packet.payload.reserve(
                    value
                );

                m_crc =
                    UpdateCrc8(
                        m_crc,
                        value
                    );

                m_state =
                    value == 0
                        ? State::ReadCrc
                        : State::ReadPayload;

                return std::nullopt;

            case State::ReadPayload:
                m_packet.payload.push_back(
                    value
                );

                m_crc =
                    UpdateCrc8(
                        m_crc,
                        value
                    );

                if (m_packet.payload.size() >=
                    m_expectedLength)
                {
                    m_state =
                        State::ReadCrc;
                }

                return std::nullopt;

            case State::ReadCrc:
            {
                if (value !=
                    m_crc)
                {
                    ++m_crcErrors;
                    Reset();
                    return std::nullopt;
                }

                Packet completed =
                    std::move(
                        m_packet
                    );

                Reset();

                return completed;
            }
        }

        Reset();
        return std::nullopt;
    }

    void PacketParser::Reset() noexcept
    {
        m_state =
            State::WaitSof1;

        m_packet =
            Packet{};

        m_expectedLength = 0;
        m_crc = 0;
    }

    std::uint64_t
    PacketParser::GetCrcErrorCount() const noexcept
    {
        return m_crcErrors;
    }

    std::uint64_t
    PacketParser::GetProtocolErrorCount() const noexcept
    {
        return m_protocolErrors;
    }

    std::uint16_t ReadUint16(
        const std::span<
            const std::uint8_t
        > bytes,
        const std::size_t offset
    )
    {
        if (offset + 2 >
            bytes.size())
        {
            throw std::out_of_range(
                "ReadUint16 exceeds payload size."
            );
        }

        return
            static_cast<std::uint16_t>(
                bytes[offset]
            ) |
            static_cast<std::uint16_t>(
                bytes[offset + 1]
            ) << 8U;
    }

    std::uint32_t ReadUint32(
        const std::span<
            const std::uint8_t
        > bytes,
        const std::size_t offset
    )
    {
        if (offset + 4 >
            bytes.size())
        {
            throw std::out_of_range(
                "ReadUint32 exceeds payload size."
            );
        }

        return
            static_cast<std::uint32_t>(
                bytes[offset]
            ) |
            static_cast<std::uint32_t>(
                bytes[offset + 1]
            ) << 8U |
            static_cast<std::uint32_t>(
                bytes[offset + 2]
            ) << 16U |
            static_cast<std::uint32_t>(
                bytes[offset + 3]
            ) << 24U;
    }
}
