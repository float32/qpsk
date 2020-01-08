#ifndef QPSK_PACKET_H_
#define QPSK_PACKET_H_

#include <cstdint>
#include <cstring>
#include "inc/crc32.h"

namespace qpsk
{

template <uint32_t packet_size>
class Packet
{
protected:
    static constexpr uint32_t kPacketDataLength = packet_size;

    uint32_t size_;
    uint32_t symbol_count_;
    Crc32 crc_;
    uint32_t seed_;

    struct __attribute__ ((__packed__)) QPSKPacket
    {
        uint8_t data[kPacketDataLength];
        uint32_t crc;
    };

    union
    {
        QPSKPacket packet;
        uint8_t bytes[sizeof(QPSKPacket)];
    } packet_;

public:
    void Init(uint32_t crc_seed)
    {
        crc_.Init();
        seed_ = crc_seed;
        Reset();
    }

    void Reset(void)
    {
        size_ = 0;
        symbol_count_ = 0;
    }

    void WriteByte(uint8_t byte)
    {
        if (size_ < sizeof(QPSKPacket))
        {
            packet_.bytes[size_] = byte;
            size_++;
        }
    }

    void WriteSymbol(uint8_t symbol)
    {
        if (size_ < sizeof(QPSKPacket))
        {
            packet_.bytes[size_] <<= 2;
            packet_.bytes[size_] |= symbol;
            symbol_count_++;

            if (symbol_count_ == 4)
            {
                symbol_count_ = 0;
                size_++;
            }
        }
    }

    bool Complete(void)
    {
        return size_ == sizeof(QPSKPacket);
    }

    uint32_t CalculatedCRC(void)
    {
        crc_.Seed(seed_);
        return crc_.Process(packet_.packet.data, kPacketDataLength);
    }

    uint32_t ExpectedCRC(void)
    {
        uint32_t crc = packet_.packet.crc;
        crc = __builtin_bswap32(crc);
        return crc;
    }

    bool Valid(void)
    {
        return CalculatedCRC() == ExpectedCRC();
    }

    uint8_t* data(void)
    {
        return packet_.packet.data;
    }
};

template <uint32_t page_size>
class Page
{
protected:
    uint32_t data_[page_size / 4];
    uint32_t size_;

public:
    void Init(void)
    {
        Clear();
    }

    void Clear(void)
    {
        size_ = 0;
    }

    template <uint32_t packet_size>
    void AppendPacket(Packet<packet_size>& packet)
    {
        if (size_ <= page_size - packet_size)
        {
            std::memcpy(&data_[size_ / 4], packet.data(), packet_size);
            size_ += packet_size;
        }
    }

    bool Complete(void)
    {
        return size_ == page_size;
    }

    uint32_t* data(void)
    {
        return data_;
    }
};

}

#endif
