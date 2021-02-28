// MIT License
//
// Copyright 2013 Émilie Gillet
// Copyright 2021 Tyler Coy
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cstdint>
#include <cstring>
#include "crc32.h"
#include "error_correction.h"

namespace qpsk
{

template <uint32_t packet_size>
class Packet
{
protected:
    static constexpr uint32_t kPacketDataLength = packet_size;

    static constexpr uint32_t max_data_bits(uint32_t num_parity_bits)
    {
        return (2 << num_parity_bits) - num_parity_bits - 1;
    }

    uint32_t size_;
    uint32_t byte_;
    Crc32 crc_;
    uint32_t seed_;
    HammingDecoder hamming_;

    struct __attribute__ ((__packed__)) QPSKPacket
    {
        uint8_t data[kPacketDataLength];
        uint32_t crc;
        uint16_t ecc;
    };

    union
    {
        QPSKPacket packet_;
        uint8_t bytes_[sizeof(QPSKPacket)];
    };

    static_assert(
        kPacketDataLength * 8 <= max_data_bits(sizeof(packet_.ecc) * 8));

    void PushByte(uint8_t byte)
    {
        if (size_ < sizeof(QPSKPacket))
        {
            bytes_[size_] = byte;
            size_++;

            if (size_ == sizeof(QPSKPacket))
            {
                Finalize();
            }
        }
    }

    void Finalize(void)
    {
        #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
            packet_.ecc = __builtin_bswap16(packet_.ecc);
        #endif

        hamming_.Init(packet_.ecc);
        hamming_.Process(bytes_, sizeof(QPSKPacket) - sizeof(packet_.ecc));

        crc_.Seed(seed_);
        crc_.Process(packet_.data, kPacketDataLength);
    }

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
        byte_ = 1;
    }

    void WriteSymbol(uint8_t symbol)
    {
        byte_ = (byte_ << 2) | symbol;

        if (byte_ & 0x100)
        {
            PushByte(byte_);
            byte_ = 1;
        }
    }

    bool full(void)
    {
        return size_ == sizeof(QPSKPacket);
    }

    uint32_t calculated_crc(void)
    {
        return crc_.crc();
    }

    uint32_t expected_crc(void)
    {
        #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
            return __builtin_bswap32(packet_.crc);
        #else
            return packet_.crc;
        #endif
    }

    bool valid(void)
    {
        return full() && (calculated_crc() == expected_crc());
    }

    uint8_t* data(void)
    {
        return packet_.data;
    }

    uint8_t last_byte(void)
    {
        return size_ ? bytes_[size_ - 1] : 0;
    }
};

template <uint32_t block_size>
class Block
{
protected:
    uint32_t data_[block_size / 4];
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
        if (size_ <= block_size - packet_size)
        {
            std::memcpy(&data_[size_ / 4], packet.data(), packet_size);
            size_ += packet_size;
        }
    }

    bool full(void)
    {
        return size_ == block_size;
    }

    uint32_t* data(void)
    {
        return data_;
    }
};

}
