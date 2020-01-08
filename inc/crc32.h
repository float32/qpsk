#ifndef QPSK_CRC32_H_
#define QPSK_CRC32_H_

#include <cstdint>

namespace qpsk
{

class Crc32
{
protected:
    static constexpr uint32_t kPolynomial = 0xEDB88320;

    static inline bool initialized_;
    static inline uint32_t table_[256];
    uint32_t crc_;

    static constexpr uint32_t ComputeTableEntry(uint32_t x)
    {
        for (uint32_t j = 0; j < 8; j++)
        {
            if (x & 1)
            {
                x >>= 1;
                x ^= kPolynomial;
            }
            else
            {
                x >>= 1;
            }
        }

        return x;
    }

public:
    void Init(void)
    {
        if (!initialized_)
        {
            for (uint32_t i = 0; i < 256; i++)
            {
                table_[i] = ComputeTableEntry(i);
            }

            initialized_ = true;
        }

        crc_ = 0;
    }

    void Seed(uint32_t crc)
    {
        crc_ = crc;
    }

    uint32_t Process(const uint8_t* data, uint32_t length)
    {
        crc_ = ~crc_;

        while (length--)
        {
            uint8_t byte = *(data++);
            crc_ = (crc_ >> 8) ^ table_[(crc_ & 0xFF) ^ byte];
        }

        crc_ = ~crc_;
        return crc_;
    }

    uint32_t crc(void) const
    {
        return crc_;
    }
};

}

#endif
