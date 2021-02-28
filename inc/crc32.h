// MIT License
//
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

        crc_ = 0xFFFFFFFF;
    }

    void Seed(uint32_t crc)
    {
        crc_ = ~crc;
    }

    uint32_t Process(const uint8_t* data, uint32_t length)
    {
        while (length--)
        {
            uint8_t byte = *(data++);
            crc_ = (crc_ >> 8) ^ table_[(crc_ & 0xFF) ^ byte];
        }

        return ~crc_;
    }

    uint32_t crc(void) const
    {
        return ~crc_;
    }
};

}
