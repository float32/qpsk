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

class HammingDecoder
{
protected:
    uint32_t syndrome_;
    uint32_t bit_num_;
    uint32_t parity_bits_;

public:
    void Init(uint32_t parity_bits)
    {
        syndrome_ = 0;
        bit_num_ = 1;
        parity_bits_ = parity_bits;
    }

    // Instead of distributing the parity bits among the data bits, we use a
    // nontraditional encoding scheme in which we leave the data in place and
    // keep track of the altered sequence of bit numbers. For example, since
    // the parity bit numbers are powers of 2, the data bits will be numbered
    // 3, 5, 6, 7, 9... etc, skipping the powers of 2.
    void Process(uint8_t* data, uint32_t size)
    {
        // Calculate the error syndrome
        for (uint32_t i = 0; i < size * 8; i++)
        {
            // For all power-of-2 bit numbers, use the corresponding parity bit
            // and then skip that number in the sequence of data bit numbers.
            while ((bit_num_ & (bit_num_ - 1)) == 0)
            {
                syndrome_ ^= parity_bits_ & bit_num_;
                bit_num_++;
            }

            uint32_t bit = (data[i / 8] >> (i % 8)) & 1;

            if (bit)
            {
                syndrome_ ^= bit_num_;
            }

            bit_num_++;
        }

        // If the syndrome is 0, there was no error detected. If it's a power
        // of 2, then one of the parity bits is flipped, which we don't care
        // about. Otherwise, do error correction.
        if ((syndrome_ & (syndrome_ - 1)) != 0)
        {
            uint32_t width = sizeof(syndrome_) * 8 - __builtin_clz(syndrome_);
            uint32_t bit_pos = syndrome_ - 1 - width;

            if (bit_pos < size * 8)
            {
                data[bit_pos / 8] ^= 1 << (bit_pos % 8);
            }
        }
    }

    void Process(void* data, uint32_t size)
    {
        Process(reinterpret_cast<uint8_t*>(data), size);
    }
};

}
