// MIT License
//
// Copyright 2013 Émilie Gillet
// Copyright 2020 Tyler Coy
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
#include "window.h"

namespace qpsk
{

template <uint32_t symbol_duration>
class CarrierRejectionFilter
{
protected:
    /* [[[cog

    import scipy.signal

    def FloatTable(table):
        for i in range(0, len(table), 4):
            line = ''
            for j in range(i, min(len(table), i + 4)):
                line += '{0: .8e},'.format(table[j]).ljust(17)
            yield line.rstrip(' ')

    kernel_length = 7
    symbol_durations = (6, 8, 12, 16)

    cog.outl('static constexpr uint32_t kKernelLength = {0};'
        .format(kernel_length))

    for symbol_duration in symbol_durations:
        cog.outl('static constexpr float kKernel{0:02}[{1}] ='
            .format(symbol_duration, kernel_length))

        freq = 1 / symbol_duration
        kernel = list(scipy.signal.remez(kernel_length,
            [0, freq, freq * 2, 0.5], [1, 0]))

        cog.outl('{')
        for line in FloatTable(kernel):
            cog.outl('    ' + line)
        cog.outl('};')

    cog.outl('')

    cog.outl('static_assert(')
    for duration in symbol_durations:
        cog.outl('    symbol_duration == {0:>2} ||'.format(duration))
    cog.outl('    false, "Unsupported symbol duration");')

    cog.outl('')

    cog.outl('static constexpr const float* kKernel =')
    for duration in symbol_durations:
        cog.outl('    symbol_duration == {0:>2} ? kKernel{0:02} :'
            .format(duration))
    cog.outl('                            nullptr;')

    ]]] */
    static constexpr uint32_t kKernelLength = 7;
    static constexpr float kKernel06[7] =
    {
        -7.61504431e-02,  4.23661388e-05,  3.04728871e-01,  5.00042366e-01,
         3.04728871e-01,  4.23661388e-05, -7.61504431e-02,
    };
    static constexpr float kKernel08[7] =
    {
        -4.62606751e-02,  1.25000000e-01,  2.96260675e-01,  3.82800831e-01,
         2.96260675e-01,  1.25000000e-01, -4.62606751e-02,
    };
    static constexpr float kKernel12[7] =
    {
         4.06822339e-02,  2.09317766e-01,  2.09317766e-01,  2.54748848e-01,
         2.09317766e-01,  2.09317766e-01,  4.06822339e-02,
    };
    static constexpr float kKernel16[7] =
    {
         1.56977082e-01,  1.37855092e-01,  1.68060009e-01,  1.79345186e-01,
         1.68060009e-01,  1.37855092e-01,  1.56977082e-01,
    };

    static_assert(
        symbol_duration ==  6 ||
        symbol_duration ==  8 ||
        symbol_duration == 12 ||
        symbol_duration == 16 ||
        false, "Unsupported symbol duration");

    static constexpr const float* kKernel =
        symbol_duration ==  6 ? kKernel06 :
        symbol_duration ==  8 ? kKernel08 :
        symbol_duration == 12 ? kKernel12 :
        symbol_duration == 16 ? kKernel16 :
                                nullptr;
    // [[[end]]]

    Window<float, kKernelLength> window_;
    float output_;

public:
    void Init(void)
    {
        window_.Init();
        output_ = 0.f;
    }

    float Process(float in)
    {
        window_.Write(in);

        output_ = 0.f;

        for (uint32_t i = 0; i < kKernelLength; i++)
        {
            output_ += window_[i] * kKernel[i];
        }

        return output_;
    }

    float output(void)
    {
        return output_;
    }
};

}
