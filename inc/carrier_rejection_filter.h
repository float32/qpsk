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
#include "window.h"

namespace qpsk
{

template <uint32_t symbol_duration>
class CarrierRejectionFilter
{
protected:
    struct Biquad
    {
        float b[3];
        float a[2];
    };

    /* [[[cog

    import scipy.signal

    symbol_durations = (6, 8, 12, 16)
    rp = 1
    rs = 18

    for symbol_duration in symbol_durations:
        wp = 2 / symbol_duration
        ws = 2 * wp
        b, a = scipy.signal.ellip(2, rp, rs, wp, output='ba')

        cog.outl('static constexpr Biquad kBiquad{0:02} ='
            .format(symbol_duration))

        cog.outl('{')
        print_coeff = lambda c: '{: .8e},'.format(c).ljust(17)
        b = ''.join([print_coeff(c) for c in b])
        a = ''.join([print_coeff(c) for c in a[1:]])
        cog.outl('    { ' + b + ' },')
        cog.outl('    { ' + a + ' },')
        cog.outl('};')

    cog.outl('')

    cog.outl('static_assert(')
    for duration in symbol_durations:
        cog.outl('    symbol_duration == {0:>2} ||'.format(duration))
    cog.outl('    false, "Unsupported symbol duration");')

    cog.outl('')

    cog.outl('static constexpr const Biquad* kBiquad =')
    for duration in symbol_durations:
        cog.outl('    symbol_duration == {0:>2} ? &kBiquad{0:02} :'
            .format(duration))
    cog.outl('                            nullptr;')

    ]]] */
    static constexpr Biquad kBiquad06 =
    {
        {  2.39359876e-01,  2.23228723e-01,  2.39359876e-01,  },
        { -6.20855598e-01,  4.08454741e-01,  },
    };
    static constexpr Biquad kBiquad08 =
    {
        {  1.87847557e-01,  6.44525698e-02,  1.87847557e-01,  },
        { -9.89139413e-01,  4.82993238e-01,  },
    };
    static constexpr Biquad kBiquad12 =
    {
        {  1.47991307e-01, -7.59076793e-02,  1.47991307e-01,  },
        { -1.35345827e+00,  6.00386413e-01,  },
    };
    static constexpr Biquad kBiquad16 =
    {
        {  1.33896140e-01, -1.36081787e-01,  1.33896140e-01,  },
        { -1.53005166e+00,  6.77833259e-01,  },
    };

    static_assert(
        symbol_duration ==  6 ||
        symbol_duration ==  8 ||
        symbol_duration == 12 ||
        symbol_duration == 16 ||
        false, "Unsupported symbol duration");

    static constexpr const Biquad* kBiquad =
        symbol_duration ==  6 ? &kBiquad06 :
        symbol_duration ==  8 ? &kBiquad08 :
        symbol_duration == 12 ? &kBiquad12 :
        symbol_duration == 16 ? &kBiquad16 :
                                nullptr;
    // [[[end]]]

    float x_[3];
    float y_[2];

public:
    void Init(void)
    {
        x_[0] = 0.f;
        x_[1] = 0.f;
        x_[2] = 0.f;
        y_[0] = 0.f;
        y_[1] = 0.f;
    }

    float Process(float in)
    {
        // Shift x state
        x_[2] = x_[1];
        x_[1] = x_[0];
        x_[0] = in;

        float out = 0.f;

        // Add x state
        out += kBiquad->b[0] * x_[0];
        out += kBiquad->b[1] * x_[1];
        out += kBiquad->b[2] * x_[2];

        // Subtract y state
        out -= kBiquad->a[0] * y_[0];
        out -= kBiquad->a[1] * y_[1];

        // Shift y state
        y_[1] = y_[0];
        y_[0] = out;

        return out;
    }

    float output(void)
    {
        return y_[0];
    }
};

}
