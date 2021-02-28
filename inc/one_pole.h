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
#include <cmath>
#include "util.h"

namespace qpsk
{

class OnePole
{
protected:
    static constexpr float Factor(float freq)
    {
        return 1 - std::exp(-2 * kPi * freq);
    }

    float factor_;
    float lp_;
    float hp_;

public:
    void Init(float normalized_frequency)
    {
        factor_ = Factor(normalized_frequency);
        Reset();
    }

    void Reset(void)
    {
        lp_ = 0.f;
        hp_ = 0.f;
    }

    void Process(float in)
    {
        lp_ += factor_ * (in - lp_);
        hp_ = in - lp_;
    }

    float lowpass(void)
    {
        return lp_;
    }

    float highpass(void)
    {
        return hp_;
    }
};

class OnePoleLowpass : public OnePole
{
protected:
    using super = OnePole;

public:
    float Process(float in)
    {
        super::Process(in);
        return super::lp_;
    }

    float output(void)
    {
        return super::lp_;
    }
};

class OnePoleHighpass : public OnePole
{
protected:
    using super = OnePole;

public:
    float Process(float in)
    {
        super::Process(in);
        return super::hp_;
    }

    float output(void)
    {
        return super::hp_;
    }
};

}
