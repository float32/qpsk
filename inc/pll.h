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
#include "one_pole.h"
#include "util.h"

namespace qpsk
{

class PhaseLockedLoop
{
protected:
    float nominal_frequency_;
    float phase_increment_;
    float phase_;
    float phase_error_;
    OnePoleLowpass lpf_;

public:
    void Init(float normalized_frequency)
    {
        nominal_frequency_ = normalized_frequency;
        Reset();
        lpf_.Init(normalized_frequency / 32.f);
    }

    void Reset(void)
    {
        phase_increment_ = nominal_frequency_;
        phase_ = 0.f;
        phase_error_ = 0.f;
    }

    void Sync(void)
    {
        phase_ = 0.f;
        phase_error_ = 0.f;
    }

    float phase(void)
    {
        return phase_;
    }

    float step(void)
    {
        return phase_increment_;
    }

    float error(void)
    {
        return phase_error_;
    }

    float Process(float error)
    {
        phase_error_ = lpf_.Process(error);
        phase_increment_ -= phase_error_ / 4096.f;
        phase_increment_ = Clamp(phase_increment_, 0.f, 1.f);

        phase_ += phase_increment_ - phase_error_ / 16.f;
        phase_ = FractionalPart(phase_);

        return phase_;
    }
};

}
