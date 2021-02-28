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
class Correlator
{
protected:
    static constexpr uint32_t kSymbolDuration = symbol_duration;
    static constexpr uint32_t kAlignmentPattern = 0b1001;
    static constexpr uint32_t kPatternLength = 2;

    static constexpr uint32_t kRipeAge = kSymbolDuration * kPatternLength / 2.f;
    static constexpr float kPeakThreshold =
        kSymbolDuration * kPatternLength / 2.f;

    Bay<float, kSymbolDuration, kPatternLength> i_history_;
    Bay<float, kSymbolDuration, kPatternLength> q_history_;
    Window<float, 3> correlation_history_;
    uint32_t age_;
    float maximum_;
    float tilt_;

public:
    void Init(void)
    {
        Reset();
    }

    void Reset(void)
    {
        i_history_.Init();
        q_history_.Init();
        correlation_history_.Init();
        age_ = 0;
        maximum_ = 0.f;
        tilt_ = 0.5f;
    }

    bool Process(float i_sample, float q_sample)
    {
        i_history_.Write(i_sample);
        q_history_.Write(q_sample);

        float correlation = 0.f;

        if (++age_ >= kRipeAge)
        {
            for (uint32_t i = 0; i < kPatternLength; i++)
            {
                uint8_t symbol = kAlignmentPattern >> (i * 2);
                uint8_t expected_i = (symbol & 2);
                uint8_t expected_q = (symbol & 1);

                float i_sum = i_history_[i].sum();
                float q_sum = q_history_[i].sum();

                correlation += expected_i ? i_sum : -i_sum;
                correlation += expected_q ? q_sum : -q_sum;
            }
        }

        if (correlation < 0.f)
        {
            // Reset the peak detector at each valley in the detection function,
            // so that we can detect several consecutive peaks.
            maximum_ = 0.f;
        }
        else if (correlation > maximum_)
        {
            maximum_ = correlation;
        }

        // Detect a local maximum in the output of the correlator.
        correlation_history_.Write(correlation);

        bool peak = (correlation_history_[1] == maximum_) &&
                    (correlation_history_[0] < maximum_) &&
                    (maximum_ >= kPeakThreshold);

        if (peak)
        {
            // We can approximate the sub-sample position of the peak by
            // comparing the relative correlation of the samples before and
            // after the raw peak.
            float left = correlation_history_[1] - correlation_history_[2];
            float right = correlation_history_[1] - correlation_history_[0];
            tilt_ = 0.5f * (left - right) / (left + right);
        }

        return peak;
    }

    float output(void)
    {
        return correlation_history_[0];
    }

    float tilt(void)
    {
        return tilt_;
    }
};

}
