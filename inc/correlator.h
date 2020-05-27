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

#ifndef QPSK_CORRELATOR_H_
#define QPSK_CORRELATOR_H_

#include <cstdint>
#include "window.h"

namespace qpsk
{

class Correlator
{
protected:
    static inline const uint8_t kAlignmentSequence[] = {2, 1};
    static constexpr uint32_t kLength = sizeof(kAlignmentSequence);

    Window<float, 3> history_;
    uint32_t age_;
    float maximum_;

public:
    void Init(void)
    {
        Reset();
    }

    void Reset(void)
    {
        history_.Init();
        age_ = 0;
        maximum_ = 0.f;
    }

    template <class bay>
    bool Process(bay& i_history, bay& q_history)
    {
        float correlation = 0.f;

        for (uint32_t i = 0; i < kLength; i++)
        {
            uint8_t symbol = kAlignmentSequence[kLength - 1 - i];
            uint8_t expected_i = (symbol & 2);
            uint8_t expected_q = (symbol & 1);

            float i_sum = i_history[i].Sum();
            float q_sum = q_history[i].Sum();

            correlation += expected_i ? i_sum : -i_sum;
            correlation += expected_q ? q_sum : -q_sum;
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

        history_.Write(correlation);
        age_++;

        // Detect a local maximum in the output of the correlator.
        uint32_t center = bay::length() / 2;

        uint8_t symbol = kAlignmentSequence[kLength - 1];

        bool i_correlated = (symbol & 2) ? (i_history[0][center] > 0) :
                                           (i_history[0][center] < 0);

        bool q_correlated = (symbol & 1) ? (q_history[0][center] > 0) :
                                           (q_history[0][center] < 0);

        bool peak = (age_ >= 3) && (history_[1] == maximum_);
        return peak && i_correlated && q_correlated;
    }

    static constexpr uint32_t length(void)
    {
        return kLength;
    }
};

}

#endif
