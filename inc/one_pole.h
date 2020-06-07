// MIT License
//
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

#ifndef QPSK_ONE_POLE_H_
#define QPSK_ONE_POLE_H_

#include <cstdint>
#include <cmath>

namespace qpsk
{

class OnePoleLowpass
{
protected:
    static constexpr float Factor(float freq)
    {
        return 1 - std::exp(-2 * M_PI * freq);
    }

    float factor_;
    float history_;

public:
    void Init(float normalized_frequency)
    {
        factor_ = Factor(normalized_frequency);
        history_ = 0;
    }

    void Reset(void)
    {
        history_ = 0;
    }

    float Process(float in)
    {
        history_ += factor_ * (in - history_);
        return history_;
    }

    float Output(void)
    {
        return history_;
    }
};

class OnePoleHighpass : public OnePoleLowpass
{
protected:
    using super = OnePoleLowpass;
    float output_;

public:
    void Init(float normalized_frequency)
    {
        super::Init(normalized_frequency);
        output_ = 0.f;
    }

    float Process(float in)
    {
        output_ = in - super::Process(in);
        return output_;
    }

    float Output(void)
    {
        return output_;
    }
};

}

#endif
