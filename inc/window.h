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
#include "delay_line.h"

namespace qpsk
{

template <typename T, uint32_t length_>
class Window
{
protected:
    static constexpr uint32_t kLengthBits = std::ceil(std::log2(length_));
    DelayLine<T, (1 << kLengthBits)> delay_line_;
    T sum_;
    T refresh_;
    uint32_t age_;

public:
    void Init(void)
    {
        delay_line_.Init(0);
        sum_ = 0;
        refresh_ = 0;
        age_ = 0;
    }

    void Write(T in)
    {
        refresh_ += in;
        age_++;

        if (age_ < length_)
        {
            sum_ += in - delay_line_.Tap(length_ - 1);
        }
        else
        {
            sum_ = refresh_;
            refresh_ = 0;
            age_ = 0;
        }

        delay_line_.Process(in);
    }

    T operator[](uint32_t i)
    {
        return delay_line_.Tap(i);
    }

    T sum(void)
    {
        return sum_;
    }

    T average(void)
    {
        return sum_ / length_;
    }

    static constexpr uint32_t length(void)
    {
        return length_;
    }
};

template <typename T, uint32_t length_, uint32_t width_>
class Bay
{
protected:
    Window<T, length_> window_[width_];
    T sum_;

public:
    void Init(void)
    {
        for (uint32_t i = 0; i < width_; i++)
        {
            window_[i].Init();
        }

        sum_ = 0;
    }

    void Write(T in)
    {
        sum_ = 0;
        T out;

        for (uint32_t i = 0; i < width_; i++)
        {
            out = window_[i][length_ - 1];
            window_[i].Write(in);
            in = out;
            sum_ += window_[i].sum();
        }
    }

    Window<T, length_>& operator[](uint32_t i)
    {
        return window_[i];
    }

    T sum(void)
    {
        return sum_;
    }

    T average(void)
    {
        return sum_ / (length_ * width_);
    }

    static constexpr uint32_t length(void)
    {
        return length_;
    }

    static constexpr uint32_t width(void)
    {
        return width_;
    }
};

}
