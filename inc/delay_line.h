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

template <typename T, uint32_t size>
class DelayLine
{
protected:
    T buffer_[size];
    uint32_t head_;

    void Push(T value)
    {
        buffer_[head_] = value;
        head_ = (head_ + 1) % size;
    }

public:
    void Init(T value = 0)
    {
        for (uint32_t i = 0; i < size; i++)
        {
            buffer_[i] = value;
        }

        head_ = 0;
    }

    T Tap(uint32_t i = 0)
    {
        // undefined for i >= size
        return buffer_[(head_ + size - 1 - i) % size];
    }

    T Process(T input)
    {
        T output = buffer_[head_];
        Push(input);
        return output;
    }
};

}
