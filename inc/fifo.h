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

#ifndef QPSK_FIFO_H_
#define QPSK_FIFO_H_

#include <cstdint>

namespace qpsk
{

template<typename T, uint32_t size>
class Fifo
{
protected:
    T buffer_[size];
    uint32_t head_;
    uint32_t tail_;
    uint32_t pushed_;
    uint32_t popped_;

public:
    void Init(void)
    {
        Flush();
    }

    void Flush(void)
    {
        head_ = 0;
        tail_ = 0;
        pushed_ = 0;
        popped_ = 0;
    }

    bool Empty(void)
    {
        return (pushed_ == popped_);
    }

    uint32_t Available(void)
    {
        return (pushed_ - popped_);
    }

    bool Full(void)
    {
        return (Available() == size);
    }

    void Push(T item)
    {
        buffer_[tail_] = item;
        tail_ = (tail_ + 1) % size;
        pushed_++;
    }

    T Peek(void)
    {
        return buffer_[head_];
    }

    T Pop(void)
    {
        T item = buffer_[head_];
        head_ = (head_ + 1) % size;
        popped_++;

        return item;
    }
};

}

#endif
