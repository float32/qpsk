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
#include <atomic>

namespace qpsk
{

template<typename T, uint32_t size>
class Fifo
{
protected:
    static_assert((size & (size - 1)) == 0, "size must be a power of 2");
    std::atomic<uint32_t> head_;
    std::atomic<uint32_t> tail_;
    T data_[size];

public:
    void Init(void)
    {
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    void Flush(void)
    {
        uint32_t tail = tail_.load(std::memory_order_acquire);
        head_.store(tail, std::memory_order_release);
    }

    bool Empty(void)
    {
        return !Available();
    }

    uint32_t Available(void)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        uint32_t tail = tail_.load(std::memory_order_acquire);
        return tail - head;
    }

    bool Full(void)
    {
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        uint32_t head = head_.load(std::memory_order_acquire);
        return tail - head == size;
    }

    void Push(T item)
    {
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        data_[tail % size] = item;
        tail_.store(tail + 1, std::memory_order_release);
    }

    T Peek(void)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        return data_[head % size];
    }

    T Pop(void)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        T item = data_[head % size];
        head_.store(head + 1, std::memory_order_release);
        return item;
    }
};

}

#endif
