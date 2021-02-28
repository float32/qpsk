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

// FIFO structures based on "Correct and Efficient Bounded FIFO Queues"
// by Nhat Minh Lê, Adrien Guatto, Albert Cohen, Antoniu Pop
// https://www.irif.fr/~guatto/papers/sbac13.pdf

#pragma once

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

    bool empty(void)
    {
        return !available();
    }

    uint32_t available(void)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        uint32_t tail = tail_.load(std::memory_order_acquire);
        return tail - head;
    }

    bool full(void)
    {
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        uint32_t head = head_.load(std::memory_order_acquire);
        return tail - head >= size;
    }

    bool Push(T item)
    {
        return Push(&item, 1);
    }

    bool Push(T* buffer, uint32_t length)
    {
        uint32_t tail = tail_.load(std::memory_order_relaxed);
        uint32_t head = head_.load(std::memory_order_acquire);

        if (tail - head > size - length)
        {
            return false;
        }

        for (uint32_t i = 0; i < length; i++)
        {
            data_[(tail + i) % size] = buffer[i];
        }

        tail_.store(tail + length, std::memory_order_release);
        return true;
    }

    bool Peek(T& item)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        uint32_t tail = tail_.load(std::memory_order_acquire);

        if (tail - head < 1)
        {
            return false;
        }

        item = data_[head % size];
        return true;
    }

    bool Pop(T& item)
    {
        uint32_t head = head_.load(std::memory_order_relaxed);
        uint32_t tail = tail_.load(std::memory_order_acquire);

        if (tail - head < 1)
        {
            return false;
        }

        item = data_[head % size];
        head_.store(head + 1, std::memory_order_release);
        return true;
    }

    bool Pop(void)
    {
        T item;
        return Pop(item);
    }
};

}
