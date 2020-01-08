#ifndef QPSK_DELAY_LINE_H_
#define QPSK_DELAY_LINE_H_

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

#endif
