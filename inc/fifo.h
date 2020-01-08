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
