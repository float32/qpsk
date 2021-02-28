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
#include <atomic>
#include "inc/demodulator.h"
#include "inc/packet.h"
#include "inc/fifo.h"

namespace qpsk
{

enum Result
{
    RESULT_NONE,
    RESULT_PACKET_COMPLETE,
    RESULT_BLOCK_COMPLETE,
    RESULT_END,
    RESULT_ERROR,
};

enum Error
{
    ERROR_NONE,
    ERROR_SYNC,
    ERROR_CRC,
    ERROR_OVERFLOW,
    ERROR_ABORT,
};

template <uint32_t sample_rate,
          uint32_t symbol_rate,
          uint32_t packet_size,
          uint32_t block_size,
          uint32_t fifo_capacity = 256>
class Decoder
{
public:
    void Init(uint32_t crc_seed)
    {
        demodulator_.Init();
        packet_.Init(crc_seed);
        block_.Init();
        last_symbol_ = 0;
        Reset();
    }

    void Reset(void)
    {
        demodulator_.Reset();
        BeginSync();

        packet_.Reset();
        block_.Clear();

        abort_.store(false, std::memory_order_relaxed);
        error_ = ERROR_NONE;

        FlushSamples();
    }

    void Push(float* buffer, uint32_t length)
    {
        if (!samples_.Push(buffer, length))
        {
            overflow_.store(true, std::memory_order_release);
        }
    }

    void Push(float sample)
    {
        Push(&sample, 1);
    }

    Result Process(void)
    {
        if (state_ == STATE_WRITE)
        {
            block_.Clear();
            demodulator_.BeginCarrierSync();
            BeginSync();
            FlushSamples();
        }
        else if (state_ == STATE_END)
        {
            return RESULT_END;
        }

        Result result = RESULT_NONE;
        float sample;

        while (result == RESULT_NONE && samples_.Pop(sample))
        {
            uint8_t symbol;

            if (abort_.load(std::memory_order_relaxed))
            {
                result = ReportError(ERROR_ABORT);
            }
            else if (overflow_.load(std::memory_order_relaxed))
            {
                result = ReportError(ERROR_OVERFLOW);
            }
            else if (demodulator_.error())
            {
                result = ReportError(ERROR_SYNC);
            }
            else if (demodulator_.Process(symbol, sample))
            {
                last_symbol_ = symbol;

                if (state_ == STATE_SYNC)
                {
                    result = Sync(symbol);
                }
                else if (state_ == STATE_DECODE)
                {
                    result = Decode(symbol);
                }
                else if (state_ == STATE_ERROR)
                {
                    result = RESULT_ERROR;
                }
            }
        }

        return result;
    }

    void Abort(void)
    {
        abort_.store(true, std::memory_order_relaxed);
    }

    Error error(void)
    {
        return (state_ == STATE_ERROR) ? error_ : ERROR_NONE;
    }

    const uint32_t* block_data(void)
    {
        return block_.data();
    }

    // Accessors for debug and simulation
    uint8_t* packet_data(void)       {return packet_.data();}
    uint8_t  packet_byte(void)       {return packet_.last_byte();}
    float    pll_phase(void)         {return demodulator_.pll_phase();}
    float    pll_error(void)         {return demodulator_.pll_error();}
    float    pll_step(void)          {return demodulator_.pll_step();}
    float    decision_phase(void)    {return demodulator_.decision_phase();}
    float    signal_power(void)      {return demodulator_.signal_power();}
    uint32_t state(void)             {return state_;}
    float    recovered_i(void)       {return demodulator_.recovered_i();}
    float    recovered_q(void)       {return demodulator_.recovered_q();}
    float    correlation(void)       {return demodulator_.correlation();}
    uint32_t demodulator_state(void) {return demodulator_.state();}
    uint8_t  last_symbol(void)       {return last_symbol_;}
    bool     early(void)             {return demodulator_.early();}
    bool     late(void)              {return demodulator_.late();}
    bool     decide(void)            {return demodulator_.decide();}
    uint32_t samples_available(void) {return samples_.available();}

protected:
    static constexpr uint32_t kMarkerLength = 16;
    static constexpr uint32_t kBlockMarker = 0xCCCCCCCC;
    static constexpr uint32_t kEndMarker = 0xF0F0F0F0;
    static_assert(block_size % packet_size == 0);
    static_assert(packet_size % 4 == 0);

    enum State
    {
        STATE_SYNC,
        STATE_DECODE,
        STATE_WRITE,
        STATE_END,
        STATE_ERROR,
    };

    Fifo<float, fifo_capacity> samples_;
    uint8_t last_symbol_; // For sim
    Demodulator<sample_rate, symbol_rate> demodulator_;
    State state_;
    Error error_;
    Packet<packet_size> packet_;
    uint32_t marker_count_;
    uint32_t marker_code_;
    Block<block_size> block_;
    std::atomic_bool abort_;
    std::atomic_bool overflow_;
    static_assert(std::atomic_bool::is_always_lock_free);

    void FlushSamples(void)
    {
        samples_.Flush();
        overflow_.store(false, std::memory_order_release);
    }

    void BeginSync(void)
    {
        state_ = STATE_SYNC;
        marker_count_ = kMarkerLength;
        marker_code_ = 0;
    }

    Result Sync(uint8_t symbol)
    {
        marker_code_ = (marker_code_ << 2) | symbol;
        marker_count_--;

        if (marker_count_ == 0)
        {
            if (marker_code_ == kBlockMarker)
            {
                state_ = STATE_DECODE;
                return RESULT_NONE;
            }
            else if (marker_code_ == kEndMarker)
            {
                state_ = STATE_END;
                return RESULT_END;
            }
            else
            {
                return ReportError(ERROR_SYNC);
            }
        }
        else
        {
            return RESULT_NONE;
        }
    }

    Result Decode(uint8_t symbol)
    {
        packet_.WriteSymbol(symbol);

        if (packet_.full())
        {
            if (packet_.valid())
            {
                block_.AppendPacket(packet_);
                packet_.Reset();

                if (block_.full())
                {
                    state_ = STATE_WRITE;
                    return RESULT_BLOCK_COMPLETE;
                }
            }
            else
            {
                return ReportError(ERROR_CRC);
            }

            return RESULT_PACKET_COMPLETE;
        }
        else
        {
            return RESULT_NONE;
        }
    }

    Result ReportError(Error error)
    {
        state_ = STATE_ERROR;
        error_ = error;
        return RESULT_ERROR;
    }
};

}
