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

#pragma once

#include <cstdint>
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
        recent_symbols_.Init();
        last_symbol_ = 0;
        Reset();
    }

    void Reset(void)
    {
        demodulator_.Reset();
        BeginSync();
        samples_.Flush();

        packet_.Reset();
        block_.Clear();

        abort_ = false;
    }

    void Abort(void)
    {
        abort_ = true;
    }

    Error GetError(void)
    {
        return (state_ == STATE_ERROR) ? error_ : ERROR_NONE;
    }

    void Push(float sample)
    {
        if (samples_.Full() && state_ != STATE_WRITE && state_ != STATE_END)
        {
            state_ = STATE_ERROR;
            error_ = ERROR_OVERFLOW;
        }

        samples_.Push(sample);
    }

    void Push(float* buffer, uint32_t length)
    {
        for (uint32_t i = 0; i < length; i++)
        {
            Push(buffer[i]);
        }
    }

    uint32_t* GetBlock(void)
    {
        return block_.data();
    }

    Result Receive(void)
    {
        if (state_ == STATE_WRITE)
        {
            block_.Clear();
            demodulator_.BeginCarrierSync();
            BeginSync();
        }
        else if (state_ == STATE_END)
        {
            return RESULT_END;
        }

        float sample;
        while (samples_.Pop(sample))
        {
            if (abort_)
            {
                state_ = STATE_ERROR;
                error_ = ERROR_ABORT;
                return RESULT_ERROR;
            }

            uint8_t symbol;
            if (demodulator_.Process(symbol, sample))
            {
                recent_symbols_.Push(symbol);
                last_symbol_ = symbol;

                if (state_ == STATE_SYNC)
                {
                    Sync(symbol);
                }
                else if (state_ == STATE_DECODE)
                {
                    packet_.WriteSymbol(symbol);

                    if (packet_.Complete())
                    {
                        if (packet_.Valid())
                        {
                            block_.AppendPacket(packet_);
                            packet_.Reset();

                            if (block_.Complete())
                            {
                                state_ = STATE_WRITE;
                                return RESULT_BLOCK_COMPLETE;
                            }
                        }
                        else
                        {
                            state_ = STATE_ERROR;
                            error_ = ERROR_CRC;
                        }

                        return RESULT_PACKET_COMPLETE;
                    }
                }
                else if (state_ == STATE_ERROR)
                {
                    return RESULT_ERROR;
                }
            }
        }

        return RESULT_NONE;
    }

    // Accessors for debug and simulation
    uint8_t* GetPacket(void)         {return packet_.data();}
    uint32_t CalculatedCRC(void)     {return packet_.CalculatedCRC();}
    uint32_t ExpectedCRC(void)       {return packet_.ExpectedCRC();}
    uint8_t PacketByte(void)         {return packet_.last_byte();}
    float PllPhase(void)             {return demodulator_.PllPhase();}
    float PllPhaseError(void)        {return demodulator_.PllPhaseError();}
    float PllPhaseIncrement(void)    {return demodulator_.PllPhaseIncrement();}
    float DecisionPhase(void)        {return demodulator_.DecisionPhase();}
    uint32_t SymbolsAvailable(void)  {return recent_symbols_.Available();}
    bool PopSymbol(uint8_t& symbol)  {return recent_symbols_.Pop(symbol);}
    float SignalPower(void)          {return demodulator_.SignalPower();}
    uint32_t state(void)             {return state_;}
    float RecoveredI(void)           {return demodulator_.RecoveredI();}
    float RecoveredQ(void)           {return demodulator_.RecoveredQ();}
    float Correlation(void)          {return demodulator_.Correlation();}
    uint32_t DemodulatorState(void)  {return demodulator_.state();}
    uint8_t LastSymbol(void)         {return last_symbol_;}
    bool    Early(void)              {return demodulator_.Early();}
    bool    Late(void)               {return demodulator_.Late();}
    bool    Decide(void)             {return demodulator_.Decide();}


protected:
    static constexpr uint32_t kMarkerLength = 16;
    static constexpr uint32_t kBlockMarker = 0xCCCCCCCC;
    static constexpr uint32_t kEndMarker = 0xF0F0F0F0;
    static_assert(block_size % packet_size == 0);

    enum State
    {
        STATE_SYNC,
        STATE_DECODE,
        STATE_WRITE,
        STATE_END,
        STATE_ERROR,
    };

    RingBuffer<float, fifo_capacity> samples_;
    RingBuffer<uint8_t, 128> recent_symbols_; // For debug
    uint8_t last_symbol_; // For sim
    Demodulator<sample_rate, symbol_rate> demodulator_;
    State state_;
    Error error_;
    Packet<packet_size> packet_;
    uint32_t marker_count_;
    uint32_t marker_code_;
    Block<block_size> block_;
    bool abort_;

    void BeginSync(void)
    {
        state_ = STATE_SYNC;
        error_ = ERROR_NONE;

        marker_count_ = kMarkerLength;
        marker_code_ = 0;
    }

    void Sync(uint8_t symbol)
    {
        marker_code_ = (marker_code_ << 2) | symbol;
        marker_count_--;

        if (marker_count_ == 0)
        {
            switch (marker_code_)
            {
            case kBlockMarker:
                state_ = STATE_DECODE;
                break;

            case kEndMarker:
                state_ = STATE_END;
                return;

            default:
                state_ = STATE_ERROR;
                error_ = ERROR_SYNC;
                return;
            }
        }
        else
        {
            state_ = STATE_SYNC;
        }
    }
};

}
