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
    RESULT_PAGE_COMPLETE,
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
    ERROR_TIMEOUT,
};

template <uint32_t samples_per_symbol,
          uint32_t packet_size,
          uint32_t page_size,
          uint32_t fifo_capacity = 1024>
class Decoder
{
public:
    void Init(uint32_t crc_seed)
    {
        demodulator_.Init();
        packet_.Init(crc_seed);
        page_.Init();
        recent_symbols_.Init();
        last_symbol_ = 0;
        Reset();
    }

    void Reset(void)
    {
        demodulator_.SyncCarrier(true);
        RestartSync();
        packet_count_ = 0;
        samples_.Flush();

        packet_.Reset();
        page_.Clear();

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
        if (samples_.Full() && state_ != STATE_WRITING)
        {
            state_ = STATE_ERROR;
            error_ = ERROR_OVERFLOW;
        }

        samples_.Push(sample);
    }

    void Push(float *buffer, uint32_t length)
    {
        for (uint32_t i = 0; i < length; i++)
        {
            Push(buffer[i]);
        }
    }

    uint32_t* GetPage(void)
    {
        return page_.data();
    }

    Result Receive(void)
    {
        if (state_ == STATE_WRITING)
        {
            page_.Clear();
            RestartSync();
            demodulator_.SyncCarrier(false);
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

            demodulator_.Process(sample);
            uint8_t symbol;

            while (demodulator_.PopSymbol(symbol))
            {
                recent_symbols_.Push(symbol);
                last_symbol_ = symbol;

                if (state_ == STATE_SYNCING)
                {
                    Sync(symbol);
                }
                else if (state_ == STATE_DECODING)
                {
                    packet_.WriteSymbol(symbol);

                    if (packet_.Complete())
                    {
                        if (packet_.Valid())
                        {
                            packet_count_++;
                            page_.AppendPacket(packet_);

                            RestartSync();
                            demodulator_.SyncDecision();

                            if (page_.Complete())
                            {
                                state_ = STATE_WRITING;
                                return RESULT_PAGE_COMPLETE;
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
                else if (state_ == STATE_END)
                {
                    return RESULT_END;
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
    uint8_t ExpectedSymbolMask(void) {return expected_;}
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
    static constexpr uint32_t kPreambleSize = 16;
    static constexpr uint32_t kMaxSyncDuration = 1000;

    static_assert(page_size % packet_size == 0);

    static constexpr uint32_t SymbolMask(uint32_t x)
    {
        return (1 << x);
    }

    enum State
    {
        STATE_SYNCING,
        STATE_DECODING,
        STATE_WRITING,
        STATE_END,
        STATE_ERROR,
    };

    RingBuffer<float, fifo_capacity> samples_;
    RingBuffer<uint8_t, 128> recent_symbols_; // For debug
    uint8_t last_symbol_; // For sim
    Demodulator<samples_per_symbol, 128> demodulator_;
    State state_;
    Error error_;
    Packet<packet_size> packet_;
    Page<page_size> page_;
    uint32_t packet_count_;
    uint32_t sync_blank_size_;
    uint32_t preamble_remaining_size_;
    uint8_t expected_;
    bool abort_;

    void RestartSync(void)
    {
        state_ = STATE_SYNCING;
        error_ = ERROR_NONE;

        expected_ = SymbolMask(4);
        sync_blank_size_ = 0;

        preamble_remaining_size_ = kPreambleSize;
    }

    void Sync(uint8_t symbol)
    {
        if (!(SymbolMask(symbol) & expected_))
        {
            state_ = STATE_ERROR;
            error_ = ERROR_SYNC;
            return;
        }

        switch (symbol)
        {
        case 4:
            if (++sync_blank_size_ >= kMaxSyncDuration && packet_count_)
            {
                state_ = STATE_END;
                return;
            }

            expected_ = SymbolMask(1) | SymbolMask(2) | SymbolMask(4);
            preamble_remaining_size_ = kPreambleSize;
            break;

        case 3:
            expected_ = SymbolMask(0);
            preamble_remaining_size_--;
            break;

        case 2:
            expected_ = SymbolMask(1);
            break;

        case 1:
            expected_ = SymbolMask(2) | SymbolMask(3);
            break;

        case 0:
            expected_ = SymbolMask(3);
            preamble_remaining_size_--;
            break;
        }

        if (preamble_remaining_size_ == 0)
        {
            packet_.Reset();
            state_ = STATE_DECODING;
        }
        else
        {
            state_ = STATE_SYNCING;
        }
    }
};

}
