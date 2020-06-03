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

#ifndef QPSK_DECODER_H_
#define QPSK_DECODER_H_

#include <cstdint>
#include "inc/demodulator.h"
#include "inc/packet.h"
#include "inc/fifo.h"

namespace qpsk
{

enum Result
{
    RESULT_NONE,
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
    ERROR_PAGE_WRITE,
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
        symbols_.Init();
        Reset();
    }

    void Reset(void)
    {
        enabled_ = false;

        demodulator_.SyncCarrier(true);
        RestartSync();
        packet_count_ = 0;
        samples_.Flush();

        packet_.Reset();
        page_.Clear();

        abort_ = false;
        enabled_ = true;
    }

    void Abort(void)
    {
        enabled_ = false;
        abort_ = true;
    }

    void Resync(void)
    {
        enabled_ = false;

        demodulator_.SyncCarrier(false);
        RestartSync();

        enabled_ = true;
    }

    uint8_t* GetPacket(void)
    {
        return packet_.data();
    }

    uint32_t CalculatedCRC(void)
    {
        return packet_.CalculatedCRC();
    }

    uint32_t ExpectedCRC(void)
    {
        return packet_.ExpectedCRC();
    }

    Error GetError(void)
    {
        return (state_ == STATE_ERROR) ? error_ : ERROR_NONE;
    }

    void Push(float sample)
    {
        if (!enabled_)
        {
            return;
        }

        if (samples_.Full())
        {
            state_ = STATE_ERROR;
            error_ = ERROR_OVERFLOW;
        }
        else
        {
            samples_.Push(sample);
        }
    }

    void Push(float *buffer, uint32_t length)
    {
        if (!enabled_)
        {
            return;
        }

        for (uint32_t i = 0; i < length; i++)
        {
            Push(buffer[i]);
        }
    }

    bool Full(void)
    {
        return samples_.Full();
    }

    using PageCallback = bool(uint32_t*);
    using PacketCallback = void(void);

    Result Receive(PageCallback page_cb,
        PacketCallback packet_cb = nullptr, uint32_t timeout = 0)
    {
        uint32_t elapsed = 0;

        for (;;)
        {
            while (samples_.Available() && demodulator_.SymbolsAvailable() < 1)
            {
                float sample = samples_.Pop();
                demodulator_.Process(sample);
                elapsed++;
            }

            while (demodulator_.SymbolsAvailable())
            {
                uint8_t symbol = demodulator_.PopSymbol();

                // This fifo has no purpose except debugging
                if (symbols_.Full()) symbols_.Pop();
                symbols_.Push(symbol);

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

                            if (packet_cb)
                            {
                                packet_cb();
                            }

                            Realign();
                            page_.AppendPacket(packet_);

                            if (page_.Complete())
                            {
                                enabled_ = false;

                                if (page_cb && !page_cb(page_.data()))
                                {
                                    state_ = STATE_ERROR;
                                    error_ = ERROR_PAGE_WRITE;
                                }
                                else
                                {
                                    page_.Clear();
                                    enabled_ = true;
                                    Resync();
                                }
                            }
                        }
                        else
                        {
                            state_ = STATE_ERROR;
                            error_ = ERROR_CRC;
                        }
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

            if (abort_)
            {
                state_ = STATE_ERROR;
                error_ = ERROR_ABORT;
                return RESULT_ERROR;
            }

            if (timeout > 0 && elapsed >= timeout)
            {
                state_ = STATE_ERROR;
                error_ = ERROR_TIMEOUT;
                return RESULT_ERROR;
            }
        }
    }

    float PllPhase(void)
    {
        return demodulator_.PllPhase();
    }

    float PllPhaseIncrement(void)
    {
        return demodulator_.PllPhaseIncrement();
    }

    float DecisionPhase(void)
    {
        return demodulator_.DecisionPhase();
    }

    uint32_t SymbolsAvailable(void)
    {
        return symbols_.Available();
    }

    uint8_t PopSymbol(void)
    {
        return symbols_.Pop();
    }

    uint8_t ExpectedSymbolMask(void)
    {
        return expected_;
    }

    float SignalPower(void)
    {
        return demodulator_.SignalPower();
    }

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
        STATE_END,
        STATE_ERROR,
    };

    Fifo<float, fifo_capacity> samples_;
    Fifo<uint8_t, 128> symbols_;
    Demodulator<samples_per_symbol, 128> demodulator_;
    State state_;
    Error error_;
    Packet<packet_size> packet_;
    Page<page_size> page_;
    uint32_t packet_count_;
    uint32_t sync_blank_size_;
    uint32_t preamble_remaining_size_;
    uint8_t expected_;
    bool enabled_;
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

    void Realign(void)
    {
        enabled_ = false;

        RestartSync();
        demodulator_.SyncDecision();

        enabled_ = true;
    }
};

}

#endif
