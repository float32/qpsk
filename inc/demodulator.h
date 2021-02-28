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
#include "carrier_rejection_filter.h"
#include "correlator.h"
#include "one_pole.h"
#include "pll.h"
#include "util.h"
#include "window.h"

namespace qpsk
{

template <uint32_t sample_rate, uint32_t symbol_rate>
class Demodulator
{
public:
    void Init(void)
    {
        state_ = STATE_WAIT_TO_SETTLE;

        hpf_.Init(0.001f);
        follower_.Init(0.0001f);
        agc_gain_ = 1.f;

        pll_.Init(1.f / kSymbolDuration);
        crf_i_.Init();
        crf_q_.Init();

        correlator_.Init();

        q_history_.Init();
        i_history_.Init();

        decision_phase_ = 0.f;
        skipped_samples_ = 0;
        carrier_sync_count_ = 0;

        correlation_peaks_ = 0;
        avg_phase_x_.Init();
        avg_phase_y_.Init();

        early_ = false;
        late_ = false;
        decide_ = false;
    }

    void Reset(void)
    {
        Init();
    }

    void BeginCarrierSync(void)
    {
        state_ = STATE_CARRIER_SYNC;
        pll_.Sync();
        carrier_sync_count_ = 0;
    }

    bool Process(uint8_t& symbol, float sample)
    {
        sample = hpf_.Process(sample);

        float env = Abs(sample);

        follower_.Process(env);
        float level = signal_power();
        sample *= agc_gain_;

        if (state_ == STATE_WAIT_TO_SETTLE)
        {
            if (skipped_samples_ < kSettlingTime)
            {
                skipped_samples_++;
            }
            else if (level > kLevelThreshold)
            {
                skipped_samples_ = 0;
                state_ = STATE_SENSE_GAIN;
            }
        }
        else if (state_ == STATE_SENSE_GAIN)
        {
            if (skipped_samples_ < kSettlingTime)
            {
                skipped_samples_++;
            }
            else if (level > kLevelThreshold)
            {
                agc_gain_ = 0.64f / level;
                BeginCarrierSync();
            }
            else
            {
                state_ = STATE_WAIT_TO_SETTLE;
            }
        }
        else if (state_ != STATE_ERROR)
        {
            if (level < kLevelThreshold)
            {
                state_ = STATE_ERROR;
            }
            else
            {
                return Demodulate(symbol, sample);
            }
        }

        return false;
    }

    bool error(void)
    {
        return state_ == STATE_ERROR;
    }

    // Accessors for debug and simulation
    uint32_t state(void)          {return state_;}
    float    pll_phase(void)      {return pll_.phase();}
    float    pll_error(void)      {return pll_.error();}
    float    pll_step(void)       {return pll_.step();}
    float    decision_phase(void) {return decision_phase_;}
    float    signal_power(void)   {return follower_.output();}
    float    recovered_i(void)    {return crf_i_.output();}
    float    recovered_q(void)    {return crf_q_.output();}
    float    correlation(void)    {return correlator_.output();}
    bool     early(void)          {return early_;}
    bool     late(void)           {return late_;}
    bool     decide(void)         {return decide_;}

protected:
    static constexpr uint32_t kSettlingTime = sample_rate * 0.25f;
    static constexpr float kLevelThreshold = 0.05f;
    static constexpr uint32_t kCarrierSyncLength = symbol_rate * 0.025f;
    static constexpr uint32_t kNumCorrelationPeaks = 8;
    static_assert(sample_rate % symbol_rate == 0);
    static constexpr uint32_t kSymbolDuration = sample_rate / symbol_rate;

    enum State
    {
        STATE_WAIT_TO_SETTLE,
        STATE_SENSE_GAIN,
        STATE_CARRIER_SYNC,
        STATE_ALIGN,
        STATE_OK,
        STATE_ERROR,
    };

    State state_;

    OnePoleHighpass hpf_;
    OnePoleLowpass follower_;
    float agc_gain_;

    PhaseLockedLoop pll_;
    CarrierRejectionFilter<kSymbolDuration> crf_i_;
    CarrierRejectionFilter<kSymbolDuration> crf_q_;

    Correlator<kSymbolDuration> correlator_;
    Window<float, kSymbolDuration> q_history_;
    Window<float, kSymbolDuration> i_history_;

    float decision_phase_;
    uint32_t skipped_samples_;
    uint32_t carrier_sync_count_;

    uint32_t correlation_peaks_;
    Window<float, kNumCorrelationPeaks> avg_phase_x_;
    Window<float, kNumCorrelationPeaks> avg_phase_y_;

    bool early_;
    bool late_;
    bool decide_;

    void BeginAlignment(void)
    {
        state_ = STATE_ALIGN;
        decision_phase_ = 0.f;
        correlator_.Reset();
        correlation_peaks_ = 0;
    }

    bool Demodulate(uint8_t& symbol, float sample)
    {
        float i_osc = Cosine(pll_.phase());
        float q_osc = -Sine(pll_.phase());
        float i = crf_i_.Process(2.f * sample * i_osc);
        float q = crf_q_.Process(2.f * sample * q_osc);
        q_history_.Write(q);
        i_history_.Write(i);

        float phase_error;

        if (state_ == STATE_CARRIER_SYNC)
        {
            phase_error = q - i; // Lock to (-1 -1) during sync phase
        }
        else
        {
            phase_error = (q > 0 ? i : -i) - (i > 0 ? q : -q);
        }

        float prev_phase = pll_.phase();
        pll_.Process(phase_error / 16.f);
        float phase = pll_.phase();
        bool wrapped = prev_phase > phase;

        if (!wrapped)
        {
            decide_ = (prev_phase < decision_phase_) &&
                     (phase >= decision_phase_);
        }
        else
        {
            decide_ = (prev_phase < decision_phase_) ||
                     (phase >= decision_phase_);
        }

        if (state_ == STATE_CARRIER_SYNC)
        {
            // We let the PLL sync to a string of zeros, then wait for the
            // nonzero symbol which marks the end of sync and start of
            // alignment. Only then do we actually enter the alignment state,
            // since the alignment correlator is a performance bottleneck.
            // This allows us to work with less processing power even if the
            // CPU couldn't otherwise run the correlator in real-time, since
            // alignment lasts only a few milliseconds and the input is
            // buffered in a FIFO.
            if (decide_)
            {
                uint8_t symbol = DecideSymbol(false);

                if (carrier_sync_count_ < kCarrierSyncLength)
                {
                    if (symbol == 0)
                    {
                        carrier_sync_count_++;
                    }
                    else
                    {
                        carrier_sync_count_ = 0;
                    }
                }
                else if (symbol != 0)
                {
                    BeginAlignment();
                }
            }
        }
        else if (state_ == STATE_ALIGN)
        {
            if (correlation_peaks_ == kNumCorrelationPeaks)
            {
                // Make sure we don't immediately demodulate a symbol off
                // the end of the alignment sequence, since the averaged
                // decision phase might be just after our current phase.
                float delta = decision_phase_ - pll_.phase();
                delta = FractionalPart(delta + 1.f);

                if (delta > 0.5f)
                {
                    state_ = STATE_OK;
                }
            }
            else if (correlator_.Process(i, q))
            {
                correlation_peaks_++;
                float correlated_phase = FractionalPart(prev_phase +
                    pll_.step() * correlator_.tilt());
                avg_phase_x_.Write(Cosine(correlated_phase));
                avg_phase_y_.Write(Sine(correlated_phase));
                decision_phase_ =
                    VectorToPhase(avg_phase_x_.sum(), avg_phase_y_.sum());
            }
        }
        else if (state_ == STATE_OK)
        {
            if (decide_)
            {
                symbol = DecideSymbol(true);
                return true;
            }
        }

        return false;
    }

    static constexpr uint32_t kLatest   = 0;
    static constexpr uint32_t kLate     = 1;
    static constexpr uint32_t kEarly    = kSymbolDuration - 2;
    static constexpr uint32_t kEarliest = kSymbolDuration - 1;

    template <class WindowT>
    float SumOnTime(float sum, WindowT& history)
    {
        return sum - history[kLatest] - history[kEarliest];
    }

    template <class WindowT>
    float SumEarly(float sum, WindowT& history)
    {
        return sum - history[kLate] - history[kLatest];
    }

    template <class WindowT>
    float SumLate(float sum, WindowT& history)
    {
        return sum - history[kEarly] - history[kEarliest];
    }

    uint8_t DecideSymbol(bool adjust_timing)
    {
        float q_sum = q_history_.sum();
        float i_sum = i_history_.sum();

        if (adjust_timing)
        {
            float q_sum_late    = SumLate(q_sum, q_history_);
            float i_sum_late    = SumLate(i_sum, i_history_);
            float q_sum_early   = SumEarly(q_sum, q_history_);
            float i_sum_early   = SumEarly(i_sum, i_history_);
            float q_sum_on_time = SumOnTime(q_sum, q_history_);
            float i_sum_on_time = SumOnTime(i_sum, i_history_);

            float late_strength    = Abs(q_sum_late)    + Abs(i_sum_late);
            float on_time_strength = Abs(q_sum_on_time) + Abs(i_sum_on_time);
            float early_strength   = Abs(q_sum_early)   + Abs(i_sum_early);

            float threshold = 1.25f * on_time_strength;

            early_ = (early_strength > threshold);
            late_ = (late_strength > threshold);

            if (late_ && !early_)
            {
                q_sum = q_sum_late;
                i_sum = i_sum_late;
            }
            else if (early_ && !late_)
            {
                q_sum = q_sum_early;
                i_sum = i_sum_early;
            }
        }
        else
        {
            q_sum = SumOnTime(q_sum, q_history_);
            i_sum = SumOnTime(i_sum, i_history_);
        }

        return (i_sum < 0 ? 0 : 2) + (q_sum < 0 ? 0 : 1);
    }
};

}
