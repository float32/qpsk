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
#include "carrier_rejection_filter.h"
#include "correlator.h"
#include "fifo.h"
#include "one_pole.h"
#include "pll.h"
#include "util.h"
#include "window.h"

namespace qpsk
{

template <uint32_t symbol_duration, uint32_t fifo_capacity>
class Demodulator
{
public:
    void Init(void)
    {
        state_ = STATE_WAIT_TO_SETTLE;
        symbols_.Init();

        hpf_.Init(0.001f);
        follower_.Init(0.0001f);
        agc_gain_ = 1.f;

        pll_.Init(1.f / symbol_duration);
        crf_i_.Init();
        crf_q_.Init();

        correlator_.Init();

        Reset();

        early_ = false;
        late_ = false;
        decide_ = false;
    }

    bool PopSymbol(uint8_t& symbol)
    {
        return symbols_.Pop(symbol);
    }

    void SyncCarrier(bool discover)
    {
        skipped_samples_ = 0;
        skipped_symbols_ = 0;
        symbols_.Flush();

        if (discover)
        {
            follower_.Reset();
            state_ = STATE_WAIT_TO_SETTLE;
        }
        else
        {
            state_ = STATE_CARRIER_SYNC;
        }

        pll_.Sync();
        q_history_.Init();
        i_history_.Init();
    }

    void SyncDecision(void)
    {
        symbols_.Flush();
        state_ = STATE_DECISION_SYNC;
        decision_phase_ = 0.f;
        inhibit_decision_ = true;
        correlator_.Reset();
        skipped_symbols_ = 0;
    }

    void Process(float sample)
    {
        sample = hpf_.Process(sample);

        float env = Abs(sample);

        follower_.Process(env);
        float level = SignalPower();
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
                state_ = STATE_CARRIER_SYNC;
            }
            else
            {
                state_ = STATE_WAIT_TO_SETTLE;
            }
        }
        else
        {
            Demodulate(sample);
        }
    }

    // Accessors for debug and simulation
    uint32_t state(void)             {return state_;}
    float    PllPhase(void)          {return pll_.Phase();}
    float    PllPhaseError(void)     {return pll_.PhaseError();}
    float    PllPhaseIncrement(void) {return pll_.PhaseIncrement();}
    float    DecisionPhase(void)     {return decision_phase_;}
    float    SignalPower(void)       {return follower_.Output();}
    float    RecoveredI(void)        {return crf_i_.output();}
    float    RecoveredQ(void)        {return crf_q_.output();}
    float    Correlation(void)       {return correlator_.output();}
    bool     Early(void)             {return early_;}
    bool     Late(void)              {return late_;}
    bool     Decide(void)            {return decide_;}

protected:
    static constexpr uint32_t kSettlingTime = 1024;
    static constexpr uint32_t kObservationWindowLength = 512;
    static constexpr float kLevelThreshold = 0.05f;
    static constexpr uint32_t kNumSkippedZeroSymbols = 32;
    static constexpr uint32_t kNumLockedTemplateSymbols = 4;
    static constexpr uint32_t kSymbolDuration = symbol_duration;

    enum State
    {
        STATE_WAIT_TO_SETTLE,
        STATE_SENSE_GAIN,
        STATE_CARRIER_SYNC,
        STATE_DECISION_SYNC,
        STATE_OVERFLOW,
        STATE_OK,
    };

    State state_;
    Fifo<uint8_t, fifo_capacity> symbols_;

    OnePoleHighpass hpf_;
    OnePoleLowpass follower_;
    float agc_gain_;

    PhaseLockedLoop pll_;
    CarrierRejectionFilter<kSymbolDuration> crf_i_;
    CarrierRejectionFilter<kSymbolDuration> crf_q_;

    Correlator correlator_;
    Bay<float, kSymbolDuration, Correlator::length()> q_history_;
    Bay<float, kSymbolDuration, Correlator::length()> i_history_;

    float decision_phase_;
    bool inhibit_decision_;
    uint32_t skipped_samples_;
    uint32_t skipped_symbols_;

    bool early_;
    bool late_;
    bool decide_;

    void Reset(void)
    {
        q_history_.Init();
        i_history_.Init();

        skipped_samples_ = 0;
        skipped_symbols_ = 0;
        state_ = STATE_WAIT_TO_SETTLE;

        decision_phase_ = 0.f;
        inhibit_decision_ = false;
    }

    void Demodulate(float sample)
    {
        float phase = pll_.Phase();

        // demodulate
        float i_osc = Cosine(phase);
        float q_osc = Sine(phase);

        // carrier rejection filter
        float i = crf_i_.Process(sample * i_osc);
        float q = crf_q_.Process(sample * -q_osc);

        float phase_error;

        if (state_ == STATE_CARRIER_SYNC)
        {
            phase_error = q - i; // Lock to (-1 -1) during sync phase
        }
        else
        {
            phase_error = (q > 0 ? i : -i) - (i > 0 ? q : -q);
        }

        // PLL to lock onto the carrier
        pll_.Process(phase_error / 8.f);

        q_history_.Write(q);
        i_history_.Write(i);

        float prev_phase = phase;
        phase = pll_.Phase();
        bool wrapped = prev_phase > phase;

        bool decide;

        if (inhibit_decision_)
        {
            decide = false;
            inhibit_decision_ = false;
        }
        else if (!wrapped)
        {
            decide = (prev_phase < decision_phase_) &&
                     (phase >= decision_phase_);
        }
        else
        {
            decide = (prev_phase < decision_phase_) ||
                     (phase >= decision_phase_);
        }

        decide_ = decide;

        if (decide)
        {
            // In carrier sync mode, we just let the PLL stabilize until we
            // consistently decode a string of 0s.
            switch (state_)
            {
            case STATE_CARRIER_SYNC:
                if (DecideSymbol(false) == 0)
                {
                    if (++skipped_symbols_ == kNumSkippedZeroSymbols)
                    {
                        SyncDecision();
                    }
                }
                else
                {
                    skipped_symbols_ = 0;
                }
                break;

            case STATE_DECISION_SYNC:
                symbols_.Push(4);
                break;

            case STATE_OK:
                symbols_.Push(DecideSymbol(true));
                break;

            case STATE_WAIT_TO_SETTLE:
            case STATE_SENSE_GAIN:
            case STATE_OVERFLOW:
                break;
            }
        }

        if (state_ == STATE_DECISION_SYNC)
        {
            bool correlated = correlator_.Process(i_history_, q_history_);

            if (correlated)
            {
                if (++skipped_symbols_ == kNumLockedTemplateSymbols)
                {
                    state_ = STATE_OK;
                }

                decision_phase_ += prev_phase / kNumLockedTemplateSymbols;
            }
        }
    }

    static constexpr uint32_t kLatest   = 0;
    static constexpr uint32_t kLate     = 1;
    static constexpr uint32_t kEarly    = kSymbolDuration - 2;
    static constexpr uint32_t kEarliest = kSymbolDuration - 1;

    template <class bay>
    float SumOnTime(float sum, bay& history)
    {
        return sum - history[0][kLatest] - history[0][kEarliest];
    }

    template <class bay>
    float SumEarly(float sum, bay& history)
    {
        return sum - history[0][kLate] - history[0][kLatest];
    }

    template <class bay>
    float SumLate(float sum, bay& history)
    {
        return sum - history[0][kEarly] - history[0][kEarliest];
    }

    uint8_t DecideSymbol(bool adjustTiming)
    {
        float q_sum = q_history_[0].Sum();
        float i_sum = i_history_[0].Sum();

        if (adjustTiming)
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

            float threshold = 1.25 * on_time_strength;

            if (late_strength > threshold)
            {
                q_sum = q_sum_late;
                i_sum = i_sum_late;
            }
            else if (early_strength > threshold)
            {
                q_sum = q_sum_early;
                i_sum = i_sum_early;
            }

            early_ = (early_strength > threshold);
            late_ = (late_strength > threshold);
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
