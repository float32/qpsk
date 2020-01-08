#ifndef QPSK_DEMODULATOR_H_
#define QPSK_DEMODULATOR_H_

#include <cstdint>
#include "inc/carrier_rejection_filter.h"
#include "inc/correlator.h"
#include "inc/fifo.h"
#include "inc/one_pole.h"
#include "inc/pll.h"
#include "inc/util.h"
#include "inc/window.h"

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
    }

    uint32_t SymbolsAvailable(void)
    {
        return symbols_.Available();
    }

    uint32_t SymbolsFull(void)
    {
        return symbols_.Full();
    }

    uint8_t PopSymbol(void)
    {
        return symbols_.Pop();
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
                agc_gain_ = 0.71f / level;
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

    float PllPhase(void)
    {
        return pll_.Phase();
    }

    float PllPhaseIncrement(void)
    {
        return pll_.PhaseIncrement();
    }

    float DecisionPhase(void)
    {
        return decision_phase_;
    }

    float SignalPower(void)
    {
        return follower_.Output();
    }

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
        }
        else
        {
            q_sum = SumOnTime(q_sum, q_history_);
            i_sum = SumOnTime(i_sum, i_history_);
        }

        return (i_sum < 0 ? 0 : 2) + (q_sum < 0 ? 0 : 1);
    }

    /* [[[cog

    import math

    def FloatTable(table):
        for i in range(0, len(table), 4):
            line = ''
            for j in range(i, min(len(table), i + 4)):
                line += '{0: .8e},'.format(table[j]).ljust(17)
            yield line.rstrip(' ')

    length = 65
    sine = [math.sin(math.pi / 2 * x / (length - 1)) for x in range(length)]

    cog.outl('static constexpr float kSineQuadrant[{0}] ='.format(length))
    cog.outl('{')
    for line in FloatTable(sine):
        cog.outl('    ' + line)
    cog.outl('};')

    ]]] */
    static constexpr float kSineQuadrant[65] =
    {
         0.00000000e+00,  2.45412285e-02,  4.90676743e-02,  7.35645636e-02,
         9.80171403e-02,  1.22410675e-01,  1.46730474e-01,  1.70961889e-01,
         1.95090322e-01,  2.19101240e-01,  2.42980180e-01,  2.66712757e-01,
         2.90284677e-01,  3.13681740e-01,  3.36889853e-01,  3.59895037e-01,
         3.82683432e-01,  4.05241314e-01,  4.27555093e-01,  4.49611330e-01,
         4.71396737e-01,  4.92898192e-01,  5.14102744e-01,  5.34997620e-01,
         5.55570233e-01,  5.75808191e-01,  5.95699304e-01,  6.15231591e-01,
         6.34393284e-01,  6.53172843e-01,  6.71558955e-01,  6.89540545e-01,
         7.07106781e-01,  7.24247083e-01,  7.40951125e-01,  7.57208847e-01,
         7.73010453e-01,  7.88346428e-01,  8.03207531e-01,  8.17584813e-01,
         8.31469612e-01,  8.44853565e-01,  8.57728610e-01,  8.70086991e-01,
         8.81921264e-01,  8.93224301e-01,  9.03989293e-01,  9.14209756e-01,
         9.23879533e-01,  9.32992799e-01,  9.41544065e-01,  9.49528181e-01,
         9.56940336e-01,  9.63776066e-01,  9.70031253e-01,  9.75702130e-01,
         9.80785280e-01,  9.85277642e-01,  9.89176510e-01,  9.92479535e-01,
         9.95184727e-01,  9.97290457e-01,  9.98795456e-01,  9.99698819e-01,
         1.00000000e+00,
    };
    // [[[end]]]

    float Sine(float t)
    {
        uint32_t index = 256.f * t;
        uint32_t quadrant = (index & 0xC0) >> 6;
        index &= 0x3F;

        if (quadrant & 1)
        {
            index = 0x40 - index;
        }

        return (quadrant & 2) ? -kSineQuadrant[index] : kSineQuadrant[index];
    }

    float Cosine(float t)
    {
        return Sine(t + 0.25f);
    }
};

}

#endif
