#ifndef QPSK_PLL_H_
#define QPSK_PLL_H_

#include <cstdint>
#include "inc/one_pole.h"
#include "inc/util.h"

namespace qpsk
{

class PhaseLockedLoop
{
protected:
    float nominal_frequency_;
    float phase_increment_;
    float phase_;
    float phase_error_;
    OnePoleLowpass lpf_;

public:
    void Init(float normalized_frequency)
    {
        nominal_frequency_ = normalized_frequency;
        Reset();
        lpf_.Init(normalized_frequency / 32.f);
    }

    void Reset(void)
    {
        phase_increment_ = nominal_frequency_;
        phase_ = 0.f;
        phase_error_ = 0.f;
    }

    void Sync(void)
    {
        phase_ = 0.f;
        phase_error_ = 0.f;
    }

    float Phase(void)
    {
        return phase_;
    }

    float PhaseIncrement(void)
    {
        return phase_increment_;
    }

    float Process(float error)
    {
        phase_error_ = lpf_.Process(error);
        phase_increment_ -= phase_error_ / 4096.f;
        phase_increment_ = Clamp(phase_increment_, 0.f, 1.f);

        phase_ += phase_increment_ - phase_error_ / 16.f;
        phase_ = FractionalPart(phase_);

        return phase_;
    }
};

}

#endif
