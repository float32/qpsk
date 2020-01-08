#ifndef QPSK_ONE_POLE_H_
#define QPSK_ONE_POLE_H_

#include <cstdint>
#include <cmath>

namespace qpsk
{

class OnePoleLowpass
{
protected:
    static constexpr float Factor(float freq)
    {
        return 1 - std::exp(-2 * M_PI * freq);
    }

    float factor_;
    float history_;

public:
    void Init(float normalized_frequency)
    {
        factor_ = Factor(normalized_frequency);
        history_ = 0;
    }

    void Reset(void)
    {
        history_ = 0;
    }

    float Process(float in)
    {
        history_ += factor_ * (in - history_);
        return history_;
    }

    float Output(void)
    {
        return history_;
    }
};

class OnePoleHighpass : public OnePoleLowpass
{
protected:
    using super = OnePoleLowpass;

public:
    float Process(float in)
    {
        return in - super::Process(in);
    }
};

}

#endif
