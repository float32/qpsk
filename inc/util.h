#ifndef QPSK_UTIL_H_
#define QPSK_UTIL_H_

#ifndef __arm__
#include <cmath>
#endif

namespace qpsk
{

inline float Abs(float x)
{
#ifdef __arm__
    float out;
    asm("vabs.f32 %0, %1" : "=t" (out) : "t" (x));
    return out;
#else
    return std::fabs(x);
#endif
}

template <typename T>
inline T Clamp(T x, T min, T max)
{
    return (x < min) ? min : (x > max) ? max : x;
}

inline float Truncate(float x)
{
#ifdef __arm__
    float y;
    // The vcvt instruction rounds toward zero
    asm("vcvt.s32.f32 %0, %1" : "=t" (y) : "t" (x));
    asm("vcvt.f32.s32 %0, %0" : "+t" (y));
    return y;
#else
    return std::truncf(x);
#endif
}

inline float FractionalPart(float x)
{
    return x - Truncate(x);
}

}

#endif
