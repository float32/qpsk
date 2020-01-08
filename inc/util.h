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

cog.outl('inline constexpr float kSineQuadrant[{0}] ='.format(length))
cog.outl('{')
for line in FloatTable(sine):
    cog.outl('    ' + line)
cog.outl('};')

]]] */
inline constexpr float kSineQuadrant[65] =
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

inline float Sine(float t)
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

inline float Cosine(float t)
{
    return Sine(t + 0.25f);
}

}

#endif
