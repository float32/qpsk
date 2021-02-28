# qpsk

A C++ QPSK decoder for embedded systems based on the excellent
[stm-audio-bootloader](https://github.com/pichenettes/stm-audio-bootloader)
by Émilie Gillet.

Its intended use case is to decode firmware from an audio signal
as part of an audio device's bootloader.

A Python script is provided for encoding firmware into a wav file.

---

## Features

- **Robust**:
    Automatic gain control and basic error correction reduce
    decoding errors.
- **High performance, small footprint**:
    Tested on a Cortex-M4F down to 32MHz with a 16Kbaud QPSK signal.
    Can squeeze into as little as 12KB.
- **Portable**: No assumptions made about underlying hardware. No dependencies
    outside of the C++ standard library.
- **Safe**: Static memory allocation, no constructors.
- **Header only** for your convenience.
- **MIT license**


## Limitations

- Uses `float` extensively, so will run poorly on anything without a
  hardware floating-point unit.
- Requires a c11/c++17 compiler.


## Usage

### Encoder

The encoder is a Python 3 script. View usage information by running
the script with the `-h` or `--help` flag.

Here's how we might encode our firmware file:

```sh
python3 encoder.py \
    --sample-rate 48000 \
    --symbol-rate 8000 \
    --packet-size 256 \
    --block-size 2048 \
    --write-time 50 \
    --flash-spec 2K:100 \
    --base-address 0x08000000 \
    --start-address +0x4000 \
    --seed 0x420ACAB \
    --file-type hex \
    --input-file firmware.hex \
    --output-file firmware.wav
```

### Decoder

First, we add the library to our C++ source with a single include directive:

```C++
#include "qpsk/decoder.h"
```

#### Instantiation

The decoder is provided as a C++ class template:

```C++
template <uint32_t sample_rate,
          uint32_t symbol_rate,
          uint32_t packet_size,
          uint32_t block_size,
          uint32_t fifo_capacity = 256>
class Decoder
{
    // ...
};
```

`sample_rate` and `symbol_rate` are measured in Hz and should approximate
the rates of the incoming signal (they need not match the rates of the
encoded audio file). `sample_rate` must be 6, 8, 12, or 16 times
the `symbol_rate`.

`packet_size` and `block_size` are measured in bytes and must match the
values that were passed to the encoder. `block_size` must be a multiple
of `packet_size`, and `packet_size` must be a multiple of 4.

The optional parameter `fifo_capacity` determines the size in samples of
the decoder's internal statically-allocated input FIFO. Larger sizes are
more robust against overflow, but the default is usually plenty.

Here's how we might instantiate our `Decoder` object:

```C++
qpsk::Decoder<48000, 8000, 256, 2048> decoder;
```

#### Initialization

We must initialize the decoder before using it by calling its `Init`
function, which has this signature:

```C++
void Init(uint32_t crc_seed);
```

`crc_seed` is a 32-bit value that the decoder uses during packet error
checking. It must match the value that was passed to the encoder. This is
useful as a firmware compatibility check. E.g. if we use a unique seed for
each of our products, then none will be able to accidentally install the
others' firmware.

Here's how we might initialize our `Decoder` object:

```C++
decoder.Init(0x420ACAB);
```

#### Processing

A simple way to implement our bootloader is to set up a periodic timer
interrupt with frequency `sample_rate` in which the audio waveform is
sampled using an ADC. Convert to `float` and normalize the sample, then
pass it to the decoder. The polarity of the sampled signal doesn't matter,
nor does the amplitude as long as it's above a very small threshold.

Here's how we might set up our interrupt, assuming our ADC generates
12-bit unsigned samples:

```C++
void TimerInterrupt(void)
{
    int16_t data = ADCRead();
    float sample = (data - 0x800) / 2048.f;
    decoder.Push(sample);
}
```

In a lower-priority thread of execution (such as our `main` function) and
within a loop, we call the decoder's `Process` function. It returns a `Result`,
the value of which we use to decide what to do next.

- **`RESULT_NONE`**: No action needed.
- **`RESULT_PACKET_COMPLETE`**: Successfully decoded a packet.
    No action needed, but we can use this opportunity to update our
    device's UI to indicate ongoing processing.
- **`RESULT_BLOCK_COMPLETE`**: Successfully decoded a block. Also implies
    that a packet was completed (the last packet in the block).
    Our bootloader should call the decoder's `block_data` function to get
    a pointer to the data and then write it to program memory.
- **`RESULT_END`**: Successfully finished decoding the
    input signal. No action needed, but we might indicate success via our
    UI and/or perform a software reset.
- **`RESULT_ERROR`**: Encountered an error during decoding. We must call
    the decoder's `Reset` function before reattempting decoding, perhaps
    after waiting for our user to press a 'retry' button.

Here's how we might set up our processing loop:

```C++
for (;;)
{
    qpsk::Result result = decoder.Process();

    if (result == qpsk::RESULT_PACKET_COMPLETE)
    {
        IndicatePacketComplete();
    }
    else if (result == qpsk::RESULT_BLOCK_COMPLETE)
    {
        IndicatePacketComplete();
        IndicateBlockComplete();
        const uint32_t* data = decoder.block_data();
        WriteBlock(data);
    }
    else if (result == qpsk::RESULT_END)
    {
        IndicateSuccess();
        PerformSoftwareReset();
    }
    else if (result == qpsk::RESULT_ERROR)
    {
        WaitForRetryButtonPress();
        decoder.Reset();
    }
}
```


## Possible improvements

### Compression

Input data could be compressed using a simple scheme like LZ4 or
[heatshrink](https://github.com/atomicobject/heatshrink).
The author's tests with these have shown a possible space saving of 20-30%.

### Error correction

Hamming error correction allows us to correct a single flipped bit per
packet. We could use a more robust scheme like Reed-Solomon to
improve the tolerated error density, but (in the case of RS) at the cost
of a lot of program memory and some CPU overhead.

### Encoding

This library could be built upon to implement a denser encoding scheme
like 8-PSK.


## Example implementation

Bootloader implementation examples, as well as unit tests and a decoder
simulation can be found at:

https://github.com/float32/qpsk-examples


---

Copyright 2021 Tyler Coy

https://www.alrightdevices.com

https://github.com/float32
