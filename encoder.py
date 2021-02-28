#!/usr/bin/env python3
#
# MIT License
#
# Copyright 2013 Emilie Gillet.
# Copyright 2021 Tyler Coy
#
# Author: Emilie Gillet (emilie.o.gillet@gmail.com)
# Author: Tyler Coy (alrightdevices.com)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
# See http://creativecommons.org/licenses/MIT/ for more information.
#
# -----------------------------------------------------------------------------
#
# QPSK encoder for converting firmware files into wav files

import argparse
import zlib
import wave
import os.path
import struct
import array
import math
import sys
import string
import io
import itertools



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--sample-rate', dest='sample_rate',
        type=int,
        required=True,
        help='Sample rate in Hz. Must be a multiple of the symbol rate.')
    parser.add_argument('-y', '--symbol-rate', dest='symbol_rate',
        type=int,
        required=True,
        help='Symbol rate in Hz.')
    parser.add_argument('-b', '--block-size', dest='block_size',
        required=True,
        help='The number of bytes that the target will write at a time. '
            'Must be a multiple of the packet size.')
    parser.add_argument('-w', '--write-time', dest='write_time',
        required=True,
        help='Amount of time in milliseconds required by the target to write '
            'a block.')
    parser.add_argument('-f', '--flash-spec', dest='flash_spec',
        required=True,
        nargs='+',
        help=
            'Flash page specification which describes the layout of the '
            'target\'s flash memory. This is a list of '
            'specifiers of the form "SIZE:ETIME[:NUM]", where SIZE is the '
            'size in bytes of the page (and must be a multiple of the block '
            'size), ETIME is the amount of time in milliseconds to allow for '
            'the page erase to complete, and NUM optionally specifies the '
            'number of these pages (if NUM is not given, then this specifier '
            'will be used for all remaining pages). For example, a simple '
            'layout consisting of 1KB pages that require 50ms each to erase '
            'could be specified as "1024:50", while a more complex layout '
            'like that of the STM32F405 could be specified as '
            '"16K:500:4 64K:1100:1 128K:2000:7".')
    parser.add_argument('-a', '--start-address', dest='start_address',
        required=True,
        help='Flash memory address at which the firmware will be written. '
            'If a "+" is prepended, this will be interpreted as an offset from '
            'the base address, otherwise as an absolute address. Must align '
            'to a page.')
    parser.add_argument('-x', '--base-address', dest='base_address',
        default='0',
        help='Base address of the target\'s flash memory. Default 0.')
    parser.add_argument('-p', '--packet-size', dest='packet_size',
        default='256',
        help='Packet size in bytes. Must be a multiple of 4. Default 256.')
    parser.add_argument('--fill', dest='fill_byte',
        default='0xFF',
        help='Byte value to use to fill gaps and pad lengths. Default 0xFF.')
    parser.add_argument('-e', '--seed', dest='crc_seed',
        default='0',
        help='CRC32 seed. Default 0.')
    parser.add_argument('-t', '--file-type', dest='file_type',
        choices=['hex', 'bin', 'auto'], default='auto',
        help='Input file type. If a hex file is used, all '
            'content before the start address will be ignored. A bin file is '
            'always assumed to start at the start address. Default auto.')
    parser.add_argument('-i', '--input-file', dest='input_file',
        default='-',
        help='Input file (bin or hex). Default stdin.')
    parser.add_argument('-o', '--output-file', dest='output_file',
        default=None,
        help='Output wav file. The default is derived from the input file name '
            'if one is given, otherwise stdout.')
    args = parser.parse_args()

    if args.input_file == '-':
        input_file = sys.stdin.buffer
        if args.output_file == None:
            args.output_file = '-'
    else:
        input_file = open(args.input_file, 'rb')
        (root, ext) = os.path.splitext(args.input_file)
        if args.file_type == 'auto' and ext in ['.bin', '.hex']:
            args.file_type = ext[1:]
        if args.output_file == None:
            args.output_file = root + '.wav'

    data = input_file.read()
    if input_file is not sys.stdin.buffer:
        input_file.close()

    if args.file_type == 'auto':
        is_hex = all(map(lambda x: chr(x) in string.hexdigits + ':\r\n', data))
        args.file_type = 'hex' if is_hex else 'bin'

    base_address = int(args.base_address, 0)
    if args.start_address.startswith('+'):
        start_address = base_address + int(args.start_address[1:], 0)
    else:
        start_address = int(args.start_address, 0)

    fill_byte = int(args.fill_byte, 0)

    if args.file_type == 'hex':
        from intelhex import IntelHex
        ihex = IntelHex(io.StringIO(data.decode('ascii')))[start_address:]
        ihex.padding = fill_byte
        data = ihex.tobinstr()

    if args.output_file == '-':
        output_file = sys.stdout.buffer
    else:
        output_file = args.output_file

    arrangement = Arrangement(
            flash_spec    = args.flash_spec,
            reserved_size = start_address - base_address,
            block_size    = parse_size(args.block_size),
            fill_byte     = fill_byte,
            write_time    = float(args.write_time),
            data          = data)

    encoder = Encoder(
            symbol_rate = args.symbol_rate,
            packet_size = parse_size(args.packet_size),
            crc_seed    = int(args.crc_seed, 0))

    symbols = encoder.encode(arrangement)

    assert (args.sample_rate % args.symbol_rate) == 0
    modulator = QPSKModulator(args.sample_rate, args.symbol_rate)
    signal = modulator.modulate(symbols)

    writer = wave.open(output_file, 'wb')
    writer.setframerate(args.sample_rate)
    writer.setsampwidth(2)
    writer.setnchannels(1)
    writer.writeframes(signal.tobytes())
    writer.close()



class Arrangement:
    def __init__(self, flash_spec, reserved_size,
                block_size, fill_byte, write_time, data):
        self._blocks = []
        for (erase_time, page_data) in Pages(data, flash_spec, reserved_size):
            blocks = Blocks(page_data, block_size, fill_byte)
            for i, block_data in enumerate(blocks):
                wait_time = write_time;
                if i == 0:
                    wait_time += erase_time
                self._blocks.append((block_data, wait_time / 1000))

    def __iter__(self):
        return iter(self._blocks)



class Encoder:

    def __init__(self, symbol_rate, packet_size, crc_seed):
        assert (packet_size % 4) == 0

        self._symbol_rate = symbol_rate
        self._packet_size = packet_size
        self._crc_seed = crc_seed

        self._alignment_sequence = b'\x99' * 4
        self._block_marker = b'\xCC\xCC\xCC\xCC'
        self._end_marker = b'\xF0\xF0\xF0\xF0'

        self._byte_table = []
        for byte in range(256):
            self._byte_table.append([
                ((byte >> 6) & 3),
                ((byte >> 4) & 3),
                ((byte >> 2) & 3),
                ((byte >> 0) & 3)])

        self._hamming_table = []
        bit_num = 1
        for i in range((packet_size + 4) * 8):
            while ((bit_num & (bit_num - 1)) == 0):
                bit_num += 1
            self._hamming_table.append((bit_num, i // 8, 1 << (i % 8)))
            bit_num += 1

    def _encode_blank(self, duration):
        return [0] * int(duration * self._symbol_rate)

    def _encode_intro(self):
        return self._encode_blank(1.0)

    def _encode_resync(self):
        # We let the PLL sync to a string of zeros, then append a single 3
        # to mark the end of sync and start of alignment.
        return self._encode_blank(0.0375) + [3]

    def _encode_outro(self):
        symbols = self._encode_resync()
        for byte in self._alignment_sequence + self._end_marker:
            symbols += self._encode_byte(byte)
        return symbols

    def _encode_byte(self, byte):
        return self._byte_table[byte]

    def _hamming(self, data):
        parity = 0
        for (bit_num, byte_num, mask) in self._hamming_table:
            if data[byte_num] & mask:
                parity ^= bit_num
        return parity

    def _encode_packet(self, data):
        assert len(data) == self._packet_size
        crc = zlib.crc32(data, self._crc_seed) & 0xFFFFFFFF
        data += struct.pack('<L', crc)
        symbols = []
        for byte in data + struct.pack('<H', self._hamming(data)):
            symbols += self._encode_byte(byte)
        return symbols

    def _encode_block(self, data):
        assert (len(data) % self._packet_size) == 0

        symbols = self._encode_resync()

        for byte in self._alignment_sequence + self._block_marker:
            symbols += self._encode_byte(byte)

        for i in range(0, len(data), self._packet_size):
            packet = data[i : i + self._packet_size]
            symbols += self._encode_packet(packet)

        return symbols

    def encode(self, blocks):
        symbols = []
        symbols += self._encode_intro()

        for (data, time) in blocks:
            symbols += self._encode_block(data)
            symbols += self._encode_blank(time)

        symbols += self._encode_outro()
        return symbols



class QPSKModulator:

    def __init__(self, sample_rate, symbol_rate):
        assert (sample_rate % symbol_rate) == 0
        symbol_duration = sample_rate // symbol_rate
        self._sample_rate = sample_rate
        self._symbol_table = self._construct_symbols(symbol_duration)

    def _construct_symbols(self, symbol_duration):
        lookup = list()
        for symbol in range(4):
            msb = (symbol & 2) - 1
            lsb = (symbol & 1) * 2 - 1
            samples = list()
            for i in range(symbol_duration):
                phase = 2 * math.pi * i / symbol_duration
                sample = (msb * math.cos(phase) - lsb * math.sin(phase))
                sample /= math.sqrt(2)
                assert (sample >= -1) and (sample <= 1)
                samples.append(int(32767 * sample))
            lookup.append(array.array('h', samples))
        return lookup

    def modulate(self, symbols):
        signal = array.array('h')
        silence = [0] * (self._sample_rate // 10)
        signal.extend(silence)
        for symbol in symbols:
            signal.extend(self._symbol_table[symbol])
        signal.extend(silence)
        return signal



def parse_size(size):
    if size.upper().endswith('K'):
        return int(size[:-1], 0) * 1024
    else:
        return int(size, 0)

class Pages:
    def __init__(self, data, flash_spec, reserved_size):
        page_spec = self._page_specs(flash_spec)

        while reserved_size > 0:
            (page_size, erase_time) = next(page_spec)
            reserved_size -= page_size
        assert reserved_size == 0

        self._pages = []
        while len(data) > 0:
            (page_size, erase_time) = next(page_spec)
            self._pages.append((erase_time, data[:page_size]))
            data = data[page_size:]

    def __iter__(self):
        return iter(self._pages)

    def _page_specs(self, flash_spec):
        for (size, time, num) in map(self._parse_page_spec, flash_spec):
            if num == None:
                yield from itertools.repeat((size, time))
            else:
                yield from itertools.repeat((size, time), num)

    def _parse_page_spec(self, page_spec):
        spec = page_spec.split(':')
        try:
            size, time, num = spec
            num = int(num, 0)
        except ValueError:
            size, time = spec
            num = None

        size = parse_size(size)
        time = float(time)
        return (size, time, num)

class Blocks:
    def __init__(self, data, block_size, fill_byte):
        if len(data) % block_size:
            padding = block_size - (len(data) % block_size)
            data += bytes([fill_byte]) * padding
        assert len(data) % block_size == 0
        self._blocks = [data[i : i + block_size]
            for i in range(0, len(data), block_size)]

    def __iter__(self):
        return iter(self._blocks)



if __name__ == '__main__':
    main()
