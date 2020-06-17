#!/usr/bin/env python3
#
# MIT License
#
# Copyright 2013 Emilie Gillet.
# Copyright 2020 Tyler Coy
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



class QPSKEncoder():

    def __init__(self, sample_rate, symbol_rate,
                packet_size, block_size, write_time,
                flash_spec, reserved_size,
                crc_seed, fill_byte):
        assert (sample_rate % symbol_rate) == 0
        assert (packet_size % 4) == 0
        assert (block_size % packet_size) == 0

        self._sample_rate = sample_rate
        self._symbol_rate = symbol_rate
        self._packet_size = packet_size
        self._block_size = block_size
        self._write_time = write_time
        self._flash_spec = flash_spec
        self._reserved_size = reserved_size
        self._crc_seed = crc_seed
        self._fill_byte = struct.pack('B', fill_byte)

        self._alignment_sequence = b'\x99' * 4
        self._block_marker = b'\xCC\xCC\xCC\xCC'
        self._end_marker = b'\xF0\xF0\xF0\xF0'
        self._samples_per_symbol = self._sample_rate // self._symbol_rate

        self._signal = array.array('h')
        self._symbol_lookup = self._construct_symbols()

    def _construct_symbols(self):
        lookup = list()
        for symbol in range(4):
            msb = (symbol & 2) - 1
            lsb = (symbol & 1) * 2 - 1
            samples = list()
            for i in range(self._samples_per_symbol):
                phase = 2 * math.pi * i / self._samples_per_symbol
                sample = (msb * math.cos(phase) - lsb * math.sin(phase))
                sample /= math.sqrt(2)
                assert (sample >= -1) and (sample <= 1)
                samples.append(int(32767 * sample))
            lookup.append(array.array('h', samples))
        return lookup

    def _encode_symbol(self, symbol):
        self._signal.extend(self._symbol_lookup[symbol])

    def _encode_blank(self, duration):
        length = int(duration * self._symbol_rate)
        self._signal.extend(self._symbol_lookup[0] * length)

    def _encode_intro(self):
        self._signal.extend([0] * (self._sample_rate // 10))
        self._encode_blank(1.0)

    def _encode_outro(self):
        for byte in self._alignment_sequence + self._end_marker:
            self._encode_byte(byte)
        self._signal.extend([0] * (self._sample_rate // 10))

    def _encode_byte(self, byte):
        self._encode_symbol((byte >> 6) & 3);
        self._encode_symbol((byte >> 4) & 3);
        self._encode_symbol((byte >> 2) & 3);
        self._encode_symbol((byte >> 0) & 3);

    def _hamming(self, data):
        parity = 0
        bit_num = 1
        for i in range(len(data) * 8):
            while ((bit_num & (bit_num - 1)) == 0):
                bit_num += 1
            bit = (data[i // 8] >> (i % 8)) & 1
            if bit:
                parity ^= bit_num
            bit_num += 1
        return parity

    def _encode_packet(self, data):
        assert len(data) == self._packet_size
        crc = zlib.crc32(data, self._crc_seed) & 0xFFFFFFFF
        data += struct.pack('<L', crc)
        for byte in data + struct.pack('<H', self._hamming(data)):
            self._encode_byte(byte)

    def _encode_block(self, data):
        assert len(data) == self._block_size
        for byte in self._alignment_sequence + self._block_marker:
            self._encode_byte(byte)

        for i in range(0, self._block_size, self._packet_size):
            packet = data[i : i + self._packet_size]
            self._encode_packet(packet)


    def _page_spec(self, flash_spec):
        for (size, time, num) in flash_spec:
            if num == None:
                yield from itertools.repeat((size, time))
            else:
                yield from itertools.repeat((size, time), num)

    def encode(self, data):
        page = self._page_spec(self._flash_spec)

        while self._reserved_size > 0:
            (page_size, erase_time) = page.__next__()
            self._reserved_size -= page_size
        assert self._reserved_size == 0

        while len(data) % self._block_size:
            data += self._fill_byte

        self._encode_intro()

        while len(data) > 0:
            (page_size, erase_time) = page.__next__()
            remaining = min(page_size, len(data))
            assert (remaining % self._block_size) == 0

            for i in range(remaining // self._block_size):
                block = data[:self._block_size]
                data = data[self._block_size:]
                self._encode_block(block)
                if i == 0:
                    self._encode_blank(erase_time)
                self._encode_blank(self._write_time)

        self._encode_outro()

    def tobytes(self):
        return self._signal.tobytes()



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

    fill_byte = int(args.fill_byte, 0)

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

    if args.file_type == 'hex':
        from intelhex import IntelHex
        ihex = IntelHex(io.StringIO(data.decode('ascii')))[start_address:]
        ihex.padding = fill_byte
        data = ihex.tobinstr()

    if args.output_file == '-':
        output_file = sys.stdout.buffer
    else:
        output_file = args.output_file

    encoder = QPSKEncoder(
            sample_rate = args.sample_rate,
            symbol_rate = args.symbol_rate,
            crc_seed    = int(args.crc_seed, 0),
            packet_size = parse_size(args.packet_size),
            block_size  = parse_size(args.block_size),
            write_time  = float(args.write_time) / 1000,
            flash_spec  = parse_flash_spec(args.flash_spec),
            reserved_size = start_address - base_address,
            fill_byte   = fill_byte)

    encoder.encode(data)

    writer = wave.open(output_file, 'wb')
    writer.setframerate(args.sample_rate)
    writer.setsampwidth(2)
    writer.setnchannels(1)
    writer.writeframes(encoder.tobytes())
    writer.close()

def parse_size(size):
    if size.upper().endswith('K'):
        return int(size[:-1], 0) * 1024
    else:
        return int(size, 0)

def parse_flash_spec(spec_list):
    parsed_list = []
    for spec in spec_list:
        spec = spec.split(':')
        try:
            size, time, num = spec
            num = int(num, 0)
        except ValueError:
            size, time = spec
            num = None

        size = parse_size(size)
        time = float(time) / 1000

        parsed_list.append((size, time, num))
    return parsed_list

if __name__ == '__main__':
    main()
