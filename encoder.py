#!/usr/bin/env python3
#
# MIT License
#
# Copyright 2013 Emilie Gillet.
# Copyright 2020 Tyler Coy
#
# Author: Emilie Gillet (emilie.o.gillet@gmail.com)
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
from intelhex import IntelHex
import os.path
import struct
import array
import math



class QPSKEncoder():

    def __init__(self, sample_rate, symbol_rate,
                page_size, packet_size, crc_seed, page_write_time):
        assert (sample_rate % symbol_rate) == 0
        assert (page_size % packet_size) == 0

        self._sample_rate = sample_rate
        self._symbol_rate = symbol_rate
        self._page_size = page_size
        self._packet_size = packet_size
        self._crc_seed = crc_seed

        self._packet_preamble = b'\x00' * 8 + b'\x99' * 4 + b'\xCC' * 4
        self._samples_per_symbol = self._sample_rate // self._symbol_rate
        self._packets_per_page = self._page_size // self._packet_size
        self._page_write_time = page_write_time

        self._signal = array.array('h')
        self._symbol_lookup = self._generate_symbols()

    def _generate_symbols(self):
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
        self._signal.extend([0] * self._sample_rate)
        self._encode_blank(1.0)

    def _encode_outro(self):
        self._encode_blank(1.0)

    def _encode_byte(self, byte):
        self._encode_symbol((byte >> 6) & 3);
        self._encode_symbol((byte >> 4) & 3);
        self._encode_symbol((byte >> 2) & 3);
        self._encode_symbol((byte >> 0) & 3);

    def _encode_packet(self, data):
        crc = zlib.crc32(data, self._crc_seed) & 0xFFFFFFFF
        for byte in self._packet_preamble + data + struct.pack('>L', crc):
            self._encode_byte(byte)

    def encode(self, data):
        assert (len(data) % self._page_size) == 0

        self._encode_intro()

        num_pages = len(data) // self._page_size
        for page in range(num_pages):
            for packet in range(self._packets_per_page):
                offset = page * self._page_size + packet * self._packet_size
                self._encode_packet(data[offset:(offset + self._packet_size)])
            self._encode_blank(self._page_write_time)

        self._encode_outro()

    def tobytes(self):
        return self._signal.tobytes()



def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-s', '--sample-rate', dest='sample_rate',
        type=int,
        required=True,
        help='Sample rate in Hz (must be a multiple of the carrier frequency')
    parser.add_argument('-c', '--carrier-frequency', dest='carrier_frequency',
        type=int,
        required=True,
        help='Carrier frequency in Hz')
    parser.add_argument('-p', '--packet-size', dest='packet_size',
        type=int,
        default=256,
        help='Packet size in bytes, default 256')
    parser.add_argument('-f', '--page-size', dest='page_size',
        type=int,
        required=True,
        help='Flash page size in bytes (must be a multiple of the packet size')
    parser.add_argument('-w', '--page-write-time', dest='page_write_time',
        required=True,
        help='Flash page write time in seconds')
    parser.add_argument('-x', '--offset', dest='file_offset',
        default='0',
        help='Ignore hex file contents before this location')
    parser.add_argument('-b', dest='file_base',
        default='0',
        help='Offset is relative to this address')
    parser.add_argument('-e', '--seed', dest='crc_seed',
        default='0',
        help='CRC32 seed (default 0)')
    parser.add_argument('-t', '--file-type', dest='file_type',
            choices=['hex', 'bin', 'auto'], default='auto',
            help='Input file type (default auto)')
    parser.add_argument('--fill', dest='fill_byte',
        default='0xFF',
        help='Fill/pad byte (default 0xFF)')
    parser.add_argument('-o', '--output-file', dest='output_file',
        default=None,
        help='Output wav file (default derived from input file name)',
        metavar='FILE')
    parser.add_argument('-i', '--input-file', dest='input_file',
        required=True,
        help='Input file (bin or hex)')
    args = parser.parse_args()

    fill_byte = int(args.fill_byte, 0)

    (root, ext) = os.path.splitext(args.input_file)
    if args.file_type == 'auto':
        assert ext in ['.bin', '.hex']
        args.file_type = ext[1:]

    if args.file_type == 'bin':
        data = open(args.input_file, 'rb').read()
    elif args.file_type == 'hex':
        offset = int(args.file_offset, 0)
        base = int(args.file_base, 0)
        ihex = IntelHex(args.input_file)[base+offset:]
        ihex.padding = fill_byte
        data = ihex.tobinstr()

    output_file = args.output_file
    if not output_file:
        output_file = root + '.wav'

    encoder = QPSKEncoder(
            sample_rate = args.sample_rate,
            symbol_rate = args.carrier_frequency,
            page_size   = args.page_size,
            packet_size = args.packet_size,
            crc_seed    = int(args.crc_seed, 0),
            page_write_time = float(args.page_write_time))

    fill_byte = struct.pack('B', int(args.fill_byte, 0))
    while len(data) % args.page_size:
        data += fill_byte

    encoder.encode(data)

    writer = wave.open(output_file, 'wb')
    writer.setframerate(args.sample_rate)
    writer.setsampwidth(2)
    writer.setnchannels(1)
    writer.writeframes(encoder.tobytes())
    writer.close()

if __name__ == '__main__':
    main()
