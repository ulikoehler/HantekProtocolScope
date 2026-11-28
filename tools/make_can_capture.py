#!/usr/bin/env python3
"""Fabricate a synthetic UliCAN raw capture containing Classic CAN frames.

The output is directly replayable with `ulican view FILE.bin`: a 64-byte
RawFileHeader followed by interleaved uint8 ADC-code pairs (CANH, CANL).

Frames are full CAN 2.0A data frames on the wire: SOF, 11-bit identifier,
RTR/IDE/r0, DLC, payload, CRC-15, bit stuffing, ACK slot and 7-bit EOF.
CANH/CANL are rendered with realistic recessive/dominant levels, a short
RC edge, independent channel noise, and 8-bit ADC quantization.
"""

import argparse
import math
import random
import struct

MAGIC = b"ULICAN01"
VERSION = 1
CHANNELS = 2

# Hantek DSO-6022BL-like calibration. v = voltsPerCode * (adc - offsetCodes)
VOLTS_PER_CODE = (0.05, 0.05)
OFFSET_CODES = (128.0, 128.0)

RECESSIVE_V = 2.5
DOMINANT_HIGH_V = 3.5
DOMINANT_LOW_V = 1.5


def int_bits(value, width):
    return [(value >> (width - 1 - i)) & 1 for i in range(width)]


def crc15(bits):
    """CAN CRC-15, polynomial 0x4599, MSB first."""
    checksum = 0
    for bit in bits:
        feedback = ((checksum >> 14) ^ bit) & 1
        checksum = (checksum << 1) & 0x7FFF
        if feedback:
            checksum ^= 0x4599
    return checksum


def bit_stuff(bits):
    """Insert one opposite bit after every five identical bits."""
    out = [bits[0]]
    last = bits[0]
    run = 1
    for bit in bits[1:]:
        out.append(bit)
        if bit == last:
            run += 1
            if run == 5:
                stuff = 1 - bit
                out.append(stuff)
                last = stuff
                run = 1
        else:
            last = bit
            run = 1
    return out


def wire_bits(identifier, payload, dlc):
    """Unstuffed SOF..CRC bits for a standard data frame."""
    bits = [0]  # SOF
    bits += int_bits(identifier & 0x7FF, 11)
    bits += [0, 0, 0]  # RTR (data), IDE (standard), r0
    bits += int_bits(dlc, 4)
    for byte in payload[:dlc]:
        bits += int_bits(byte, 8)
    bits += int_bits(crc15(bits), 15)
    return bits


def full_frame(identifier, payload, dlc):
    """Stuff SOF..CRC, then append CRC delimiter, ACK, delimiter, EOF."""
    stuffed = bit_stuff(wire_bits(identifier, payload, dlc))
    # CRC delimiter (recessive), ACK slot (dominant), ACK delimiter + 7 EOF
    # (recessive), then 3 intermission bits.
    return stuffed + [1, 0] + [1] * 8 + [1] * 3


def le16(value):
    value = int(value) & 0xFFFF
    return [value & 0xFF, value >> 8]


class Message:
    def __init__(self, identifier, dlc, period_s, payload_fn):
        self.identifier = identifier
        self.dlc = dlc
        self.period_s = period_s
        self.payload_fn = payload_fn
        self.due = 0.0
        self.sequence = 0

    def payload(self):
        data = self.payload_fn(self.sequence)
        self.sequence += 1
        return (data + [0] * 8)[:8]


# A small, plausible automotive-style broadcast schedule.
def engine_payload(seq):
    rpm = 800 + (seq * 137) % 5200
    throttle = min(100, (seq * 7) % 105)
    coolant = 80 + (seq // 50) % 25
    return le16(rpm * 4) + [throttle, coolant, 0x00, 0x00, 0x00, 0x00]


def wheel_payload(seq):
    base = 40 + (seq * 3) % 80
    return le16(base * 10) + le16((base + 1) * 10) + \
        le16((base - 1) * 10) + le16(base * 10)


def brake_payload(seq):
    pressure = 0 if (seq // 20) % 4 else (seq * 13) % 120
    angle = ((seq * 29) % 600) - 300
    return le16(pressure * 10) + le16(angle * 10) + [0x20, 0x00, 0x00, 0x00]


def battery_payload(seq):
    millivolt = 13500 + (seq * 17) % 900
    deciamp = 100 + (seq * 11) % 800
    return le16(millivolt) + le16(deciamp) + [0x00, 0x00, 0x00, 0x00]


def temperature_payload(seq):
    return [80 + (seq * 3) % 40, 25 + (seq * 2) % 20, 0x00, 0x00]


def gear_payload(seq):
    return [(seq // 25) % 6 + 1, (seq * 5) % 256]


def gps_payload(seq):
    speed = 300 + (seq * 7) % 500
    heading = (seq * 23) % 3600
    return le16(speed) + le16(heading) + [0x00, 0x00]


def door_payload(seq):
    return [0x01 if (seq // 30) % 3 == 0 else 0x00, 0x00, 0x00]


MESSAGES = [
    Message(0x0C0, 8, 0.010, wheel_payload),
    Message(0x1A0, 8, 0.010, engine_payload),
    Message(0x2F1, 4, 0.050, temperature_payload),
    Message(0x3B4, 2, 0.100, gear_payload),
    Message(0x4A0, 8, 0.020, brake_payload),
    Message(0x5C8, 8, 0.020, battery_payload),
    Message(0x6E0, 6, 0.050, gps_payload),
    Message(0x7F1, 3, 0.100, door_payload),
]


def build_schedule(frame_count, bitrate, rng):
    """Return [(start_bit, identifier, payload, dlc, crc)] in bus order.

    Times are kept in whole bit slots so the rendered waveform and the
    reported frame times agree exactly.
    """
    scheduled = []
    previous_end = 0
    while len(scheduled) < frame_count:
        msg = min(MESSAGES, key=lambda m: m.due)
        due_bits = int(math.ceil(msg.due * bitrate))
        # At least 6 idle bit times; the first frame additionally starts
        # after 20 bit times of leading bus idle.
        earliest = previous_end + (20 if not scheduled else 6)
        start_bit = max(due_bits, earliest)
        payload = msg.payload()
        crc = crc15(wire_bits(msg.identifier, payload, msg.dlc)[:-15])
        frame = full_frame(msg.identifier, payload, msg.dlc)
        scheduled.append((start_bit, msg.identifier, payload, msg.dlc, crc))
        previous_end = start_bit + len(frame)
        msg.due += msg.period_s
        if msg.due * bitrate <= start_bit:
            msg.due = (start_bit + 1) / bitrate
    return scheduled


def render(scheduled, bitrate, rate_hz, rng, noise_v, rise_tau_s):
    """Render scheduled frames into interleaved ADC code bytes."""
    samples_per_bit = rate_hz / bitrate
    levels = []
    for start_bit, ident, payload, dlc, _crc in scheduled:
        levels += [1] * (start_bit - len(levels))
        levels += full_frame(ident, payload, dlc)
    levels += [1] * 20

    total_samples = int(round(len(levels) * samples_per_bit))
    dt = 1.0 / rate_hz
    alpha = 1.0 - math.exp(-dt / rise_tau_s)

    canh = RECESSIVE_V
    canl = RECESSIVE_V
    out = bytearray(total_samples * 2)
    for index in range(total_samples):
        bit = levels[min(int(index / samples_per_bit), len(levels) - 1)]
        if bit:
            target_h, target_l = RECESSIVE_V, RECESSIVE_V
        else:
            target_h, target_l = DOMINANT_HIGH_V, DOMINANT_LOW_V
        canh += (target_h - canh) * alpha
        canl += (target_l - canl) * alpha
        vh = canh + (rng.gauss(0.0, noise_v) if noise_v else 0.0)
        vl = canl + (rng.gauss(0.0, noise_v) if noise_v else 0.0)
        code_h = round(OFFSET_CODES[0] + vh / VOLTS_PER_CODE[0])
        code_l = round(OFFSET_CODES[1] + vl / VOLTS_PER_CODE[1])
        out[2 * index] = max(0, min(255, int(code_h)))
        out[2 * index + 1] = max(0, min(255, int(code_l)))
    return bytes(out), total_samples, len(levels) / bitrate


def write_header(f, rate_hz, sample_count):
    header = struct.pack(
        "<8sII5dQ",
        MAGIC, VERSION, CHANNELS, rate_hz,
        VOLTS_PER_CODE[0], VOLTS_PER_CODE[1],
        OFFSET_CODES[0], OFFSET_CODES[1],
        sample_count,
    )
    assert len(header) == 64
    f.write(header)


def write_expected(path, scheduled, bitrate):
    """JSON Lines matching `--log-can`, for offline comparison."""
    import json
    with open(path, "w", encoding="utf-8") as f:
        for start_bit, ident, payload, dlc, crc in scheduled:
            f.write(json.dumps({
                "type": "frame",
                "t_ms": round(start_bit * 1000.0 / bitrate, 3),
                "id": ident,
                "id_hex": f"0x{ident:03X}",
                "dlc": dlc,
                "data": bytes(payload[:dlc]).hex(),
                "crc": crc,
            }) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", nargs="?", default="can_capture.bin")
    parser.add_argument("--frames", type=int, default=200,
                        help="number of CAN frames (default: 200)")
    parser.add_argument("--bitrate", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=10_000_000.0,
                        help="sample rate in Hz (default: 10 MS/s)")
    parser.add_argument("--noise-v", type=float, default=0.012,
                        help="per-channel Gaussian noise RMS in volts")
    parser.add_argument("--rise-tau-ns", type=float, default=40.0,
                        help="RC edge time constant in ns")
    parser.add_argument("--seed", type=int, default=0xCA4)
    parser.add_argument("--expected",
                        help="optional JSONL of the generated frames")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    scheduled = build_schedule(args.frames, args.bitrate, rng)
    data, sample_count, duration_s = render(
        scheduled, args.bitrate, args.rate, rng,
        args.noise_v, args.rise_tau_ns * 1e-9)

    with open(args.out, "wb") as f:
        write_header(f, args.rate, sample_count)
        f.write(data)

    if args.expected:
        write_expected(args.expected, scheduled, args.bitrate)

    print(f"wrote {args.out}: {sample_count} samples, "
          f"{duration_s * 1000.0:.2f} ms, {args.frames} frames, "
          f"{len(data) + 64} bytes")


if __name__ == "__main__":
    main()
