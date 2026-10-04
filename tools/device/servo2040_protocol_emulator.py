#!/usr/bin/env python3
"""Firmware-derived Servo2040 serial protocol emulator.

This models the byte stream in firmware/servo2040-driver/chica-servo2040:
SET (0xd3) writes servo/relay values, GET (0xc7) echoes the three-byte header
and returns two 7-bit bytes per requested pin.
"""

from __future__ import annotations

from dataclasses import dataclass, field


SET_CMD = 0xD3
GET_CMD = 0xC7

SERVO1 = 0
SERVO18 = 17
TS1 = 18
TS6 = 23
CURR = 24
VOLT = 25
RELAY = 26
A1 = 27
A2 = 28

B1024_3_3V_RATIO = 310.3
CURR_LSB = 0.0814


def encode14(value: int) -> bytes:
    value &= 0x3FFF
    return bytes((value & 0x7F, (value >> 7) & 0x7F))


def decode14(low: int, high: int) -> int:
    return (low & 0x7F) | ((high & 0x7F) << 7)


@dataclass
class Servo2040ProtocolEmulator:
    servos: list[int] = field(default_factory=lambda: [1500] * 18)
    touch_voltages: list[float] = field(default_factory=lambda: [0.0] * 6)
    current_amps: float = 0.0
    voltage: float = 0.0
    relay_enabled: bool = False
    outputs: dict[int, bool] = field(default_factory=dict)

    def set_analog(self, *, touches: list[float] | None = None,
                   voltage: float | None = None,
                   current_amps: float | None = None) -> None:
        if touches is not None:
            if len(touches) != 6:
                raise ValueError("touches must contain six voltages")
            self.touch_voltages = list(touches)
        if voltage is not None:
            self.voltage = voltage
        if current_amps is not None:
            self.current_amps = current_amps

    def exchange(self, data: bytes | bytearray | list[int]) -> bytes:
        request = bytes(data)
        reply = bytearray()
        offset = 0
        while offset < len(request):
            cmd = request[offset]
            offset += 1
            if (cmd & 0x80) == 0:
                continue
            if offset + 2 > len(request):
                break
            start = request[offset]
            count = request[offset + 1]
            offset += 2
            if cmd == SET_CMD:
                values: list[int] = []
                for _ in range(count):
                    if offset + 2 > len(request):
                        return bytes(reply)
                    values.append(decode14(request[offset], request[offset + 1]))
                    offset += 2
                self._set(start, values)
            elif cmd == GET_CMD:
                reply.extend((GET_CMD, start, count))
                for pin in range(start, start + count):
                    reply.extend(encode14(self._read_raw(pin)))
            else:
                break
        return bytes(reply)

    def _set(self, start: int, values: list[int]) -> None:
        for index, value in enumerate(values):
            pin = start + index
            if SERVO1 <= pin <= SERVO18:
                self.servos[pin] = value
            elif pin >= RELAY:
                enabled = value != 0
                self.outputs[pin] = enabled
                if pin == RELAY:
                    self.relay_enabled = enabled

    def _read_raw(self, pin: int) -> int:
        if SERVO1 <= pin <= SERVO18:
            return self.servos[pin]
        if TS1 <= pin <= TS6:
            return round(self.touch_voltages[pin - TS1] * B1024_3_3V_RATIO)
        if pin == CURR:
            return round(self.current_amps / CURR_LSB) + 512
        if pin == VOLT:
            return round(self.voltage * B1024_3_3V_RATIO)
        return 0
