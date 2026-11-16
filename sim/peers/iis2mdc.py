# ST IIS2MDC magnetometer on I2C4 at 0x1E (WHO_AM_I 0x4F = 0x40, output 0x68..0x6D XYZ LE, 1.5 mG/LSB).
# MODELED SIMPLIFICATION: constant field (Earth-like, 250 mG on x, 0 elsewhere); control registers stored only.
import struct


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.regs = {0x4F: 0x40}
        self.ptr = 0

    def on_write(self, data):
        if not data:
            return
        self.ptr = data[0] & 0x7F
        for i, b in enumerate(data[1:]):
            self.regs[(self.ptr + i) & 0x7F] = b

    def on_read(self, count):
        field = struct.pack("<hhh", int(250 / 1.5), 0, 0)
        out = bytes((field[(self.ptr + i) - 0x68] if 0x68 <= self.ptr + i <= 0x6D else self.regs.get(self.ptr + i, 0)) for i in range(count))
        self.ptr += count
        return out
