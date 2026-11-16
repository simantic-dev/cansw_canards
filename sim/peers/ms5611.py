# TE MS5611 barometer on I2C5 at 0x77. Command set from the firmware's driver (src/drivers/MS5611): 0x1E reset,
# 0xA0|(n<<1) PROM word n, 0x40.. D1 (pressure) and 0x50.. D2 (temperature) conversions, 0x00 reads the 24-bit result.
# PROM is the datasheet's example set with the CRC-4 recomputed here by the datasheet algorithm. The conversion
# result is the inverse of the datasheet first-order formula for ~1013 mbar / 20 C, so the driver computes it back.
# MODELED SIMPLIFICATION: fixed pressure and temperature; conversion completes instantly.
PROM = [0, 40127, 36924, 23317, 23282, 33464, 28312, 0]


def crc4(prom):
    p = list(prom)
    p[7] &= 0xFF00
    rem = 0
    for cnt in range(16):
        rem ^= (p[cnt >> 1] & 0xFF) if cnt & 1 else (p[cnt >> 1] >> 8)
        for _ in range(8):
            rem = ((rem << 1) ^ 0x3000) & 0xFFFF if rem & 0x8000 else (rem << 1) & 0xFFFF
    return (rem >> 12) & 0xF


PROM[7] = crc4(PROM)
TEMP_C, P_MBAR = 20.0, 1013.25


def raw_values():
    c1, c2, c3, c4, c5, c6 = PROM[1:7]
    dT = (TEMP_C * 100 - 2000) * (1 << 23) / c6
    d2 = int(dT + c5 * 256)
    off, sens = c2 * 65536 + c4 * dT / 128, c1 * 32768 + c3 * dT / 256
    # P = (D1*SENS/2^21 - OFF)/2^15  ->  D1 = (P*2^15 + OFF)*2^21/SENS
    d1 = int((P_MBAR * 100 * (1 << 15) + off) * (1 << 21) / sens)
    return d1, d2


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.cmd = 0
        self.result = 0

    def on_write(self, data):
        if not data:
            return
        self.cmd = data[0]
        d1, d2 = raw_values()
        if 0x40 <= self.cmd <= 0x48:
            self.result = d1
        elif 0x50 <= self.cmd <= 0x58:
            self.result = d2

    def on_read(self, count):
        if 0xA0 <= self.cmd <= 0xAE:
            w = PROM[(self.cmd - 0xA0) >> 1]
            return bytes([w >> 8, w & 0xFF])[:count].ljust(count, b"\0")
        return self.result.to_bytes(3, "big")[:count].ljust(count, b"\0")
