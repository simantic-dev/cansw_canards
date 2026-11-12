# TE MS5611 barometer on I2C5 at 0x77 (CSB low), src/drivers/MS5611/MS5611.c. Command bytes are
# plain writes; PROM and ADC reads follow as a write-then-read (i2c_read_reg).
# The driver runs the datasheet's first-order formula plus its second-order cold correction
# (below 20 C, SECOND_COMP_TEMP_THRESHOLD), so the peer inverts the driver's own compensation
# by bisection on D1/D2 instead of the first-order formula alone.
import flight

PROM_WORDS = [0x0000, 40127, 36924, 23317, 23282, 33464, 28312, 0]  # C0 factory, C1..C6, CRC word
PAD_PA = round(flight.pressure_pa(0.0))
ADC_MAX = (1 << 24) - 1


def crc4(w):
    """a_ms5611_crc_check: the CRC nibble over words 0..7 with word 7 cleared."""
    rem = 0
    for cnt in range(16):
        rem ^= (w[cnt >> 1] & 0xFF) if cnt & 1 else (w[cnt >> 1] >> 8)
        for _ in range(8):
            rem = (((rem << 1) ^ 0x3000) if rem & 0x8000 else (rem << 1)) & 0xFFFF
    return (rem >> 12) & 0x0F


PROM_WORDS[7] = crc4(PROM_WORDS)


def compensate(d1, d2):
    """MS5611.c ms5611_read_raw_pressure: (temperature in centi-C, pressure in Pa)."""
    c1, c2, c3, c4, c5, c6 = PROM_WORDS[1:7]
    dt = d2 - (c5 << 8)
    temp = 2000 + ((dt * c6) >> 23)
    off = (c2 << 16) + ((c4 * dt) >> 7)
    sens = (c1 << 15) + ((c3 * dt) >> 8)
    if temp < 2000:
        t2 = (dt * dt) >> 31
        sq = (temp - 2000) ** 2
        off2, sens2 = (5 * sq) >> 1, (5 * sq) >> 2
        if temp < -1500:
            sq2 = (temp + 1500) ** 2
            off2 += 7 * sq2
            sens2 += (11 * sq2) >> 1
        temp -= t2
        off -= off2
        sens -= sens2
    return temp, (((d1 * sens) >> 21) - off) >> 15


def invert(f, target):
    """Smallest 24-bit ADC code whose output reaches target (f increases with the code)."""
    lo, hi = 0, ADC_MAX
    while lo < hi:
        mid = (lo + hi) // 2
        if f(mid) < target:
            lo = mid + 1
        else:
            hi = mid
    return lo


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.cmd = 0
        self.adc = bytes(3)
        self.d2 = 0
        self.logged_prom = False
        self.logged_flight = False

    def on_write(self, data):
        self.cmd = data[0]
        t = self.ctx.now_us / 1e6
        if self.cmd & 0xF0 == 0x50:  # D2: temperature, latched for the next ADC read
            # shortcut: OSR ignored (no quantisation noise), conversion time not modelled
            self.d2 = invert(lambda d: compensate(0, d)[0], round(flight.temperature_c(t) * 100))
            self.adc = self.d2.to_bytes(3, "big")
        elif self.cmd & 0xF0 == 0x40:  # D1: pressure, against the D2 the driver cached
            p_pa = flight.pressure_pa(t)
            d1 = invert(lambda d: compensate(d, self.d2)[1], round(p_pa))
            self.adc = d1.to_bytes(3, "big")
            if not self.logged_flight and round(p_pa) < PAD_PA:
                self.logged_flight = True
                self.ctx.info(f"first in-flight sample: {round(p_pa)} Pa below pad {PAD_PA} Pa, t={t:.3f} s")

    def on_read(self, count):
        cmd = self.cmd
        if cmd == 0x00:  # ADC read: last conversion, 3 bytes big-endian
            out = self.adc
        elif 0xA0 <= cmd <= 0xAE:  # PROM word 0..7, 2 bytes big-endian
            addr = (cmd - 0xA0) >> 1
            out = PROM_WORDS[addr].to_bytes(2, "big")
            if addr == 7 and not self.logged_prom:  # the driver reads 0..7 in order
                self.logged_prom = True
                self.ctx.info(f"PROM read, C1..C6 served: {PROM_WORDS[1:7]}")
        else:  # reset (0x1E) and anything else reads back nothing
            out = b""
        return (out + bytes(count))[:count]

    def on_finish(self):
        pass
