# ST IIS2MDC magnetometer on the canards board's I2C4, 7-bit 0x1E, DRDY on PE0 (INT_MAG,
# EXTI0; main.h INT_MAG_Pin on GPIOE). Register file with an 8-bit pointer; bit 7 of the pointer
# is the auto-increment flag, so it is masked to 7 bits. A HAL Mem_Write/Mem_Read sends the
# pointer as its own chunk; on_finish or a read ends the transaction.
# Scale from IIS2MDC.c: 1.5 mgauss/LSB; 1 uT = 10 mgauss.
import flight

GAUSS_PER_LSB = 0.0015
WHO_AM_I, CFG_C = 0x4F, 0x62
OUTX_L = 0x68                    # X/Y/Z, int16 LE
DRDY_PERIOD_US = 10_000          # 100 Hz, the ODR set by CFG_REG_A = 0x8C
SELF_TEST_GAUSS = 0.3            # shortcut: on-chip self-test field, range 0.015-0.5 G; mid-range stand-in
SELF_TEST_CFG_C = 0x02


def _i16(v):
    return max(-32768, min(32767, int(round(v)))).to_bytes(2, "little", signed=True)


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.regs = {WHO_AM_I: 0x40}
        self.ptr = 0
        self.in_txn = False
        self.drdy_on = False

    def _field_gauss(self):
        # MAG_UT is in uT; 100 uT = 1 G. The peers rotate nothing, so the sensor frame is the field.
        g = [v / 100.0 for v in flight.MAG_UT]
        if self.regs.get(CFG_C, 0) & SELF_TEST_CFG_C:
            g = [v + SELF_TEST_GAUSS for v in g]
        return g

    def on_write(self, data):
        data = bytes(data)
        if not self.in_txn:
            self.in_txn = True
            self.ptr, data = data[0] & 0x7F, data[1:]
        for v in data:
            self.regs[self.ptr] = v
            if self.ptr == CFG_C and v & 0x01 and not self.drdy_on:
                self.drdy_on = True
                self.ctx.info("configured, 100 Hz")
                self.ctx.schedule_periodic(DRDY_PERIOD_US)
            self.ptr += 1

    def on_timer(self):
        # DRDY stays high until the data is read, as on the part
        self.ctx.gpio(0).set(True)

    def on_finish(self):
        self.in_txn = False

    def on_read(self, count):
        self.in_txn = False
        start = self.ptr & 0x7F
        if OUTX_L <= start < OUTX_L + 6:
            self.ctx.gpio(0).set(False)
            buf = b"".join(_i16(v / GAUSS_PER_LSB) for v in self._field_gauss())
            out = buf[start - OUTX_L:]
        else:
            out = bytes(self.regs.get(start + i, 0) for i in range(count))
        return (out + bytes(count))[:count]
