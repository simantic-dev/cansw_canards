# TI ADS1219 24-bit ADC on I2C2 at 0x40: the ADXRS649 gyro's converter on the AD breakout board
# (src/drivers/ad_breakout_board/ADS1219.c, ADXRS649.c). The gyro uses one differential input,
# AIN0-AIN1, gain 1, external reference 0..2048 mV, 1000 SPS, continuous (ADXRS649.c init).
# The data-ready line is ADC_INT, PE12 (main.h), active low; the driver polls it first and
# falls back to RREG 0x24 only if the GPIO read fails.
import math

import flight

CMD_RESET, CMD_POWERDOWN, CMD_START = 0x06, 0x02, 0x08
CMD_RDATA, CMD_RREG_CONFIG, CMD_RREG_STATUS, CMD_WREG = 0x10, 0x20, 0x24, 0x40
PERIOD_US = 1000          # 1000 SPS
SPAN_MV = 2048.0          # AREF_P - AREF_N, gain 1
DPS_PER_MV = 0.1          # ADXRS649.c MV_TO_DPS, so mV = dps / 0.1
ADC_MAX = (1 << 23) - 1   # 24-bit two's complement, 23 bits of magnitude

# shortcut: the AD board's ADXL380 accelerometer is not behind this ADC (it is its own I2C2
# part), so only the gyro channel is served. The gyro self-test drives ST1/ST2 as firmware
# outputs this peer cannot read, so the test (needs +100..200 mV then -100..-200 mV) fails.


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.cmd = 0
        self.config = 0x00   # power-on default
        self.ready = False
        self.wreg_pending = False  # WREG data may arrive in a later chunk
        self.logged_flight = False
        self.ctx.gpio(0).set(True)  # ADC_INT high: no data

    def _new_sample(self):
        self.ready = True
        self.ctx.gpio(0).set(False)

    def _sample_bytes(self):
        t = self.ctx.now_us / 1e6
        dps = math.degrees(flight.gyro_radps(t)[2])  # roll axis, the only gyro axis in flight.py
        code = max(-ADC_MAX, min(ADC_MAX, round(dps / DPS_PER_MV * (1 << 23) / SPAN_MV)))
        if dps and not self.logged_flight:
            self.logged_flight = True
            self.ctx.info(f"first in-flight sample: {dps:.2f} dps")
        return (code & 0xFFFFFF).to_bytes(3, "big")

    def on_write(self, data):
        if self.wreg_pending:
            self.config, self.wreg_pending = data[0], False
            return
        self.cmd = data[0]
        if self.cmd == CMD_RESET:
            self.config, self.ready = 0x00, False
            self.ctx.cancel_timer()
            self.ctx.gpio(0).set(True)
        elif self.cmd == CMD_START:
            self.ctx.schedule_periodic(PERIOD_US, self._new_sample)
        elif self.cmd == CMD_POWERDOWN:
            self.ctx.cancel_timer()
        elif self.cmd == CMD_WREG:
            if len(data) >= 2:
                self.config = data[1]
            else:
                self.wreg_pending = True

    def on_read(self, count):
        cmd = self.cmd
        if cmd == CMD_RREG_CONFIG:
            out = bytes([self.config])
        elif cmd == CMD_RREG_STATUS:
            out = bytes([0x80 if self.ready else 0x00])  # DRDY is bit 7
        elif cmd == CMD_RDATA:
            out = self._sample_bytes()
            self.ready = False
            self.ctx.gpio(0).set(True)  # reading the result clears DRDY
        else:
            out = b""
        return (out + bytes(count))[:count]

    def on_finish(self):
        pass
