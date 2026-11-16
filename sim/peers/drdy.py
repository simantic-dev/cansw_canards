# Data-ready strobes for the IMU (INT1, 960 Hz, ODR from CTRL1_XL = 0x08 in the driver) and the magnetometer
# (DRDY, 100 Hz, CFG_REG_A ODR = 100 Hz). Line 0 -> PD7 (IMU_INT1), line 1 -> PE0 (INT_MAG); rising edge, 20 us wide.
# MODELED SIMPLIFICATION: strobes run free from t=0; a real part strobes only once its ODR is programmed.
PERIOD_US = {0: 1_000_000 // 960, 1: 10_000}
WIDTH_US = 20


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.next = {0: PERIOD_US[0], 1: PERIOD_US[1]}
        self.falling = []
        for l in (0, 1):
            ctx.gpio(l).set(False)
        self._arm()

    def _arm(self):
        t = min(list(self.next.values()) + [f[0] for f in self.falling])
        self.ctx.schedule_oneshot(max(1, t - self.ctx.now_us))

    def on_timer(self):
        now = self.ctx.now_us
        for f in [f for f in self.falling if f[0] <= now]:
            self.falling.remove(f)
            self.ctx.gpio(f[1]).set(False)
        for l in (0, 1):
            if self.next[l] <= now:
                self.ctx.gpio(l).set(True)
                self.falling.append((now + WIDTH_US, l))
                self.next[l] += PERIOD_US[l]
        self._arm()

    def on_gpio(self, line, level):
        pass
