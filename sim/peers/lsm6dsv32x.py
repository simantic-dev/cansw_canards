# ST LSM6DSV32X IMU on I2C1 at 0x6A, as a scripted I2C peer. Register addresses and the 0x70 WHO_AM_I come
# from the firmware's own regmap (src/drivers/lsm6dsv32x). Scales are the ones the driver applies:
# gyro 0.140 dps/LSB, accel 0.000976 g/LSB. Reads auto-increment. Output regs 0x22..0x2D: gyro XYZ, accel XYZ.
# MODELED SIMPLIFICATION: control registers are stored and ignored; data is the synthetic profile in flight.py.
import os, struct, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight import accel_g, roll_dps
GYRO_LSB, ACC_LSB = 0.140, 0.000976


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.regs = {0x0F: 0x70}
        self.ptr = 0

    def on_write(self, data):
        if not data:
            return
        self.ptr = data[0]
        for i, b in enumerate(data[1:]):
            self.regs[(self.ptr + i) & 0xFF] = b

    def _out(self):
        t = self.ctx.now_us / 1e6
        s = lambda v, lsb: max(-32768, min(32767, int(round(v / lsb))))
        return struct.pack("<hhhhhh", s(roll_dps(t), GYRO_LSB), 0, 0, s(accel_g(t), ACC_LSB), 0, 0)

    def on_read(self, count):
        out = bytearray()
        for i in range(count):
            r = (self.ptr + i) & 0xFF
            out.append(self._out()[r - 0x22] if 0x22 <= r <= 0x2D else self.regs.get(r, 0))
        self.ptr = (self.ptr + count) & 0xFF
        return bytes(out)
