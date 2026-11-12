# ST LSM6DSV32X IMU on the canards board's I2C1, 7-bit 0x6A, INT1 on PD7 (IMU_INT1, EXTI7).
# Register file with an 8-bit pointer. HAL Mem_Write/Mem_Read send the pointer as its own chunk,
# so the first byte of a transaction sets the pointer; on_finish or a read ends the transaction.
# Scales from LSM6DSV32X.c: +-32 g at 0.000976 g/LSB, +-4000 dps at 0.140 dps/LSB.
# Samples come from flight.py at the virtual time of each read, so every DMA burst is fresh.
import math

import flight

ACC_G_PER_LSB = 0.000976
GYRO_DPS_PER_LSB = 0.140
WHO_AM_I, INT1_CTRL = 0x0F, 0x0D
OUTX_L_G = 0x22                  # gyro X/Y/Z then accel X/Y/Z, int16 LE
DRDY_HALF_PERIOD_US = 1042       # shortcut: 1041.7 us half period of 480 Hz rounded to whole us
LAUNCH_ACCEL = 30.0              # m/s^2, flight_phase.c ACCEL_THRESHOLD_LAUNCH_M_S2


def _i16(v):
    return max(-32768, min(32767, int(round(v)))).to_bytes(2, "little", signed=True)


def frame(t_s):
    """12 bytes of the burst at OUTX_L_G for flight time t_s, and the up accel in m/s^2."""
    # Board = M * sensor with M = [[0,0,-1],[1,0,0],[0,-1,0]] (sensor_handler.c), so board up
    # (x) is -sensor z; sensor = M^T * board gives the gyro axes below.
    up = flight.proper_accel_up(t_s)
    gx, gy, gz = flight.gyro_radps(t_s)
    gyro = (gy, -gz, -gx)
    acc = (0.0, 0.0, -up / flight.G)
    raw = [math.degrees(v) / GYRO_DPS_PER_LSB for v in gyro]
    raw += [v / ACC_G_PER_LSB for v in acc]
    return b"".join(_i16(v) for v in raw), up


class Slave:
    def __init__(self, ctx):
        self.ctx = ctx
        self.regs = {WHO_AM_I: 0x70}
        self.ptr = 0
        self.in_txn = False
        self.drdy_on = False
        self.launch_logged = False
        self.level = False

    def on_write(self, data):
        data = bytes(data)
        if not self.in_txn:
            self.in_txn = True
            self.ptr, data = data[0], data[1:]
        for v in data:
            self.regs[self.ptr] = v
            if self.ptr == INT1_CTRL and v & 0x01 and not self.drdy_on:
                self.drdy_on = True
                self.ctx.info("configured, 480 Hz")
                self.ctx.schedule_periodic(DRDY_HALF_PERIOD_US)
            self.ptr += 1

    def on_timer(self):
        # toggling gives one rising edge per 2 half periods, i.e. at the ODR
        self.level = not self.level
        self.ctx.gpio(0).set(self.level)

    def on_finish(self):
        self.in_txn = False

    def on_read(self, count):
        self.in_txn = False
        start = self.ptr
        if OUTX_L_G <= start < OUTX_L_G + 12:
            buf, up = frame(self.ctx.now_us / 1e6)
            if up > LAUNCH_ACCEL and not self.launch_logged:
                self.launch_logged = True
                self.ctx.info(f"launch-phase sample above 30 m/s^2: proper accel up {up:.1f}")
            out = buf[start - OUTX_L_G:]
        else:
            out = bytes(self.regs.get(start + i, 0) for i in range(count))
        return (out + bytes(count))[:count]
