# Movella/Xsens MTi IMU on the canards board's USART3 (2 Mbaud, 8N1). The driver sends no
# configuration (the unit is pre-configured with the Xsens app) and only consumes MTData2
# (MID 0x36) at 200 Hz: one frame per 5 ms tick, carrying the fields movella_event_callback
# reads. Values come from the shared flight profile sampled at virtual time.
import struct

import flight

PERIOD_US = 5_000
MID_MTDATA2 = 0x36
PREAMBLE = (0xFA, 0xFF)
XDI_TIME_FINE_TICKS_US = 100  # XDI_SAMPLE_TIME_FINE counts 10 kHz ticks


def xdi(xid, fmt, *values):
    # XDI id (BE u16), length (u8), big-endian data
    data = struct.pack(">" + fmt, *values)
    return struct.pack(">HB", xid, len(data)) + data


def mtdata2(counter, t_us):
    t_s = t_us / 1e6
    pa = flight.proper_accel_up(t_s)
    gx, gy, gz = flight.gyro_radps(t_s)
    mx, my, mz = (v / 40.0 for v in flight.MAG_UT)  # arbitrary units, as the driver expects
    payload = b"".join([
        xdi(0x1020, "H", counter & 0xFFFF),
        xdi(0x1060, "I", (t_us // XDI_TIME_FINE_TICKS_US) & 0xFFFFFFFF),
        # shortcut: the proper acceleration is put on Z (up); the driver's IMU-to-body
        # correction matrix is not modelled, so the body-axis mapping is unverified
        xdi(0x4020, "fff", 0.0, 0.0, pa),
        xdi(0x8020, "fff", gx, gy, gz),
        xdi(0xC020, "fff", mx, my, mz),
        xdi(0x2010, "ffff", 1.0, 0.0, 0.0, 0.0),
        xdi(0x3010, "I", int(round(flight.pressure_pa(t_s)))),
        xdi(0x0810, "f", flight.temperature_c(t_s)),
        xdi(0xE020, "I", 0),
    ])
    body = bytes((0xFF, MID_MTDATA2, len(payload))) + payload
    checksum = (-sum(body)) & 0xFF  # frame bytes after the preamble sum to 0 mod 256
    return bytes(PREAMBLE) + bytes((MID_MTDATA2, len(payload))) + payload + bytes((checksum,))


class Peer:
    def __init__(self, ctx):
        self.ctx = ctx
        self.counter = 0
        self.sent = 0
        self.rx_bytes = 0
        self.ctx.info("MTData2 at 200 Hz")
        self.ctx.schedule_periodic(PERIOD_US)

    def on_byte(self, b):
        # the driver writes nothing to the IMU in this build; count and drop anything it does send
        self.rx_bytes += 1

    def on_timer(self):
        t_us = self.ctx.now_us
        self.ctx.send(mtdata2(self.counter, t_us))
        self.counter += 1
        self.sent += 1
        if self.sent == 1:
            self.ctx.info("first MTData2 frame sent")
