# AK45 servo stand-in on the motor CAN (FDCAN1). Frame layout is read from the firmware's own driver
# (src/drivers/ak45_driver/ak45_driver.c): commands are extended ids (mode<<8)|0x45, mode 4 = position
# (int32 deg*1e4 big-endian), mode 6 = position+speed; feedback is id (0x29<<8)|0x45, 8 bytes:
# pos int16 x0.1 deg, speed int16 x10 erpm, current int16 x0.01 A, temp int8, fault u8.
# MODELED SIMPLIFICATION: the shaft slews to the commanded angle at a fixed rate (SLEW_DPS); it is not a
# motor model. The feedback period (5 ms, AK45_UPDATE_PERIOD_MS in the driver) is the firmware's own.
import os, struct
ID = 0x45
FB = (0x29 << 8) | ID
SLEW_DPS = 360.0
PERIOD_US = 5000


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.pos = 0.0
        self.target = 0.0
        self.cmds = 0
        self.log = open(os.environ.get("SIM_OUT", ".") + "/servo.csv", "w")
        self.log.write("t_s,cmd_deg,pos_deg,cmds_rx\n")
        ctx.schedule_periodic(PERIOD_US)

    def on_frame(self, frame):
        i = frame["id"]
        if (i & 0xFF) != ID or len(frame["data"]) < 4:
            return
        mode = i >> 8
        if mode in (4, 6):
            self.target = struct.unpack(">i", bytes(frame["data"][:4]))[0] / 10000.0
            self.cmds += 1

    def on_timer(self):
        step = SLEW_DPS * PERIOD_US * 1e-6
        d = max(-step, min(step, self.target - self.pos))
        self.pos += d
        raw = int(self.pos * 10)
        self.ctx.send(FB, struct.pack(">hhhbB", raw, int(d / (PERIOD_US * 1e-6) / 6 / 10 * 1), 0, 30, 0), extended=True)
        self.log.write("%.4f,%.3f,%.3f,%d\n" % (self.ctx.now_us / 1e6, self.target, self.pos, self.cmds))
        self.log.flush()
