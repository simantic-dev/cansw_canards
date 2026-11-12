# CubeMars AK45 servo stand-in on the canards board's FDCAN1 (1 Mbit, extended IDs).
# The driver's init waits for any frame matching (CAN_REAL_TIME_FEEDBACK << 8 | AK45_DRIVER_ID),
# which the real motor streams at 300 Hz (ak45_driver.c). This peer streams that frame and logs
# every command the board sends it. CAN_PACKET_SET_POS (4) carries the angle as a big-endian
# int32 in 1e-4 deg (ak45_send_position_cmd, AK45_POS_CMD_DEG_TO_POS).
AK45_ID = 0x45
FEEDBACK = 0x29
PERIOD_US = 3_333
CAN_PACKET_SET_POS = 4
POS_SCALE = 10000.0


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.sent = 0
        self.cmds = {}
        self.pos_count = 0
        self.pos_report_us = 0
        self.ctx.schedule_periodic(PERIOD_US)

    def on_timer(self):
        # position 0, speed 0, current 0, temperature 25 C, no error (ak45_parse_feedback layout)
        self.ctx.send((FEEDBACK << 8) | AK45_ID, bytes([0, 0, 0, 0, 0, 0, 25, 0]), extended=True)
        self.sent += 1
        if self.sent == 1:
            self.ctx.info("feedback stream started")

    def on_frame(self, frame):
        cid, data = frame["id"], bytes(frame["data"])
        if cid & 0xFF != AK45_ID:
            return
        cmd = (cid >> 8) & 0xFF
        self.cmds[cmd] = self.cmds.get(cmd, 0) + 1
        if self.cmds[cmd] == 1:
            self.ctx.info(f"command 0x{cmd:02x} from the board: {data.hex()}")
        if cmd == CAN_PACKET_SET_POS and len(data) == 4:
            self.on_set_pos(int.from_bytes(data, "big", signed=True) / POS_SCALE)

    def on_set_pos(self, deg):
        self.pos_count += 1
        if self.pos_count == 1:
            self.ctx.info(f"SET_POS {deg:.2f} deg at {self.ctx.now_us / 1e6:.3f} s")
            self.pos_report_us = self.ctx.now_us
        elif self.ctx.now_us - self.pos_report_us >= 1_000_000:
            # one line per second, not per command: the board sends at its loop rate
            self.pos_report_us = self.ctx.now_us
            self.ctx.info(f"SET_POS count {self.pos_count}, latest {deg:.2f} deg")
