# Ground station on the canards board's FDCAN3 (250 kbit, extended IDs). It stands in for the
# launch controller: arms the pad filter at 1.5 s and opens the ox injector valve at LAUNCH_S,
# which the FSM takes as launch (flight_phase.c, STATE_PAD_FILTER on EVENT_INJ_OPEN). Frames
# from the board are logged once per RocketCAN message type.
from flight import LAUNCH_S

PRIO = 0x3  # PRIO_LOW (packet_format.h)
MSG_ACTUATOR_CMD = 0x006
ACTUATOR_OX_INJECTOR_VALVE = 0x00
ACTUATOR_CANARD_PAD_FILTER = 0x0E
ACT_STATE_ON = 0x00
PAD_FILTER_US = 1_500_000
LAUNCH_US = int(LAUNCH_S * 1_000_000)
# build_sid: prio<<27 | type<<20 | board_type<<14 | board_inst<<8 | metadata. Board type and
# instance 0 (BOARD_TYPE_ID_ANY) are accepted: the actuator callback never checks the sender.
TYPE_NAMES = {0x01: "MSG_GENERAL_BOARD_STATUS", 0x0A: "MSG_SENSOR_ANALOG16", 0x18: "MSG_CANARD_FIRMWARE_ERROR"}
CTRL_CMD_ANGLE = 0x35  # SENSOR_CANARD_CTRL_CMD_ANGLE, sent by Ctrl CAN telemetry (controller.c)


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.sent = set()
        self.seen = set()
        self.ctrl_logged = False
        self.ctx.schedule_periodic(100_000)

    def on_timer(self):
        now = self.ctx.now_us
        if now >= PAD_FILTER_US and "pad" not in self.sent:
            self.sent.add("pad")
            self.send_actuator(ACTUATOR_CANARD_PAD_FILTER, "pad filter ON")
        if now >= LAUNCH_US and "ox" not in self.sent:
            self.sent.add("ox")
            self.send_actuator(ACTUATOR_OX_INJECTOR_VALVE, "ox injector valve ON")

    def send_actuator(self, actuator_id, text):
        ts = (self.ctx.now_us // 1000) & 0xFFFF
        sid = (PRIO << 27) | (MSG_ACTUATOR_CMD << 20) | actuator_id
        self.ctx.send(sid, bytes([ts >> 8, ts & 0xFF, ACT_STATE_ON]), extended=True)
        self.ctx.info(f"{text} at {self.ctx.now_us / 1e6:.3f} s")

    def on_frame(self, frame):
        cid, data = frame["id"], bytes(frame["data"])
        mtype = (cid >> 20) & 0x7F
        if mtype == 0x0A and (cid & 0xFF) == CTRL_CMD_ANGLE and not self.ctrl_logged:
            # no state message exists: the controller's telemetry only runs in PAD_NAV, BOOST and
            # ACT_ALLOWED, so its first frame marks the FSM leaving PAD_FILTER
            self.ctrl_logged = True
            self.ctx.info(f"canards ctrl telemetry started at {self.ctx.now_us / 1e6:.3f} s (FSM past PAD_FILTER)")
        if mtype not in self.seen:
            self.seen.add(mtype)
            name = TYPE_NAMES.get(mtype, f"MSG type 0x{mtype:02x}")
            self.ctx.info(f"first {name}: id=0x{cid:08x} data={data.hex()}")
