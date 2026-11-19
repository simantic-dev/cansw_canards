# Passive RocketCAN listener on the canards board's FDCAN3 (250 kbit, extended IDs). Decodes the
# SID (rocketlib msg_common.h build_sid: prio<<27 | type<<20 | board_type<<14 | board_inst<<8 |
# metadata) and logs the first two frames of each message type, so the boot is observable:
# health_checks sends MSG_GENERAL_BOARD_STATUS every second, the sensor tasks their analog
# messages, and MSG_CANARD_FIRMWARE_ERROR reports the sensors that have no peer here.
TYPES = {
    0x01: "MSG_GENERAL_BOARD_STATUS", 0x02: "MSG_RESET_CMD", 0x03: "MSG_DEBUG_RAW",
    0x0A: "MSG_SENSOR_ANALOG16", 0x0B: "MSG_SENSOR_ANALOG32", 0x0C: "MSG_SENSOR_2D_ANALOG24",
    0x0D: "MSG_SENSOR_3D_ANALOG16", 0x16: "MSG_TELEMETRY_INFO", 0x18: "MSG_CANARD_FIRMWARE_ERROR",
}


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.seen = {}

    def on_frame(self, frame):
        cid, data = frame["id"], bytes(frame["data"])
        mtype = (cid >> 20) & 0x7F
        board_type, board_inst = (cid >> 14) & 0x3F, (cid >> 8) & 0x3F
        self.seen[mtype] = self.seen.get(mtype, 0) + 1
        if self.seen[mtype] <= 2:
            name = TYPES.get(mtype, f"MSG type 0x{mtype:02x}")
            self.ctx.info(f"{name} from board type 0x{board_type:02x} inst 0x{board_inst:02x}: id=0x{cid:08x} data={data.hex()}")
