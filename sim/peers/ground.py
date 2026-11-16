# Ground-side node on the rocket CAN bus (FDCAN3): logs every frame to $SIM_OUT/can3.csv (t_s,id_hex,data_hex)
# and sends the two actuator commands the flight-phase module listens for (flight_phase.c act_cmd_callback):
# CANARD_PAD_FILTER on at PAD_FILTER_S, IGNITION on at IGNITION_S. Frame layout is rocketlib's own
# (build_sid: prio<<27 | msg_type<<20 | board_type<<14 | board_inst<<8 | actuator id; data = timestamp u16, state).
import os

PAD_FILTER_S, IGNITION_S = 1.5, 2.0
MSG_ACTUATOR_CMD, ACT_CANARD_PAD_FILTER, ACT_IGNITION, ON = 0x06, 0x0E, 0x02, 0x00
SENDER_TYPE, SENDER_INST = 0x0B, 0x01          # RLCS_GLS, GROUND


def cmd_id(actuator):
    return (2 << 27) | (MSG_ACTUATOR_CMD << 20) | (SENDER_TYPE << 14) | (SENDER_INST << 8) | actuator


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.f = open(os.environ.get("SIM_OUT", ".") + "/can3.csv", "w")
        self.f.write("t_s,id,data\n")
        self.todo = [(PAD_FILTER_S, ACT_CANARD_PAD_FILTER), (IGNITION_S, ACT_IGNITION)]
        ctx.schedule_oneshot(int(self.todo[0][0] * 1e6))

    def on_timer(self):
        t_s, act = self.todo.pop(0)
        ts = int(t_s * 1000) & 0xFFFF
        self.ctx.send(cmd_id(act), bytes([ts >> 8, ts & 0xFF, ON]), extended=True)
        if self.todo:
            self.ctx.schedule_oneshot(int(self.todo[0][0] * 1e6) - self.ctx.now_us)

    def on_frame(self, frame):
        self.f.write("%.4f,%x,%s\n" % (self.ctx.now_us / 1e6, frame["id"], bytes(frame["data"]).hex()))
        self.f.flush()
