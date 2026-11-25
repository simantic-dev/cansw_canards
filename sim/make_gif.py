"""Builds sim/screenshots/signals.gif from a run with the FSM trace. Pillow + ffmpeg only (sim/gifkit.py).
Run (repo root, after sim/build.sh):
  SIM_OUT=sim/out sim run --scenario sim/scenario.yaml --timeout 22 --on-fault continue --trace-symbol flight_phase_update_state > sim/out/stdout.txt
  python3 sim/make_gif.py <sim-version>
CAN ids: rocketlib sid = prio<<27 | msg_type<<20 | board_type<<14 | inst<<8 | meta (can_message/packet_format.h).
The FSM state is not on the CAN bus; it is read from the arguments of flight_phase_update_state(event, curr_state)."""
import csv, re, sys
sys.path.insert(0, "sim")
from gifkit import *

VER = sys.argv[1]
OFF, R2D = 32768, 57.29578
STATES = ["IDLE", "PAD_FILTER", "PAD_NAV", "BOOST", "ACT_ALLOWED", "RECOVERY", "SLEEPY", "ERROR"]   # fsm_state_t, common/gnc/gnc_types.h
EVENTS = ["NONE", "PAD_FILTER", "IGNITOR", "LAUNCH_ACCEL", "INJ_OPEN", "ACT_DELAY_ELAPSED", "RECOVERY_START", "SLEEP_START"]  # flight_phase.h
MODS = "ADC ADXL380 ADXRS649 AK45 CAN_HANDLER CONTROLLER FLIGHT_PHASE FSM GPIO I2C IIS2MDC LOGGER LSM6DSV32X MOVELLA MS5611 NAVIGATOR POWER_HANDLER SD_CARD SENSOR_HANDLER TELEMETRY TIMER UART".split()

acc, gyr, cmd, srv_a, health, nav = [], [], [], [], {}, []
for r in csv.DictReader(open("sim/out/can3.csv")):
    i, d, t = int(r["id"], 16), bytes.fromhex(r["data"]), float(r["t_s"])
    typ, meta = (i >> 20) & 0x7F, i & 0xFF
    v3 = [(int.from_bytes(d[2 + 2 * k:4 + 2 * k], "big") - OFF) / 100 for k in range(3)] if len(d) >= 8 else None
    if typ == 0x0D and meta == 2: acc.append((t, v3))
    elif typ == 0x0D and meta == 3: gyr.append((t, [x * R2D for x in v3]))
    elif typ == 0x0A and meta == 0x35: cmd.append((t, (int.from_bytes(d[2:4], "big") - OFF) / 1000 * R2D))
    elif typ == 0x0B and meta == 0x3B: srv_a.append((t, (int.from_bytes(d[2:4], "big") - OFF) / 1000 * R2D))
    elif typ == 0x18: health.setdefault(meta, []).append((t, int.from_bytes(d[2:6], "big"), d[6]))
servo = [(float(r["t_s"]), float(r["pos_deg"])) for r in csv.DictReader(open("sim/out/servo.csv"))]
fsm, log, last = [], [], None   # fsm: (t, state)
for l in open("sim/out/stdout.txt", errors="replace"):
    m = re.match(r"\[([\d.]+)s\] \(TRACE\) flight_phase_update_state pc=\S+ r0=0x(\w+) r1=0x(\w+)", l)
    if not m: continue
    t, e, s = float(m[1]), int(m[2], 16), int(m[3], 16)
    if last is None or s != last[1]:
        fsm.append((t, s)); log.append((t, "INFO", f"fsm state is {STATES[s]}"))
    if e: log.append((t, "WARN", f"flight_phase_update_state(event={EVENTS[e]}, state={STATES[s]})"))
    last = (e, s)
# state entry = time of the call that carried the event (it returns the new state)
entry = [None] * 8
for t, e_s in [(x[0], x[1]) for x in fsm]:
    entry[e_s] = t
for t, s in [(x[0], x[1]) for x in fsm][1:]:
    entry[s] = max(0, t - 0.0025)
entry[0] = fsm[0][0]
END = 22.0

def cur(series, t):
    r = [v for a, v in series if a <= t]
    return r[-1] if r else None

def state_at(t):
    r = [s for a, s in fsm if a <= t]
    return r[-1] if r else None

def mod_health(t):
    bad = []
    for m, rows in health.items():
        r = [x for x in rows if x[0] <= t]
        if r and r[-1][2] >= 1: bad.append(MODS[m])
    return bad

def draw(c, t):
    c.header("Canard board firmware (STM32H723) flying a stand-in launch", "team firmware, unmodified. Sensor values are what the firmware sent on its CAN bus; the FSM state is read from its flight_phase_update_state calls.", t)
    st = state_at(t); a = cur(acc, t); g = cur(gyr, t); cm = cur(cmd, t); sv = cur(servo, t)
    tw = 196; X = [24 + i * (tw + 12) for i in range(6)]
    c.panel(X[0], 90, tw, 92); c.text(X[0] + 14, 98, "FSM STATE", 15, DIM, True)
    c.text(X[0] + 14, 164, STATES[st] if st is not None else "--", 24, GREEN if st is not None else DIM, True, False, "ld")
    c.tile(X[1], 90, tw, 92, "ACCEL Y (IMU)", f"{a[1]:.1f}" if a else "--", "m/s2", CYAN)
    c.tile(X[2], 90, tw, 92, "GYRO Y (IMU)", f"{g[1]:.1f}" if g else "--", "deg/s", CYAN)
    c.tile(X[3], 90, tw, 92, "CMD ANGLE (CTRL)", f"{cm:.1f}" if cm is not None else "no cmd", "deg" if cm is not None else "", AMBER)
    sp = (sv[1] if isinstance(sv, tuple) else None)
    sv_now = [p for tt, p in servo if tt <= t]
    c.tile(X[4], 90, tw, 92, "SERVO POSITION", f"{sv_now[-1] + 0.0:.1f}".replace("-0.0", "0.0") if sv_now else "--", "deg", AMBER)
    c.tile(X[5], 90, tw, 92, "MODULES IN ERROR", str(len(mod_health(t))), f"of {len(health)} reporting", RED if mod_health(t) else GREEN)
    c.panel(24, 196, 1232, 112, "Flight-phase state machine (fsm_state_t): when each state was entered")
    c.chain(40, 226, 1200, 70, STATES, st, [x if (x is not None and x <= t) else None for x in entry], [BLUE, BLUE, BLUE, AMBER, GREEN, GREEN, GREEN, RED], sz=15)
    c.panel(24, 322, 470, 360, "CAN bus, decoded (rocketlib packet_format.h)")
    rows = [("LSM6DSV32X accel", "0x0D/2", f"x {a[0]:.1f} y {a[1]:.1f} z {a[2]:.1f} m/s2" if a else None),
            ("LSM6DSV32X gyro", "0x0D/3", f"x {g[0]:.0f} y {g[1]:.0f} z {g[2]:.0f} deg/s" if g else None),
            ("Controller cmd angle", "0x0A/35", f"{cm:.1f} deg" if cm is not None else None),
            ("Servo angle", "0x0B/3B", f"{cur(srv_a, t):.1f} deg" if cur(srv_a, t) is not None else None)]
    for i, (nm, mid, val) in enumerate(rows):
        y = 354 + i * 40
        c.text(38, y, nm, 15, FG if val else DIM, True); c.text(456, y + 1, mid, 13, DIM, False, True, "ra")
        c.text(38, y + 19, val or "not sent yet", 14, AMBER if val else DIM, False, True)
    bad = mod_health(t)
    c.text(38, 520, "Module health (0x18 FIRMWARE_ERROR)", 15, FG, True)
    c.text(38, 540, f"{len(bad)} modules report severity ERROR:" if bad else "no reports yet", 14, RED if bad else DIM, False, True)
    for k in range(0, len(bad), 3):
        c.text(38, 560 + (k // 3) * 20, ", ".join(bad[k:k + 3]), 13, DIM, False, True)
    c.strip(506, 322, 372, 168, [(a_, v[1]) for a_, v in acc], t, END, -10, 50, "IMU accel y (m/s2)", CYAN, "")
    c.strip(890, 322, 366, 168, [(a_, p) for a_, p in servo[::10]], t, END, -25, 25, "servo position (deg)", AMBER, "", step=False, ticks=5)
    c.log(506, 502, 750, 180, log, t, "firmware FSM trace (--trace-symbol flight_phase_update_state)", sz=14)
    c.footer(VER, SPEED, ".  Stimulus is a stand-in profile (5 g boost from 3 s), not a flight log.")

SPEED = END / 15
print("frames", render(draw, END, SPEED, "sim/screenshots/signals.gif"))
