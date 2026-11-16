# Renders sim/screenshots/signals.png from sim/out/{can3,servo}.csv (the run's own captures). Needs matplotlib.
# usage: python3 sim/plot.py "caption text for the figure title"
import csv, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OFF = 32768
S = {"acc": [], "gyr": [], "cmd": []}
for r in csv.DictReader(open("sim/out/can3.csv")):
    i, d, t = int(r["id"], 16), bytes.fromhex(r["data"]), float(r["t_s"])
    typ, meta = (i >> 20) & 0x7F, i & 0xFF
    if typ == 0x0D and meta in (2, 3):                       # 3D analog16: LSM6DSV32X accel / gyro, scale 100
        v = [(int.from_bytes(d[2 + 2 * k:4 + 2 * k], "big") - OFF) / 100 for k in range(3)]
        S["acc" if meta == 2 else "gyr"].append((t, v))
    elif typ == 0x0A and meta == 0x35:                       # controller command angle, scale 1000
        S["cmd"].append((t, (int.from_bytes(d[2:4], "big") - OFF) / 1000))
servo = list(csv.DictReader(open("sim/out/servo.csv")))
fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
ax[0].plot([t for t, _ in S["acc"]], [v[1] for _, v in S["acc"]], label="IMU accel, board y axis (firmware telemetry)")
ax[0].set_ylabel("m/s^2"); ax[0].legend(loc="center right", fontsize=8)
ax[1].plot([t for t, _ in S["gyr"]], [v[1] * 57.29578 for _, v in S["gyr"]], color="tab:green", label="IMU gyro, board y axis (firmware telemetry, rad/s -> deg/s)")
ax[1].set_ylabel("deg/s"); ax[1].legend(loc="center right", fontsize=8)
ax[2].plot([t for t, _ in S["cmd"]], [v * 57.29578 for _, v in S["cmd"]], color="tab:red", label="controller command (CAN 0x35, rad->deg)")
ax[2].plot([float(r["t_s"]) for r in servo[::20]], [float(r["pos_deg"]) for r in servo[::20]], color="k", lw=0.8, label="servo stand-in position")
ax[2].set_ylabel("deg"); ax[2].set_xlabel("virtual time (s)"); ax[2].legend(loc="center right", fontsize=8)
for a in ax:
    a.grid(alpha=0.3)
fig.suptitle(sys.argv[1] if len(sys.argv) > 1 else "", fontsize=9)
fig.tight_layout()
import os
os.makedirs("sim/screenshots", exist_ok=True)
fig.savefig("sim/screenshots/signals.png", dpi=90)
