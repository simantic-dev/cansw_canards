# Animates sim/out/{can3,servo}.csv (the run's own captures) into sim/screenshots/signals.gif. Needs matplotlib, pillow.
# usage: python3 sim/make_gif.py "caption tag"
import csv, sys, io
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
OFF = 32768
S = {"acc": [], "gyr": [], "cmd": []}
for r in csv.DictReader(open("sim/out/can3.csv")):
    i, d, t = int(r["id"], 16), bytes.fromhex(r["data"]), float(r["t_s"])
    typ, meta = (i >> 20) & 0x7F, i & 0xFF
    if typ == 0x0D and meta in (2, 3):
        v = [(int.from_bytes(d[2 + 2 * k:4 + 2 * k], "big") - OFF) / 100 for k in range(3)]
        S["acc" if meta == 2 else "gyr"].append((t, v[1] * (1 if meta == 2 else 57.29578)))
    elif typ == 0x0A and meta == 0x35:
        S["cmd"].append((t, (int.from_bytes(d[2:4], "big") - OFF) / 1000 * 57.29578))
servo = [(float(r["t_s"]), float(r["pos_deg"])) for r in csv.DictReader(open("sim/out/servo.csv"))][::20]
end = max(servo[-1][0], S["acc"][-1][0]); frames = []
lim = {k: (min(v for _, v in S[k]) - 1, max(v for _, v in S[k]) + 1) for k in S}
for tc in [end * k / 44 for k in range(1, 45)]:
    fig, ax = plt.subplots(3, 1, figsize=(7, 5.4), dpi=70, sharex=True)
    for a, k, c, lab in ((ax[0], "acc", "C0", "IMU accel y, m/s^2"), (ax[1], "gyr", "C2", "IMU gyro y, deg/s"), (ax[2], "cmd", "C3", "controller command, deg")):
        a.plot([t for t, _ in S[k] if t <= tc], [v for t, v in S[k] if t <= tc], c=c); a.set_ylabel(lab, fontsize=7); a.set_ylim(*lim[k]); a.grid(alpha=.3)
    ax[2].plot([t for t, _ in servo if t <= tc], [v for t, v in servo if t <= tc], c="k", lw=.8)
    ax[2].set_xlim(0, end); ax[2].set_xlabel("virtual time, s")
    ax[0].set_title(f"Canard board firmware on simulated H723, t={tc:.1f} s virtual, {sys.argv[1]}", fontsize=8)
    fig.tight_layout(); b = io.BytesIO(); fig.savefig(b, format="png"); plt.close(fig)
    frames.append(Image.open(b).convert("RGB").quantize(colors=64, dither=Image.Dither.NONE))
frames[0].save("sim/screenshots/signals.gif", save_all=True, append_images=frames[1:], duration=[110] * 43 + [1500], loop=0, optimize=True)
