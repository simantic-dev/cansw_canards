# Adds `pio run -e <env> -t sim`: boots this env's firmware.elf in pyrite with the peers in sim/
# on its buses (see sim/README.md). Needs the sim binary: $PYRITE_SIM, or `sim` on PATH.
Import('env')

import os
import shutil
import subprocess
import tempfile

SIM_DIR = Dir('#/sim').abspath
# sim exits 1 unless the UART output holds each of these (sim/canards.yaml peers' log lines)
MARKERS = ["[motor] SET_POS"]


def run_sim(target, source, env):
    sim = os.environ.get("PYRITE_SIM") or shutil.which("sim")
    if not sim:
        print("sim: set PYRITE_SIM to the pyrite sim binary (see sim/README.md)")
        Exit(1)
    elf = env.subst("$PROGPATH")  # the build's firmware.elf
    # copy sim/ out of the checkout so the peers' __pycache__ does not land in the repo
    work = tempfile.mkdtemp(prefix="canards-sim-")
    try:
        shutil.copytree(SIM_DIR, work, dirs_exist_ok=True)
        with open(os.path.join(SIM_DIR, "canards.yaml")) as f:
            text = f.read().replace('"@ELF@"', '"%s"' % os.path.abspath(elf))
        with open(os.path.join(work, "canards.yaml"), "w") as f:
            f.write(text)
        cmd = [sim, "run", "--scenario", os.path.join(work, "canards.yaml"), "--timeout", "12"]
        # the peers' log lines are not UART bytes, so `sim --expect` cannot see them: check the output here
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    out = proc.stdout.decode(errors="replace")
    print(out, end="")
    missing = [m for m in MARKERS if m not in out]
    if proc.returncode or missing:
        print("sim: markers missing: %s" % missing)
        Exit(1)
    print("sim: boot markers present")


env.AddCustomTarget(
    "sim",
    "$PROGPATH",  # builds the firmware first, so `-t sim` alone is enough
    run_sim,
    title="Sim",
    description="Boot the firmware on pyrite (simulated STM32H733) with the sensor and CAN peers"
)
