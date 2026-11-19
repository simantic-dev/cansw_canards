# One synthetic flight every sensor peer samples, keyed on the sim's virtual time (ctx.now_us).
# 1-D vertical kinematics: on the pad until LAUNCH_S (the ground peer opens the ox injector
# valve then, which the FSM takes as launch), a constant-thrust burn, ballistic coast through
# apogee, free fall back to the ground. Drag is left out: the FSM only needs |accel| >= 30 m/s^2
# for five samples (flight_phase.c), then 7 s (ACT_DELAY_MS) before canards may actuate.
G = 9.80665
LAUNCH_S = 2.5
BURN_S = 3.0
THRUST_MPS2 = 60.0  # net upward acceleration during the burn (proper accel = THRUST + G)


def kin(t_s):
    """(altitude m, vertical velocity m/s, net vertical acceleration m/s^2) at t_s."""
    if t_s < LAUNCH_S:
        return 0.0, 0.0, 0.0
    t = t_s - LAUNCH_S
    if t < BURN_S:
        return 0.5 * THRUST_MPS2 * t * t, THRUST_MPS2 * t, THRUST_MPS2
    tb = t - BURN_S
    v0 = THRUST_MPS2 * BURN_S
    alt = 0.5 * THRUST_MPS2 * BURN_S * BURN_S + v0 * tb - 0.5 * G * tb * tb
    if alt <= 0.0:
        return 0.0, 0.0, 0.0  # landed
    return alt, v0 - G * tb, -G


def proper_accel_up(t_s):
    """What an accelerometer reads along the rocket's up axis, m/s^2 (gravity included)."""
    alt, _, a = kin(t_s)
    if t_s < LAUNCH_S or (alt <= 0.0 and t_s > LAUNCH_S + BURN_S):
        return G  # resting on the pad or the ground
    return a + G  # burn: THRUST + G; coast and fall: 0 (free fall)


def gyro_radps(t_s):
    """A slow roll while airborne, nothing on the ground."""
    alt, _, _ = kin(t_s)
    return (0.0, 0.0, 0.2) if alt > 0.0 else (0.0, 0.0, 0.0)


def pressure_pa(t_s):
    """ISA barometric pressure at the current altitude."""
    alt, _, _ = kin(t_s)
    return 101325.0 * (1.0 - 2.25577e-5 * alt) ** 5.25588


def temperature_c(t_s):
    alt, _, _ = kin(t_s)
    return 15.0 - 0.0065 * alt


MAG_UT = (20.0, 0.0, 45.0)  # a fixed field; the peers rotate nothing


if __name__ == "__main__":
    # self-check: pad, burn, coast, apogee, landing
    assert proper_accel_up(0.0) == G and kin(1.0) == (0.0, 0.0, 0.0)
    assert abs(proper_accel_up(LAUNCH_S + 1.0) - (THRUST_MPS2 + G)) < 1e-9
    assert proper_accel_up(LAUNCH_S + BURN_S + 1.0) == 0.0
    apogee = max(kin(LAUNCH_S + BURN_S + t / 10.0)[0] for t in range(0, 400))
    assert 1500 < apogee < 2500, apogee
    assert kin(LAUNCH_S + 60.0) == (0.0, 0.0, 0.0) and proper_accel_up(LAUNCH_S + 60.0) == G
    assert 80000 < pressure_pa(LAUNCH_S + BURN_S + 10.0) < 101325
    print("flight.py ok, apogee %.0f m" % apogee)
