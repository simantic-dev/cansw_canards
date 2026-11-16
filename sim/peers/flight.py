# Shared stimulus for the sensor peers: a launch-and-spin profile against virtual seconds.
# The numbers are a made-up rocket, not a flight log: the firmware reads whatever the peers say.
#   pad until T_LAUNCH, then BOOST_G axial acceleration for BURN_S, then coast at -1 g drag-free (0 g specific force)
#   roll rate: ROLL_DPS while boosting and coasting, 0 on the pad.
T_LAUNCH = 3.0
BURN_S = 3.0
BOOST_G = 5.0
ROLL_DPS = 40.0


def accel_g(t):
    """Specific force along the rocket axis (x), g."""
    if t < T_LAUNCH:
        return 1.0
    if t < T_LAUNCH + BURN_S:
        return BOOST_G
    return 0.0


def roll_dps(t):
    return 0.0 if t < T_LAUNCH else ROLL_DPS
