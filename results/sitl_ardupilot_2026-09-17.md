# EKF3 fuses our ExternalNav fixes. Measured, not assumed.

17 Sept 2026. ArduPilot master built from source for SITL (`./waf configure
--board sitl && ./waf copter`, 1429 targets, 7m30s), ArduCopter quad model,
`--speedup 5`. The runtime's own `output_layer/fcout.py` and
`scripts/check_extnav.py` ran against it unmodified.

## check_extnav.py: all four stages pass

    1/4  connecting            system 1, autopilot 3, type 2
    2/4  parameters            12 of 12 ok
    3/4  position echo         -35.3632621, 149.1652374
    4/4  ODOMETRY at 4 Hz      60 messages, sigma 5.0 m

The parameter block in `configs/flight.yaml` is correct as written. Every one
of `AHRS_EKF_TYPE`, `VISO_TYPE`, `VISO_POS_X/Y/Z`, `VISO_DELAY_MS`,
`VISO_QUAL_MIN`, `EK3_SRC1_POSXY/POSZ/VELXY/VELZ/YAW` read back as intended
from a real autopilot rather than from the documentation.

**Step 3 fails with GPS disabled, and that is not a bug.** The first run set
`GPS1_TYPE 0` in the defaults, so the EKF never got an origin and the check
stopped exactly where it should: `no GLOBAL_POSITION_INT with a valid fix`. An
ExternalNav position is local and needs an origin to be local *to*. Boot with
GPS, take the origin, then remove GPS.

## watch_extnav.py: the part check_extnav could not answer

`check_extnav.py` proves the message is well formed and then tells you to go and
watch Mission Planner. `scripts/watch_extnav.py` closes that loop with no human
in it, by taking GPS away and injecting a position the vehicle is not at:

    origin -35.3632621, 149.1652374
    GPS1_TYPE set to 0 -- vehicle is now GNSS-denied
    injecting a position 20 m north, sigma 2.0 m, 4.0 Hz for 30s

        t   drift N (m)   ekf flags
      0.0          0.00  0x00000000
      5.0         19.80  0x00000000
     10.0         19.78  0x00000000
     15.0         19.77  0x00000000
     20.0         19.78  0x00000000
     25.0         19.79  0x00000000

    sent 114 messages, final drift 19.81 m of 20 m injected

**EKF3 walked to 19.81 m of the 20 m injected, inside 5 seconds, and held it
with no GPS.** The write path is fused rather than merely accepted. That is the
one thing three MAVLink bugs and a parameter block could each have broken
silently, and it is now measured.

Two honest limits on this run.

- **The `ekf flags` column is not evidence.** It reads 0x00000000 throughout
  because the EXTENDED_STATUS stream request did not deliver `EKF_STATUS_REPORT`
  on this build, so the variable never left its initial value. The drift column
  is the result; the flags column is decoration that did not work. Either fix
  the stream request or drop the column.
- **SITL is not a Pixhawk.** It shares ArduPilot's estimator and its MAVLink
  handler, which is what was under test, and shares neither the serial link, the
  timing, nor the camera. `VISO_DELAY_MS` is 50 here because that is NGPS's
  number; ours is 45.7 ms median capture-to-matcher on the Xavier plus the
  MAVLink hop, and it has never been measured end to end.

## Reproduce

    ./waf configure --board sitl && ./waf copter
    build/sitl/bin/arducopter -S -I0 --model quad --speedup 5 \
        --defaults Tools/autotest/default_params/copter.parm,extnav.parm --sysid 1
    python scripts/check_extnav.py --endpoint tcp:127.0.0.1:5760
    python scripts/watch_extnav.py  --endpoint tcp:127.0.0.1:5760 --offset-m 20

`extnav.parm` is the block in the `configs/flight.yaml` header, minus
`GPS1_TYPE`, which must stay enabled long enough for the EKF to take an origin.
