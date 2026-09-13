# USB MAVLink bench test

`morphing_mavlink_bench.py` exercises the production path on the connected FC:

```text
DO_SET_ACTUATOR -> peripheral outputs + calibrated arm publisher -> matrix
```

It requires the publisher firmware, source 13, four rotors, and the verified
MAIN mapping 5=302, 6=304, 7=303, 8=301. MAIN 1–4 must retain motor functions
101,104,103,102. It refuses an already running/enabled publisher.

Remove all propellers and keep the FC disarmed. Close QGroundControl so it does
not compete for the USB serial port. With `pymavlink` and `pyserial` installed:

```sh
python3 tools/morphing_mavlink_bench.py --device /dev/cu.usbmodem01 --props-removed --assume-linear-bench --enable-prearm-bench
```

The explicit `--assume-linear-bench` flag temporarily installs -30/0/+30 degree
curves. These are **software test assumptions**, not measured servo calibration.
It commands only 0 and +0.05 (about 1500 and 1525 us with the verified PWM range).
The publisher reports +0.02618 rad (1.5 degrees) for the front-right test. It does
not command a physical 10 degree movement. Three other channels stay at command
centre, which is not necessarily measured mechanical neutral.

The explicit `--enable-prearm-bench` flag temporarily sets `COM_PREARM_MODE=2`.
This enables non-motor outputs while still disarmed; it never sends an arming
command. Without that flag the FC must already be prearmed. No PWM assignments,
limits or safety-switch settings are changed. Keep the FC disarmed throughout.

The script primes centre targets, enables the publisher, streams full four-arm
commands at 10 Hz, and captures `morphing_arm_state`, `morphing_geometry`,
`morphing_allocation_matrix` and allocator status. It sends parameters 5/6 as NaN.
It returns to centre, stops its publisher and restores original calibration,
enable and prearm parameter values, with readback checks.

Logs and a parameter recovery record are written under `logs/morphing-bench/`
(ignored by Git). PX4 may autosave parameter changes even though this script does
not call `param save`. A normal exit restores them; USB loss, power loss or a
forced process kill can prevent cleanup. Do not disconnect until completion.
If cleanup reports failure, keep disarmed, reconnect QGroundControl, set
`MORPH_PUB_EN=0`, stop the publisher, and restore the exact values from the recovery
JSON with `param set NAME VALUE`, verifying each. The script does not attempt
physical recovery after an unexpected armed heartbeat.

This is a bench integration test, not flight validation. Normal disarmed output
test commands bypass the publisher and cannot substitute for this test. Reopen
QGroundControl after the script exits. No new firmware flash is needed if the
publisher firmware is already installed.

Offline tests (no serial access):

```sh
python3 tools/test_morphing_mavlink_bench.py
```
