# Moving-arm allocation: review summary

Firmware change: [861f26f26e — full code diff](https://github.com/DariusGiannoli/PX4-Autopilot/commit/861f26f26e4f5cc7b6b379e7bae6b09f42239967).

Publisher addition: [2f1f92457c — code and tests](https://github.com/DariusGiannoli/PX4-Autopilot/commit/2f1f92457ca9774c18638f57817405582b041260).

## Objective and implemented behavior

Update PX4's motor effectiveness matrix from commanded horizontal arm angles,
while all four rotor thrust axes remain vertical. The listener lives inside the
existing control allocator; no separate scheduled listener module is needed.
Select it with `CA_AIRFRAME=13` and `CA_ROTOR_COUNT=4`.

```text
MAVLink DO_SET_ACTUATOR -> calibrated morphing_arm_publisher
    -> morphing_arm_state: timestamp + four commanded angles
    -> ActuatorEffectivenessMorphingQuad: validate angles and compute rotor positions
    -> updated effectiveness matrix B
    -> existing PX4 allocation, motor limits and outputs
```

The disarmed console command supplies simulated angles for bench testing. The new
`morphing_arm_publisher` observes actual MAVLink commands; existing PX4 peripheral
outputs perform servo actuation. Both paths publish commanded, not measured, angles.

## Changes to review

All paths below are inside `PX4-Autopilot/`.

| File / area | Change |
| --- | --- |
| `msg/MorphingArmState.msg` | Internal interface: radians, FR/RR/RL/FL order, +/-30 degrees from CAD neutral. |
| `src/modules/control_allocator/ActuatorEffectiveness/MorphingQuadGeometry.hpp` | CAD-derived hinge/arm dimensions, coordinate conversion, rotor positions and 6x4 effectiveness matrix. |
| `src/modules/control_allocator/ActuatorEffectiveness/ActuatorEffectivenessMorphingQuad.*` | Subscribe, reject invalid/old inputs, retain the last valid geometry, and rebuild on angle changes (ordinary updates at most 100 Hz). |
| `src/modules/control_allocator/ControlAllocator.*`, `ControlAllocation/`, `ActuatorEffectiveness.hpp` | Register source 13; add optional normalization/update/threshold/diagnostic hooks; move large update buffers off the shared work-queue stack. Existing providers retain default policies. |
| `src/modules/control_allocator/module.yaml` | Add source selection and fixed CoG parameters `CA_MORPH_COGX/Y/Z`. Reuse existing rotor CT/KM coefficients. |
| `msg/MorphingGeometry.msg`, `msg/MorphingAllocationMatrix.msg`, logger | Record accepted angles, calculated geometry and the matrix actually assigned after failure masking. |
| `Tools/morphing_quad_geometry/`, allocator tests | Reference geometry checks, integration tests, ARM stack check and detailed documentation. |

For each motor, with its lever arm measured from the configured CoG:

```text
position = hinge + Rz(-angle) * neutral_offset
B_column = CT * [-lever_y, lever_x, KM, 0, 0, -1]
```

Matrix columns use PX4 motor order FR/RL/FL/RR, which differs from message order.
Neutral geometry supplies a fixed normalization reference; current geometry is
used for allocation. `B` is the effectiveness matrix, not the normalized inverse.

## Verification

- Host geometry: 1,081 CAD/reference comparisons passed, including +/-30 degree
  combinations and random configurations; maximum matrix-entry error 2.42e-7.
- Five targeted PX4 test suites passed, including the new publisher-to-matrix test: provider, actual allocator integration,
  pseudoinverse, and sequential desaturation.
- NxtPX4v2 ARM target `hkust_nxt-dual_default` built successfully with GCC 9.3.1.
  Static checked stack paths use at most 2,296 of 3,150 bytes, with 512 bytes reserved
  for uncounted calls. This is not a complete runtime stack measurement.
- User-reported FC console test on 2026-09-10: `CA_AIRFRAME=13` persisted across
  reboot; the listener displayed a front-right 10 degree command as 0.17453 rad.
  Motor 0 roll/pitch coefficients changed from -0.44094 / 0.44152 to
  -0.39194 / 0.48254. The other motors were unchanged. Publishing four zeros
  restored the neutral matrix and the listener displayed zero angles.

To repeat the console test with props removed and the FC disarmed, run separately:

```sh
control_allocator morphing_test 10 0 0 0
listener morphing_arm_state -n 1
control_allocator status
control_allocator morphing_test 0 0 0 0
listener morphing_arm_state -n 1
control_allocator status
```

## Remaining work and limits

`morphing_arm_publisher` now subscribes to `vehicle_command`, filters
`DO_SET_ACTUATOR` group 0 and maps parameters 1–4 to FR/RR/RL/FL. It retains
NaN channels, validates complete commands, interpolates measured per-arm
calibration, and publishes finite radians with increasing timestamps.
All four targets must be known before publication. Queue loss or command timeout
invalidates remembered channel availability and requires all four targets again.

The module is included in the NxtPX4v2 build and starts at boot only when
`MORPH_PUB_EN=1`. It refuses startup unless source 13, four rotors and valid
calibration are configured. Calibration uses measured arm angles at normalized
-1, 0 and +1, with two linear interpolation segments. Default calibration is
invalid; no measured data is supplied for this aircraft. Restart the publisher
after changing calibration or timeout. The console simulator is blocked while
the production publisher is enabled.

The bridge observes the MAVLink peripheral-output path; RC AUX passthrough uses
`manual_control_setpoint` and remains unsupported. It does not confirm actual PWM
output, servo motion or feedback, and it does not synchronize manual overrides,
output tests or output failsafes. Valid output assignments and a sender following
the documented command interface are required. It does not intercept or veto
commands reaching the output driver.

`morphing_publisher_status` reports initialization, stale commands, rejected inputs
and command-queue loss. The publisher warns after `MORPH_PUB_TMO` seconds without
an accepted arm update. This is diagnostic protection, not an arming interlock or
an independent watchdog: the allocator still holds its last accepted geometry if
the publisher is stale or stops, and servo travel time is not modeled.

Host tests verify startup/calibration rejection, reversed curves, NaN/partial
updates, invalid and replayed commands, timestamps, timeout recovery, queue loss,
and real uORB commands through to the expected front-right effectiveness matrix.
The new publisher has not been tested on physical servo hardware or in flight.

See [publisher setup, calibration and verification](PX4-Autopilot/src/modules/morphing_arm_publisher/README.md).

Physical geometry/CoG and motor rotation signs still need verification. The tested
configuration had three positive yaw coefficients and one negative; these must be
checked against the actual motor directions. The reported `sysinit: fopen failed`
warning and earlier USB disconnect reports have not been fully diagnosed.

See [the detailed implementation and test guide](PX4-Autopilot/Tools/morphing_quad_geometry/README.md).

## Combined MAVLink bench test

A [USB sender and test guide](tools/README.md) now exercises real
`DO_SET_ACTUATOR` commands through the servo output path and publisher, recording
the arm messages and matrices. It can temporarily install explicitly assumed
calibration and enable non-motor outputs while disarmed, then restores the original
parameters. Offline success/failure cleanup tests pass; the combined hardware
run is still pending. No additional FC firmware change is required for this sender.
