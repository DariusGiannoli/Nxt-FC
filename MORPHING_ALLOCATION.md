# Moving-arm allocation: review summary

Firmware change: [861f26f26e — full code diff](https://github.com/DariusGiannoli/PX4-Autopilot/commit/861f26f26e4f5cc7b6b379e7bae6b09f42239967).

## Objective and implemented behavior

Update PX4's motor effectiveness matrix from commanded horizontal arm angles,
while all four rotor thrust axes remain vertical. The listener lives inside the
existing control allocator; no separate scheduled listener module is needed.
Select it with `CA_AIRFRAME=13` and `CA_ROTOR_COUNT=4`.

```text
Servo-driving code (still to be added)
    -> morphing_arm_state: timestamp + four commanded angles
    -> ActuatorEffectivenessMorphingQuad: validate angles and compute rotor positions
    -> updated effectiveness matrix B
    -> existing PX4 allocation, motor limits and outputs
```

The included disarmed console command substitutes for the missing publisher during
bench testing. It does not move a servo.

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
- Four targeted PX4 test suites passed: provider, actual allocator integration,
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

No physical servo driver, PWM calibration, laptop/radio interface or flight test
is included. A future servo-driving implementation must publish its accepted arm
targets using this interface. With no position feedback, angles are assumed
commanded positions; servo travel time is not modeled. The provider starts neutral
and then holds the last valid angles if publication stops.

Physical geometry/CoG and motor rotation signs still need verification. The tested
configuration had three positive yaw coefficients and one negative; these must be
checked against the actual motor directions. The reported `sysinit: fopen failed`
warning and earlier USB disconnect reports have not been fully diagnosed.

See [the detailed implementation and test guide](PX4-Autopilot/Tools/morphing_quad_geometry/README.md).
