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

The console test publishes `morphing_arm_state`, but no production servo-command
publisher is connected. Before the first valid message the allocator uses neutral;
afterward it holds the last valid geometry, including any simulated test angles.

Existing PX4 output support can drive suitably configured servos without informing
the morphing allocator. The two relevant paths are distinct:

| Input | Existing output path | Missing connection |
| --- | --- | --- |
| MAVLink `DO_SET_ACTUATOR` | `vehicle_command` -> `FunctionActuatorSet` -> assigned peripheral output | Convert the accepted commands into calibrated arm angles and publish them. |
| RC AUX passthrough | `manual_control_setpoint` -> `FunctionManualRC` -> assigned RC output | Observe the selected AUX values or a shared servo-command stage; a `vehicle_command` subscriber alone cannot see these updates. |

For the proposed MAVLink bridge, use `VEHICLE_CMD_DO_SET_ACTUATOR` with group
`param7=0`, and explicitly assign `param1` through `param4` to FR/RR/RL/FL.
Retain the previous channel target for NaN fields. Initialize all four targets
from an established arm state before publishing partial updates; an unknown
position must not silently become a claimed neutral position. The existing
`FunctionActuatorSet` retains its values for all nonfinite fields and rounds
`param7` when selecting a group, so input acceptance must be coordinated with
that output path.

Convert normalized commands with a **measured per-arm calibration curve** that
includes output scaling/reversal, linkage and mounting zero. Calibration data is
not yet available. Published values must be finite radians within the firmware's
exact +/-pi/6 limit (approximately +/-0.523598776), with a strictly increasing FC
monotonic timestamp. One invalid angle rejects the whole message. Do not merely
clamp the published angle while allowing the servo to receive a different target.
Output assignments and command limits must match what the bridge reports.

Verify with `listener morphing_geometry -n 1`: `input_result=1` indicates an
accepted input, and `rotor_position` should follow the calibrated angles. Check
`listener morphing_allocation_matrix -n 1` for the assigned matrix. Changed angles
should produce a new matrix generation; an unchanged accepted target need not.

These are commanded positions, not measured feedback. Servo travel time is not
modeled, so during movement the matrix can lead the physical geometry. There is
no stale-input timeout, neutral fallback or stale-input warning in this provider.
Production integration needs an explicit policy for initialization and loss of
commands/publication, coordinated with actual servo behavior. Physical servo
movement, calibration and flight testing remain outstanding.

Physical geometry/CoG and motor rotation signs still need verification. The tested
configuration had three positive yaw coefficients and one negative; these must be
checked against the actual motor directions. The reported `sysinit: fopen failed`
warning and earlier USB disconnect reports have not been fully diagnosed.

See [the detailed implementation and test guide](PX4-Autopilot/Tools/morphing_quad_geometry/README.md).
