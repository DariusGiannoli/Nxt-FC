#!/usr/bin/env python3
"""Disarmed USB bench test: DO_SET_ACTUATOR -> arm publisher -> allocation matrix.

Never arms the aircraft or changes PWM settings. Prearm changes require an explicit bench flag.
Use --assume-linear-bench only for a software-path test with unmeasured angles.
"""
import argparse
import json
import math
from pathlib import Path
import re
import sys
import time


def parse_parameters(output):
    values = {}
    for line in output.splitlines():
        match = re.search(r"\b([A-Z][A-Z0-9_]*)\s+\[[^\]]+\]\s*:\s*([-+0-9.eE]+)\s*$", line)
        if match:
            values[match[1]] = float(match[2])
    return values


def assumed_calibration():
    return {f"MARM_{arm}_{point}": angle for arm in ("FR", "RR", "RL", "FL")
            for point, angle in (("NEG", -30.0), ("ZERO", 0.0), ("POS", 30.0))}


def require_disarmed_prearmed(output, allow_not_prearmed=False):
    if not re.search(r"(?m)^\s*armed:\s*False\s*$", output, re.I):
        raise RuntimeError("Cannot confirm disarmed state; stopping without changing parameters.")
    if not allow_not_prearmed and not re.search(r"(?m)^\s*prearmed:\s*True\s*$", output, re.I):
        raise RuntimeError("FC is disarmed but NOT prearmed. Normal peripheral outputs are gated. "
                           "No safety/prearm setting was changed. Share the actuator_armed output.")
    for flag in ("lockdown", "manual_lockdown", "force_failsafe", "in_esc_calibration_mode"):
        if not re.search(rf"(?m)^\s*{flag}:\s*False\s*$", output, re.I):
            raise RuntimeError(f"Cannot confirm {flag}=false; test stopped.")


class Link:
    def __init__(self, device, log):
        from pymavlink import mavutil
        self.util = mavutil
        self.log = log
        self.link = mavutil.mavlink_connection(device, baud=115200, source_system=254,
                                              source_component=190, autoreconnect=False)
        self.system = None
        self.component = None
        self.heartbeat_at = 0.0
        self.next_gcs = 0.0
        self.next_target = 0.0
        self.targets = None
        self.console = ""
        self.unsafe = False
        self.cleanup = False
        try:
            deadline = time.monotonic() + 10
            while self.system is None and time.monotonic() < deadline:
                self.pump()
            if self.system is None:
                raise RuntimeError("No PX4 heartbeat. Close QGroundControl and check the USB device.")
            self.shell("")
        except BaseException:
            self.link.close()
            raise

    def note(self, text):
        print(text, flush=True)
        self.log.write(text + "\n")
        self.log.flush()

    def pump(self):
        now = time.monotonic()
        mav = self.util.mavlink
        if now >= self.next_gcs:
            self.link.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.next_gcs = now + 1
        message = self.link.recv_match(blocking=True, timeout=0.02)
        if message:
            if message.get_type() == "HEARTBEAT" and message.autopilot == mav.MAV_AUTOPILOT_PX4:
                if self.system is None:
                    self.system, self.component = message.get_srcSystem(), message.get_srcComponent()
                if (message.get_srcSystem(), message.get_srcComponent()) == (self.system, self.component):
                    self.heartbeat_at = now
                    if message.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
                        self.unsafe = True
                        self.targets = None
            elif message.get_type() == "SERIAL_CONTROL" and message.get_srcSystem() == self.system:
                if message.device == 10:
                    self.console += bytes(message.data[:message.count]).decode("utf-8", "replace")
        if self.unsafe and not self.cleanup:
            raise RuntimeError("Aircraft became armed: servo-command streaming stopped.")
        if self.system and now - self.heartbeat_at > 3:
            self.targets = None
            raise RuntimeError("PX4 heartbeat lost: command streaming stopped; recovery may be needed.")
        if self.targets is not None and not self.unsafe and now >= self.next_target:
            self.link.mav.command_long_send(self.system, self.component, mav.MAV_CMD_DO_SET_ACTUATOR,
                                            0, *self.targets, math.nan, math.nan, 0)
            self.next_target = now + 0.1

    def shell(self, command, timeout=8):
        self.console = ""
        data = (command + "\n").encode()
        flags = (self.util.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                 self.util.mavlink.SERIAL_CONTROL_FLAG_RESPOND)
        for index in range(0, len(data), 70):
            chunk = data[index:index + 70]
            self.link.mav.serial_control_send(10, flags, 0, 0, len(chunk),
                                              list(chunk) + [0] * (70 - len(chunk)))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if "nsh>" in self.console:
                output = self.console
                self.note(f"\n$ {command}\n{output}")
                return output
        raise RuntimeError(f"Console timed out: {command}")

    def set_param(self, name, value):
        output = self.shell(f"param set {name} {value:.9g}")
        if "not found" in output.lower() or "ERROR" in output:
            raise RuntimeError(f"Parameter write failed: {name}")
        actual = parse_parameters(self.shell(f"param show {name}")).get(name)
        if actual is None or not math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-6):
            raise RuntimeError(f"Parameter readback failed: {name}")

    def dwell(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump()

    def close(self):
        self.targets = None
        try:
            self.link.mav.serial_control_send(10, 0, 0, 0, 0, [0] * 70)
        finally:
            self.link.close()


def run(args):
    stamp = time.strftime("%Y%m%d-%H%M%S")
    folder = Path(args.log_dir).resolve()
    folder.mkdir(parents=True, exist_ok=True)
    backup_path = folder / f"morphing-bench-{stamp}-restore.json"
    with (folder / f"morphing-bench-{stamp}.log").open("w") as log:
        link = Link(args.device, log)
        backup = {}
        mutated = False
        started = False
        try:
            require_disarmed_prearmed(link.shell("listener actuator_armed -n 1"), args.enable_prearm_bench)
            assignments = parse_parameters(link.shell("param show PWM_MAIN_FUNC*"))
            expected = {"PWM_MAIN_FUNC1": 101, "PWM_MAIN_FUNC2": 104,
                        "PWM_MAIN_FUNC3": 103, "PWM_MAIN_FUNC4": 102,
                        "PWM_MAIN_FUNC5": 302, "PWM_MAIN_FUNC6": 304,
                        "PWM_MAIN_FUNC7": 303, "PWM_MAIN_FUNC8": 301}
            if any(assignments.get(key) != value for key, value in expected.items()):
                raise RuntimeError("Peripheral output assignments differ from the verified arm wiring.")
            if sum(301 <= value <= 304 for value in assignments.values()) != 4:
                raise RuntimeError("Duplicate peripheral output assignments; stopping.")
            for name, expected_value in (("CA_AIRFRAME", 13), ("CA_ROTOR_COUNT", 4), ("MORPH_PUB_EN", 0)):
                if parse_parameters(link.shell(f"param show {name}")).get(name) != expected_value:
                    raise RuntimeError(f"Requires {name}={expected_value} before this bench test.")
            publisher_status = link.shell("morphing_arm_publisher status")
            if "not running" not in publisher_status.lower():
                raise RuntimeError("Publisher must be stopped before this isolated bench test.")
            requested = assumed_calibration() if args.assume_linear_bench else {}
            values = parse_parameters(link.shell("param show MARM_*"))
            if not set(assumed_calibration()).issubset(values):
                raise RuntimeError("Calibration parameters missing; flash the publisher firmware.")
            if not args.assume_linear_bench:
                for arm in ("FR", "RR", "RL", "FL"):
                    lo, mid, hi = (values[f"MARM_{arm}_{p}"] for p in ("NEG", "ZERO", "POS"))
                    if not ((lo < mid < hi or lo > mid > hi) and max(abs(lo), abs(mid), abs(hi)) <= 30):
                        raise RuntimeError("Set measured calibration first, or explicitly use --assume-linear-bench.")
            backup = {name: values[name] for name in requested}
            backup["MORPH_PUB_EN"] = 0
            if args.enable_prearm_bench:
                prearm = parse_parameters(link.shell("param show COM_PREARM_MODE")).get("COM_PREARM_MODE")
                if prearm is None:
                    raise RuntimeError("COM_PREARM_MODE missing.")
                backup["COM_PREARM_MODE"] = prearm
            backup_path.write_text(json.dumps({"device": args.device, "parameters": backup,
                                               "restored": False}, indent=2) + "\n")
            link.note(f"Parameter recovery record: {backup_path}")
            if args.assume_linear_bench:
                link.note("ASSUMED BENCH CALIBRATION: command 0.05 represents 1.5 deg in software ONLY.")
            mutated = True
            for name, value in requested.items():
                link.set_param(name, value)
            # Prime peripheral targets at centre before allowing prearmed output.
            link.targets = [0.0, 0.0, 0.0, 0.0]
            link.dwell(0.5)
            if args.enable_prearm_bench:
                link.note("BENCH ONLY: temporarily enabling non-motor outputs with COM_PREARM_MODE=2.")
                link.set_param("COM_PREARM_MODE", 2)
                link.dwell(0.5)
                require_disarmed_prearmed(link.shell("listener actuator_armed -n 1"))
            link.set_param("MORPH_PUB_EN", 1)
            started = True  # Attempt cleanup even if the start reply is lost.
            link.shell("morphing_arm_publisher start")
            link.dwell(0.5)
            health = link.shell("listener morphing_publisher_status -n 1")
            if not (re.search(r"initialized_mask:\s*15\b", health)
                    and re.search(r"stale:\s*False\b", health, re.I)):
                raise RuntimeError("Publisher did not initialize from MAVLink commands.")
            link.shell("control_allocator status")
            link.targets = [0.05, 0.0, 0.0, 0.0]
            link.dwell(0.5)
            for command in ("listener morphing_arm_state -n 1", "listener morphing_geometry -n 1",
                            "listener morphing_allocation_matrix -n 1", "control_allocator status"):
                link.shell(command)
            link.note("Returning all four channels to command centre.")
            link.targets = [0.0, 0.0, 0.0, 0.0]
            link.dwell(0.5)
            link.shell("listener morphing_arm_state -n 1")
            link.shell("control_allocator status")
        finally:
            failures = []
            link.cleanup = True
            if mutated:
                if started and not link.unsafe:
                    try:
                        link.targets = [0.0] * 4
                        link.dwell(0.5)
                    except Exception as error:
                        failures.append(str(error))
                link.targets = None
                try:
                    link.set_param("MORPH_PUB_EN", 0)
                    if started:
                        link.shell("morphing_arm_publisher stop")
                except Exception as error:
                    failures.append(str(error))
                for name, value in backup.items():
                    try:
                        link.set_param(name, value)
                    except Exception as error:
                        failures.append(str(error))
                if not failures:
                    backup_path.write_text(json.dumps({"device": args.device, "parameters": backup,
                                                       "restored": True}, indent=2) + "\n")
                    link.note("Original parameter values restored and verified. No arming command was sent.")
                else:
                    link.note("RESTORATION INCOMPLETE. Keep disarmed. Restore values from " + str(backup_path))
                    link.note("\n".join(failures))
            link.close()
            if failures:
                raise RuntimeError("Bench cleanup incomplete; see recovery record.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="USB serial device; close QGroundControl first")
    parser.add_argument("--props-removed", action="store_true", required=True,
                        help="Confirm this is a propellers-off bench test")
    parser.add_argument("--assume-linear-bench", action="store_true",
                        help="Temporarily use unmeasured -30/0/+30 calibration; restore original values afterward")
    parser.add_argument("--enable-prearm-bench", action="store_true",
                        help="Temporarily set COM_PREARM_MODE=2 for non-motor output while disarmed; restore afterward")
    parser.add_argument("--log-dir", default="logs/morphing-bench")
    args = parser.parse_args()
    try:
        run(args)
    except (Exception, KeyboardInterrupt) as error:
        print(f"TEST STOPPED: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
