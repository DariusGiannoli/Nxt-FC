"""Offline tests only; no serial connection or physical commands."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("bench", Path(__file__).with_name("morphing_mavlink_bench.py"))
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)

STATUS = """armed: False
prearmed: True
lockdown: False
manual_lockdown: False
force_failsafe: False
in_esc_calibration_mode: False
"""


class FakeLink:
    fail_movement = False
    last = None

    def __init__(self, device, log):
        type(self).last = self
        self.params = {name: 0.0 for name in bench.assumed_calibration()}
        self.params.update(MORPH_PUB_EN=0, COM_PREARM_MODE=0, CA_AIRFRAME=13, CA_ROTOR_COUNT=4)
        self.params.update({f"PWM_MAIN_FUNC{i}": v for i, v in enumerate([101,104,103,102,302,304,303,301], 1)})
        self.original = self.params.copy()
        self.targets = None
        self.sent = []
        self.commands = []
        self.cleanup = False
        self.unsafe = False
        self.closed = False

    def note(self, text):
        pass

    def shell(self, command):
        self.commands.append(command)
        if command == "listener actuator_armed -n 1":
            return STATUS.replace("prearmed: True", f"prearmed: {self.params['COM_PREARM_MODE'] == 2}")
        if command.startswith("param show -q "):
            name = command.split()[-1]
            return f"{command}\r\n{self.params[name]:.4f}nsh> \x1b[K"
        if command.startswith("param show "):
            name = command.split()[-1]
            if name.startswith("MARM_"):
                return "Symbols: x = used, + = saved, * = unsaved\nnsh>"
            return "\n".join(f"x + {k} [1,2] : {v}" for k,v in self.params.items()
                             if k == name or (name.endswith('*') and k.startswith(name[:-1])))
        if command == "morphing_arm_publisher status":
            return "INFO [morphing_arm_publisher] not running"
        if command == "listener morphing_publisher_status -n 1":
            return "initialized_mask: 15\nstale: False"
        if command == "listener morphing_arm_state -n 1" and self.fail_movement and self.targets[0] > 0:
            raise RuntimeError("simulated failure")
        return "nsh>"

    get_param = bench.Link.get_param

    def set_param(self, name, value):
        self.params[name] = value

    def dwell(self, seconds):
        if self.targets is not None:
            self.sent.append(self.targets.copy())

    def close(self):
        self.closed = True


class BenchTests(unittest.TestCase):
    def test_quiet_read_includes_unused_parameters_and_handles_prompt(self):
        self.assertEqual(bench.parse_quiet_parameter("param show -q MARM_FR_NEG\r\n-30.0000nsh> "), -30.)
        self.assertEqual(bench.parse_quiet_parameter("param show -q MORPH_PUB_EN\n0\nnsh> "), 0.)
        with self.assertRaises(RuntimeError):
            bench.parse_quiet_parameter("param show -q MISSING\nnsh> ")
        link = FakeLink("fake", None)
        self.assertNotIn("MARM_FR_NEG [", link.shell("param show MARM_*"))
        self.assertEqual(link.get_param("MARM_FR_NEG"), 0.)

    def test_real_fc_quiet_output_with_terminal_escapes(self):
        # Exact response captured from the failed FC run, including erase-to-EOL.
        output = "param show -q CA_AIRFRAME\n13nsh> \x1b[K"
        self.assertEqual(bench.parse_quiet_parameter(output), 13.)
        for trailer in ("\x1b", "\x1b[", "\x1b[K", "\x1b[0m\x1b[K"):
            self.assertEqual(bench.parse_quiet_parameter("param show -q X\r\n-30.0000nsh> " + trailer), -30.)
        self.assertEqual(bench.parse_quiet_parameter("param show -q X\n\x1b[32m0.0000\x1b[0mnsh> "), 0.)
        with self.assertRaises(RuntimeError):
            bench.parse_quiet_parameter("param show -q MISSING\nnsh> \x1b[K")
        bench.require_disarmed_prearmed("\x1b[0m" + STATUS + "nsh> \x1b[K")

    def test_parser_and_disarmed_gate(self):
        self.assertEqual(bench.parse_parameters("x + MARM_FR_NEG [1,2] : -30.0000"), {"MARM_FR_NEG": -30})
        bench.require_disarmed_prearmed(STATUS)
        with self.assertRaises(RuntimeError):
            bench.require_disarmed_prearmed(STATUS.replace("armed: False", "armed: True"))
        with self.assertRaises(RuntimeError):
            bench.require_disarmed_prearmed(STATUS.replace("prearmed: True", "prearmed: False"))
        bench.require_disarmed_prearmed(STATUS.replace("prearmed: True", "prearmed: False"), True)

    def exercise(self, failure):
        with tempfile.TemporaryDirectory() as folder, patch.object(bench, "Link", FakeLink):
            FakeLink.fail_movement = failure
            args = types.SimpleNamespace(device="fake", log_dir=folder, assume_linear_bench=True, enable_prearm_bench=True)
            if failure:
                with self.assertRaisesRegex(RuntimeError, "simulated failure"):
                    bench.run(args)
            else:
                bench.run(args)
            link = FakeLink.last
            self.assertEqual(link.params, link.original)
            self.assertTrue(link.closed)
            self.assertTrue(all(all(abs(v) <= .05 for v in target) for target in link.sent))
            self.assertEqual(link.sent[-1], [0.] * 4)
            self.assertIn("morphing_arm_publisher stop", link.commands)
            self.assertNotIn("param show MARM_*", link.commands)
            self.assertTrue(all(f"param show -q {name}" in link.commands for name in bench.assumed_calibration()))
            record = json.loads(next(Path(folder).glob('*restore.json')).read_text())
            self.assertTrue(record['restored'])

    def test_success_restores_parameters(self):
        self.exercise(False)

    def test_failure_restores_parameters(self):
        self.exercise(True)


if __name__ == "__main__":
    unittest.main()
