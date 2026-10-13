"""Tests for providers/valve_steam_controller.py. No hardware is needed.

The Steam controller dongle exposes a HID interface that returns a 15-byte battery
report with report id 0x43, followed by a charging state and battery percentage.
This unit test exercises the parsing path directly without opening any real device.

Run from the repository root:

    python -m unittest discover -s tests
"""
import os
import random
import sys
import threading
import types
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from providers import valve_steam_controller as V  # noqa: E402


def make_fake_hid_response():
    """Build a stable but randomized fake Valve HID response."""
    rng = random.Random(2026)
    serial = f"FXB{rng.randint(100000000, 999999999):09d}"
    interfaces = [2, 3, 4, 5]
    rng.shuffle(interfaces)
    response = []
    for iface in interfaces:
        response.append({
            "path": f"{rng.getrandbits(32):08x}-{iface}".encode(),
            "vendor_id": V.VID,
            "product_id": V.PID,
            "serial_number": serial,
            "release_number": rng.randint(1, 10),
            "manufacturer_string": "Valve Software",
            "product_string": "Steam Controller Puck",
            "usage_page": V.USAGE_PAGE,
            "usage": V.USAGE,
            "interface_number": iface,
            "bus_type": 1,
        })
        response.append({
            "path": f"{rng.getrandbits(32):08x}-kbd-{iface}".encode(),
            "vendor_id": V.VID,
            "product_id": V.PID,
            "serial_number": serial,
            "release_number": rng.randint(1, 10),
            "manufacturer_string": "Valve Software",
            "product_string": "Steam Controller Puck",
            "usage_page": 0x0001,
            "usage": 0x0006,
            "interface_number": iface,
            "bus_type": 1,
        })
    return response


FAKE_HID_RESPONSE = make_fake_hid_response()


class FakeDevice:
    def __init__(self, report):
        self.report = report
        self.calls = 0

    def read(self, n, timeout=None):
        self.calls += 1
        if self.calls == 1:
            return list(self.report)
        return []


class ProviderTest(unittest.TestCase):
    def setUp(self):
        self._saved = (V.hid, V.hidlist)

    def tearDown(self):
        V.hid, V.hidlist = self._saved


class ReadStreamTest(ProviderTest):
    def test_active_charge_report_has_level_and_charging_state(self):
        report = [0x43, 0x02, 96] + [0] * 12
        slot = {
            "product_string": "Steam Controller",
            "serial_number": "SN123",
            "interface_number": 2,
            "usage_page": V.USAGE_PAGE,
            "usage": V.USAGE,
            "path": b"/test/slot",
        }
        device = FakeDevice(report)
        puck = V._Puck(threading.Lock(), 0)

        status, diag = puck._read_stream(device, slot, "fallback")

        self.assertIsNotNone(status)
        self.assertEqual((status.level, status.charging, status.online, status.source,
                          status.kind, status.via),
                         (96, True, True, "Valve", "gamepad", "dongle"))
        self.assertEqual(status.name, "Steam Controller ")
        self.assertIn("ACTIVE", diag)
        self.assertIn("Steam Controller", diag)


class HandlerFilterTest(unittest.TestCase):
    def test_handler_tracks_a_real_thread_and_puck_starts_with_four_slots(self):
        handler = V._Handler(threading.Lock(), on_poll_callback=None)
        puck = V._Puck(threading.Lock(), 0)
        try:
            self.assertIsInstance(handler.thread, threading.Thread)
            self.assertEqual(puck.slots, [None] * 4)
        finally:
            puck.kill()
            handler.kill()

    def test_filter_response_creates_a_puck_per_serial_and_ignores_non_matches(self):
        handler = V._Handler(threading.Lock(), on_poll_callback=None)
        try:
            handler._filter_response(FAKE_HID_RESPONSE)

            self.assertEqual(len(handler.pucks), 1)
            serial = next(iter(handler.pucks))
            self.assertEqual(len(handler.pucks[serial].slots), 4)
            self.assertEqual(sorted(slot["interface_number"] for slot in handler.pucks[serial].slots),
                             [2, 3, 4, 5])
        finally:
            handler.kill()

    def test_filter_response_removes_missing_pucks(self):
        handler = V._Handler(threading.Lock(), on_poll_callback=None)
        try:
            serial = "FXB9961401E19"
            handler._filter_response([
                {"serial_number": serial, "interface_number": 2,
                 "usage_page": V.USAGE_PAGE, "usage": V.USAGE, "path": b"slot-2"},
                {"serial_number": serial, "interface_number": 3,
                 "usage_page": V.USAGE_PAGE, "usage": V.USAGE, "path": b"slot-3"},
            ])
            self.assertIn(serial, handler.pucks)

            handler._filter_response([])
            self.assertEqual(handler.pucks, {})
        finally:
            handler.kill()

    def test_filter_response_ignores_bluetooth_and_non_puck_interfaces(self):
        handler = V._Handler(threading.Lock(), on_poll_callback=None)
        try:
            response = [
                {"serial_number": "BT-1", "interface_number": -1,
                 "usage_page": V.USAGE_PAGE, "usage": V.USAGE, "path": b"ignore-bluetooth"},
                {"serial_number": "KEYBOARD", "interface_number": 1,
                 "usage_page": 0x0001, "usage": 0x0006, "path": b"ignore-keyboard"},
                {"serial_number": "STEAM-1", "interface_number": 4,
                 "usage_page": V.USAGE_PAGE, "usage": V.USAGE, "path": b"steam-slot"},
            ]

            handler._filter_response(response)

            self.assertEqual(set(handler.pucks), {"STEAM-1"})
            self.assertEqual(handler.pucks["STEAM-1"].slots[0]["path"], b"steam-slot")
        finally:
            handler.kill()


class PollTest(ProviderTest):
    def test_empty_hid_response_returns_no_devices(self):
        V.hidlist = types.SimpleNamespace(enumerate=lambda vid=0: [])
        self.assertEqual(V.ValveSteamControllerProvider().poll(), [])

    def test_status_change_ignores_key_suffixes_and_battery_level_only_updates(self):
        puck = V._Puck(threading.Lock(), 0)
        previous = V.DeviceStatus(
            key="Steam Controller ",
            name="Steam Controller ",
            level=80,
            charging=False,
            online=True,
            source="Valve",
            kind="gamepad",
            via="dongle",
        )
        current = V.DeviceStatus(
            key="Steam Controller 1",
            name="Steam Controller 1",
            level=81,
            charging=False,
            online=True,
            source="Valve",
            kind="gamepad",
            via="dongle",
        )

        self.assertFalse(puck._status_changed(previous, current))

    def test_status_change_detects_real_connect_and_disconnect_events(self):
        puck = V._Puck(threading.Lock(), 0)
        previous = V.DeviceStatus(
            key="Steam Controller ",
            name="Steam Controller ",
            level=75,
            charging=False,
            online=True,
            source="Valve",
            kind="gamepad",
            via="dongle",
        )
        disconnected = V.DeviceStatus(
            key="Steam Controller ",
            name="Steam Controller ",
            level=75,
            charging=False,
            online=False,
            source="Valve",
            kind="gamepad",
            via="dongle",
        )

        self.assertTrue(puck._status_changed(previous, disconnected))


if __name__ == "__main__":
    unittest.main()
