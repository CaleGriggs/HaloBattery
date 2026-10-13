"""Valve Steam Controller(2026) over the Steam Controller Puck \n
From fraustiz who made a similar project in Rust at https://github.com/fraustiz/steam-controller-battery \n
    None of this is documented by Valve; all of it was established by probing the hardware.
    The dongle (PID 0x1304) exposes several HID interfaces on usage_page 0xFF00. The one with usage 0x0002 is 
    the control interface; the others, usage 0x0001, are the pairing slots — only one emits, the one where a 
    controller is actually connected.

    Input reports are numbered. Two matter:
    id      |   rate    |   contents
    0x42    |   ~270 Hz |   buttons, axes, trackpads, gyroscope
    0x43    |   ~0.3 Hz |   power state

    Report 0x43 is fifteen bytes:

    [0] 0x43        report id
    [1] state       0x01 discharging, 0x02 charging, 0x04 charged
    [2] percentage  0 to 100
    [3] voltage     cell, 16-bit little-endian, millivolts
    [5] voltage     second value, role not established
    [7] supply      16-bit little-endian, millivolts; zero off mains
    [9] current     16-bit little-endian, milliamps; zero at full charge

    Three states, read off the hardware:
    Situation           | 	[1]  |	[7..9]  |	[9..11] |
    Off the dock, 94 %  |	0x01 |	0       |	0       |
    Charging, 96 %      |	0x02 |	4800 mV |	175 mA  |
    On the dock, 100 %  |	0x04 |	4860 mV |	0 mA    |

When using the puck or USB to connect a Steam Controller, the controller doesn't interact with Windows' 
notification system the same way it does when connecting via bluetooth. This means that connecting and 
disconnecting a controller from the puck doesn't force a poll update, whereas unplugging the puck itself 
does meaning you'd have to wait for the main app to refresh to see the new icon for the new controller.

For every puck or USB connection, a thread opens up to listen for reports from each possible device connection.
Ignores bluetooth connections which are handled through bluetooth.py
"""
from __future__ import annotations

import time
import threading
from typing import List, Optional

import hid

from . import hidlist
from .base import DeviceStatus, Provider, hexdump, log

VID = 0x28DE
PID = 0x1304

USAGE_PAGE = 0xFF00
USAGE = 0x0001

REPORT_ID = 0x43
REPORT_SIZE = 0x0F

#STATE_DISCHARGING = 0x01
STATE_CHARGING = 0x02
#STATE_CHARGED = 0x04

ATTEMPTS = 3
READ_TIMEOUT = 100

POLLTIME = 5

class _Puck():

    def __init__(self, lock: threading.Lock, puckNum, repoll = None):
        self.puckNum : int = puckNum
        self.slots: list = [None] * 4
        self.lock = lock
        self.wait = threading.Event().clear()
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._probe_slots, daemon=True)
        self._diag : List = []
        self._reports : dict [int, Optional[DeviceStatus]] = {1:None,2:None,3:None,4:None}
        self.repoll : threading.Event = repoll

    @staticmethod
    def _status_changed(previous: Optional[DeviceStatus], current: Optional[DeviceStatus]) -> bool:
        if previous is None or current is None:
            return previous is not current
        return (
            previous.online != current.online
            or previous.charging != current.charging
            or previous.source != current.source
            or previous.kind != current.kind
            or previous.via != current.via
        )

    def get_reports(self):
        out = []
        with self.lock:
            for report in self._reports.values():
                if report:
                    out.append(report)
            return [out, self._diag]

    def _poll_callback(self):
        if self.repoll is not None:
            self.repoll.set()

    def _probe_slots(self):
        while True:
            loopDiag = []
            changed = False
            for slotNum, slot in enumerate(self.slots):
                if slot is None:
                    continue
                # The first connected controller has been the last in the list of hidResponses for me
                # so I'm making the assumption each slot is reported in reverse order: 4,3,2,1
                readout = self._read_slot(slot)
                slotStats = readout[0]
                diag = readout[1]
                with self.lock:
                    previous = self._reports.get(slotNum + 1)
                    if self._status_changed(previous, slotStats):
                        changed = True
                    self._reports[slotNum + 1] = slotStats
                loopDiag.append(diag)
            self._diag = loopDiag
            if changed and self.repoll is not None:
                self._poll_callback()
            self._stop.wait(timeout=10)
            if self._stop.is_set():
                break

    def _read_slot(self, slot) -> tuple[DeviceStatus, str]:
        # Test each controller slot with proper REPORT_ID and REPORT_SIZE
        # connected controllers send reports in a steam
        device = hid.device()
        slotStats = [None, ""]

        try:
            device.open_path(slot["path"])
        except (OSError, IOError) as e:
            with self.lock:
                self._diag.append(f"  open: {e}")
            return slotStats
        defaultDiag = (f"[Valve Software] pid = {PID} {slot['product_string']} {slot['serial_number']}"
            f" interface= {slot['interface_number']} {slot['usage_page']}:{slot['usage']}" 
            f" INACTIVE")
        device.set_nonblocking(False)
        for _ in range(ATTEMPTS):
            try: 
                report = device.read(64, timeout_ms = READ_TIMEOUT)
                if report:
                    # Sending reports
                    slotStats = self._read_stream(device, slot, defaultDiag)
                    break
                else:
                    # Not sending reports
                    slotStats = [None, defaultDiag]
            except Exception as e:
                pass
            finally:
                try:
                    device.close()
                except Exception:
                    pass
        return slotStats

    def _read_stream(self, device, puckSlot, defaultDiag) -> tuple[Optional[DeviceStatus], str]:
        # Doesn't seem to be a way to request a report by ID so the best I can figure is to listen to
        # the stream of reports and wait for REPORT_ID
        endTime = time.monotonic() + POLLTIME
        while time.monotonic() < endTime:
            report = device.read(64, READ_TIMEOUT)
            if report and report[0] == REPORT_ID and len(report) >= REPORT_SIZE:
                state = report[1]
                percent = report[2]
                chargeState = False
                if state == STATE_CHARGING:
                    chargeState = True
                diag = (f"[Valve Software] pid = {PID} {puckSlot['product_string']} {puckSlot['serial_number']}"
                        f" interface= {puckSlot['interface_number']} {puckSlot['usage_page']}:{puckSlot['usage']}"
                        f" ACTIVE {hexdump(report, 4)}")
                return DeviceStatus( 
                            key=f"Steam Controller ",
                            name=f"Steam Controller ",
                            level=percent, charging=chargeState,
                            online=True, source="Valve", 
                            kind="gamepad", via="dongle"
                        ), diag
        return None, defaultDiag
    
    def kill(self):
        self._stop.set()
    
    def __del__(self):
        pass

class _Handler():
 
    def __init__(self, lock, on_poll_callback = None):
        self.pucks:dict[str, _Puck] = {}
        self.lock = lock
        self._on_poll_callback = on_poll_callback
        self.repoll = threading.Event()
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._poll_callback, daemon=True)
        self.thread.start()

    def _poll_callback(self):
        while True:
            self.repoll.wait(timeout=5)            
            if self.stop.is_set():
                self.repoll.clear()
                break
            if self._on_poll_callback and self.repoll.is_set():
                self._on_poll_callback()
            self.repoll.clear()

    def _filter_response(self, hidResponse):
        seen = []
        newPucks:dict[str, List] = {}
        numPucks = len(self.pucks.keys())
        for item in hidResponse:
            # Interface_number -1 seems to be for bluetooth which works wihtout this script
            if (item["usage_page"] == USAGE_PAGE and item["usage"] == USAGE and item["interface_number"] != -1):
                serialNum:str = item["serial_number"]
                if serialNum not in seen:
                    seen.append(serialNum)
                if serialNum not in self.pucks.keys():
                    #create new pucks
                    self.pucks[serialNum] = _Puck(self.lock, numPucks, self.repoll)
                    newPucks[serialNum] = []
                    numPucks += 1
                if serialNum in newPucks.keys():
                    #store possible slots
                    newPucks[serialNum].append(item)
        for serial in newPucks.keys():
            #distribute slots to pucks
            self.pucks[serial].slots = newPucks[serial]
            if not self.pucks[serial].thread.is_alive():
                self.pucks[serial].thread.start()
        diff = list(set(self.pucks.keys())^set(seen))
        if diff:
            # remove disconnected pucks
            with self.lock:
                toDelete = []
                for serial in diff:
                    toDelete.append(self.pucks.pop(serial))
                deletedPucks = []
                for puck in toDelete:
                    deletedPucks.append(puck.puckNum)
                    puck.kill()
                    del puck         
                
    def poll(self, hidResponse):
        self._filter_response(hidResponse)
        out = []
        outDiag = []
        for puck in self.pucks.values():
            reports, diag = puck.get_reports()
            out += reports
            outDiag += diag
        return out, outDiag
            
    def kill(self):
        with self.lock:
            self.stop.set()
            toDelete = self.pucks.values()
            for puck in toDelete:
                puck.kill()
                del puck
            self.pucks = {}
        return True

    def __del__(self):
        pass

class ValveSteamControllerProvider(Provider):
    name = "valve software"
    def __init__(self, on_poll_callback = None):
        self._diag: List[str] = []
        self._on_poll_callback = on_poll_callback
        self._handler = None
        self.lock = threading.Lock()

    def _poll_callback(self):
        """Set wait() in the main app to repoll"""
        if self._on_poll_callback:
            self._on_poll_callback.set()

    def poll(self) -> List[DeviceStatus]:
        out: List[DeviceStatus] = []
        self._diag = []
        try:
            hidResponse = hidlist.enumerate(VID)
        except Exception as e:
            log.warning(f"hid.enumerate(valve): {e}")
            return []
        if hidResponse:
            if not self._handler:
                self._handler = _Handler(self.lock, self._poll_callback)
            out, self._diag = self._handler.poll(hidResponse)
        else:
            if self._handler:
                deleteHandler = self._handler
                self._handler = None
                if deleteHandler.kill():
                    del deleteHandler
        copied = []
        for i, report in enumerate(out, 1):
            clone = report.__class__(**report.__dict__)
            clone.key += f"{i}"
            clone.name += f"{i}"
            copied.append(clone)
        return copied
    
    def diagnostics(self) -> List[str]:
        return list(self._diag)