#!/usr/bin/env python3
"""Read the controller from the Black Pill: print the stream, or import it.

One labeled stream:

    arm_roll    wrist roll motor (MKS 1), degrees from the saved center
    arm_twist   wrist twist motor (MKS 2), degrees from the saved center
    arm_v       the linear rail (white AS5600 on the rail's small gear),
                degrees of that gear from the saved center, counted across
                full turns; or millimetres once --rail-mm-per-rev is set
    joy_x       joystick X (A4), -1.0 .. +1.0, 0 = middle
    joy_y       joystick Y (A5), -1.0 .. +1.0, 0 = middle
    button      switch (B9), 1 = pressed, 0 = not
    trigger     legacy trigger threshold, 1 = >50% travel, 0 = off
    trigger_percent  analog PS5 trigger 0..100; nan until calibrated

The joystick, button and trigger come from firmware/controller; with older
firmware those values are nan (button and trigger: None). The analog percentage
requires updated CTL2 firmware and a trigger wired to analog pin PA1.

Runs on Windows, macOS and Linux with nothing to install: it only needs
Python 3.8 or newer. (If pyserial happens to be installed it can be used
too, see --backend.)

PRINT THE STREAM (run this file):

    python read_encoders.py                    stream as CSV
    python read_encoders.py --format labeled   arm_roll=... arm_twist=... arm_v=...
    python read_encoders.py --format json      one JSON object per line
    python read_encoders.py --center           save the current pose as center, then stream
    python read_encoders.py --flip roll        reverse a sign (saved): roll, twist, v,
                                               several with commas, or none
    python read_encoders.py --rail-mm-per-rev 40   rail travel per gear turn (saved);
                                               arm_v is then in millimetres (0 = degrees)
    python read_encoders.py --raw              the drivers' own angles, no center
    python read_encoders.py --list             list the serial ports it can see
    python read_encoders.py COM5               use this port (or /dev/ttyACM0, ...)
    python read_encoders.py --stlink           read through the ST-Link instead of
                                               USB (needs read_controller.py and
                                               read_trigger.py next to this file)

CSV output is: time_ms,arm_roll,arm_twist,arm_v,joy_x,joy_y,button,trigger,trigger_percent.
Ctrl-C stops it.

USE IT FROM YOUR OWN CODE (keep this file next to yours):

    from read_encoders import EncoderStream

    with EncoderStream() as encoders:        # finds the board by itself
        for angles in encoders:              # one Angles per reading, forever
            print(angles.arm_roll, angles.arm_twist, angles.arm_v,
                  angles.joy_x, angles.joy_y, angles.button, angles.trigger)

    # or, inside a loop of your own that must not wait for the board:
    encoders = EncoderStream()
    encoders.start()                         # keeps reading in the background
    ...
    angles = encoders.latest()               # newest reading, or None so far
    ...
    encoders.close()

A value is nan (math.isnan) when that encoder did not answer; for arm_v
that includes the sensor seeing no magnet. The center is kept in
center.json next to this file. The motor drivers restart their angle count
when they are powered off and on, so set the center again after a power
cycle (--center, or EncoderStream.set_center()).

arm_v counts gear turns from where this program starts, so start it with
the rail within half a gear turn of where it was centered (or center again).
It can only keep count while the rail is read often enough: the gear must
turn less than half a turn between two readings. In a "for angles in
encoders" loop do no slow work between readings; if your loop is slow, use
start() and latest(), which keep reading in the background. If the count
may have slipped, a warning is printed once and rail_uncertain becomes True.

Works with any of the board's firmwares: the controller firmware and the
encoder stream firmware (which send lines by themselves) or the motor test
firmware (which is asked for a reading about 10 times a second).
"""

from __future__ import annotations

import argparse
import errno
import functools
import glob
import importlib
import importlib.util
import json
import math
import os
import platform
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

HEADER = "time_ms,arm_roll,arm_twist,arm_v,joy_x,joy_y,button,trigger,trigger_percent"
CENTER_FILE = Path(__file__).resolve().parent / "center.json"
BAUD = 115200

# USB identity of the Black Pill running an Arduino STM32 sketch.
BOARD_USB_VID = 0x0483
BOARD_USB_PID = 0x5740

# How long to listen for a self-streaming firmware before polling instead.
DETECT_S = 1.5

# Polling rate when the motor test firmware is on the board.
POLL_PERIOD_S = 0.1
POLL_TIMEOUT_S = 1.0

# The rail's turn count can no longer be trusted when its gear seems to
# move more than this between two readings (half a turn is the hard limit),
# or when two rail readings are further apart than this.
RAIL_SUSPECT_STEP_DEG = 120.0
RAIL_SUSPECT_GAP_S = 0.25

# Readings averaged when saving the center.
CENTER_SAMPLES = 10
CENTER_TIMEOUT_S = 10.0


class PortError(Exception):
    """The serial port could not be found, opened or read."""


class PortUnavailableError(PortError):
    """The port is busy, missing or not permitted; no other backend can help."""


class CenterError(Exception):
    """The center could not be recorded or loaded."""


class SerialLink(Protocol):
    """The few serial operations this program needs."""

    def read(self, wait_s: float) -> bytes:
        """Return whatever arrives within wait_s seconds (may be empty)."""
        ...

    def write(self, data: bytes) -> None:
        """Send bytes to the board."""
        ...

    def close(self) -> None:
        """Release the port."""
        ...


# --------------------------------------------------------------------------
# Serial port access with the standard library only
# --------------------------------------------------------------------------

if sys.platform == "win32":
    import ctypes
    import winreg
    from ctypes import wintypes

    _GENERIC_READ = 0x80000000
    _GENERIC_WRITE = 0x40000000
    _OPEN_EXISTING = 3
    _SETRTS = 3
    _SETDTR = 5
    _MAXDWORD = 0xFFFFFFFF
    _ERROR_FILE_NOT_FOUND = 2
    _ERROR_ACCESS_DENIED = 5
    _READ_SLICE_MS = 50

    # DCB flag bits: binary mode, DTR on, RTS on, no flow control.
    _DCB_FLAGS = 0x0001 | (0x01 << 4) | (0x01 << 12)

    class _DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wintypes.DWORD),
            ("BaudRate", wintypes.DWORD),
            ("Flags", wintypes.DWORD),
            ("wReserved", wintypes.WORD),
            ("XonLim", wintypes.WORD),
            ("XoffLim", wintypes.WORD),
            ("ByteSize", wintypes.BYTE),
            ("Parity", wintypes.BYTE),
            ("StopBits", wintypes.BYTE),
            ("XonChar", ctypes.c_char),
            ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char),
            ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char),
            ("wReserved1", wintypes.WORD),
        ]

    class _COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wintypes.DWORD),
            ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
            ("ReadTotalTimeoutConstant", wintypes.DWORD),
            ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
            ("WriteTotalTimeoutConstant", wintypes.DWORD),
        ]

    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)

    _kernel32.CreateFileW.argtypes = [
        wintypes.LPCWSTR,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.LPVOID,
        wintypes.DWORD,
        wintypes.DWORD,
        wintypes.HANDLE,
    ]
    _kernel32.CreateFileW.restype = wintypes.HANDLE
    _kernel32.GetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(_DCB)]
    _kernel32.GetCommState.restype = wintypes.BOOL
    _kernel32.SetCommState.argtypes = [wintypes.HANDLE, ctypes.POINTER(_DCB)]
    _kernel32.SetCommState.restype = wintypes.BOOL
    _kernel32.SetCommTimeouts.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(_COMMTIMEOUTS),
    ]
    _kernel32.SetCommTimeouts.restype = wintypes.BOOL
    _kernel32.EscapeCommFunction.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    _kernel32.EscapeCommFunction.restype = wintypes.BOOL
    _kernel32.ReadFile.argtypes = [
        wintypes.HANDLE,
        wintypes.LPVOID,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        wintypes.LPVOID,
    ]
    _kernel32.ReadFile.restype = wintypes.BOOL
    _kernel32.WriteFile.argtypes = [
        wintypes.HANDLE,
        wintypes.LPCVOID,
        wintypes.DWORD,
        ctypes.POINTER(wintypes.DWORD),
        wintypes.LPVOID,
    ]
    _kernel32.WriteFile.restype = wintypes.BOOL
    _kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    _kernel32.CloseHandle.restype = wintypes.BOOL

    _INVALID_HANDLE = ctypes.c_void_p(-1).value

    class NativeLink:
        """A COM port opened through the Win32 API."""

        def __init__(self, path: str) -> None:
            name = path if path.startswith("\\\\.\\") else "\\\\.\\" + path
            handle = _kernel32.CreateFileW(
                name,
                _GENERIC_READ | _GENERIC_WRITE,
                0,
                None,
                _OPEN_EXISTING,
                0,
                None,
            )

            if handle is None or handle == _INVALID_HANDLE:
                code = ctypes.get_last_error()
                if code == _ERROR_ACCESS_DENIED:
                    raise PortUnavailableError(
                        f"Cannot open {path}: another program has it open "
                        "(close any serial monitor)"
                    )
                if code == _ERROR_FILE_NOT_FOUND:
                    raise PortUnavailableError(f"Cannot open {path}: no such port")
                raise PortError(f"Cannot open {path}: Windows error {code}")

            self._handle = handle
            self._path = path

            try:
                self._configure()
            except PortError:
                _kernel32.CloseHandle(handle)
                raise

        def _configure(self) -> None:
            dcb = _DCB()
            dcb.DCBlength = ctypes.sizeof(_DCB)

            if not _kernel32.GetCommState(self._handle, ctypes.byref(dcb)):
                raise PortError(f"{self._path} is not a serial port")

            dcb.BaudRate = BAUD
            dcb.ByteSize = 8
            dcb.Parity = 0
            dcb.StopBits = 0
            dcb.Flags = _DCB_FLAGS

            if not _kernel32.SetCommState(self._handle, ctypes.byref(dcb)):
                raise PortError(f"Cannot configure {self._path}")

            # Return as soon as any byte arrives, or after the slice.
            timeouts = _COMMTIMEOUTS(_MAXDWORD, _MAXDWORD, _READ_SLICE_MS, 0, 1000)

            if not _kernel32.SetCommTimeouts(self._handle, ctypes.byref(timeouts)):
                raise PortError(f"Cannot set timeouts on {self._path}")

            # The board only sends while DTR is asserted.
            _kernel32.EscapeCommFunction(self._handle, _SETDTR)
            _kernel32.EscapeCommFunction(self._handle, _SETRTS)

        def read(self, wait_s: float) -> bytes:
            buffer = ctypes.create_string_buffer(4096)
            received = wintypes.DWORD(0)
            deadline = time.monotonic() + max(wait_s, 0.0)

            while True:
                ok = _kernel32.ReadFile(
                    self._handle, buffer, 4096, ctypes.byref(received), None
                )
                if not ok:
                    raise PortError("The board disconnected.")
                if received.value:
                    return buffer.raw[: received.value]
                if time.monotonic() >= deadline:
                    return b""

        def write(self, data: bytes) -> None:
            written = wintypes.DWORD(0)
            ok = _kernel32.WriteFile(
                self._handle, data, len(data), ctypes.byref(written), None
            )
            if not ok or written.value != len(data):
                raise PortError("Could not send to the board.")

        def close(self) -> None:
            _kernel32.CloseHandle(self._handle)

    def native_ports() -> list[str]:
        """COM ports from the registry, USB serial devices first."""
        usb: list[str] = []
        other: list[str] = []

        try:
            key = winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM"
            )
        except OSError:
            return []

        with key:
            index = 0
            while True:
                try:
                    device, value, _ = winreg.EnumValue(key, index)
                except OSError:
                    break
                index += 1
                port = str(value)
                if "USBSER" in device.upper():
                    usb.append(port)
                else:
                    other.append(port)

        return sorted(usb, key=_com_order) + sorted(other, key=_com_order)

    def _com_order(port: str) -> tuple[int, str]:
        """Sort key that puts COM3 before COM10."""
        digits = port[3:]
        return (int(digits) if digits.isdigit() else 0, port)

    def _subkeys(key: winreg.HKEYType) -> list[str]:
        names: list[str] = []
        index = 0
        while True:
            try:
                names.append(winreg.EnumKey(key, index))
            except OSError:
                return names
            index += 1

    def port_is_board(port: str) -> bool | None:
        """True if this COM port belongs to a device with the board's USB id.

        None means Windows would not say (the registry could not be read).
        """
        prefix = f"VID_{BOARD_USB_VID:04X}&PID_{BOARD_USB_PID:04X}"

        try:
            usb = winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE, r"SYSTEM\CurrentControlSet\Enum\USB"
            )
        except OSError:
            return None

        with usb:
            for device in _subkeys(usb):
                if not device.upper().startswith(prefix):
                    continue
                try:
                    device_key = winreg.OpenKey(usb, device)
                except OSError:
                    continue
                with device_key:
                    for instance in _subkeys(device_key):
                        try:
                            with winreg.OpenKey(
                                device_key, instance + r"\Device Parameters"
                            ) as parameters:
                                value, _ = winreg.QueryValueEx(parameters, "PortName")
                        except OSError:
                            continue
                        if str(value).upper() == port.upper():
                            return True

        return False

else:
    import fcntl
    import grp
    import select
    import subprocess
    import termios

    _PORT_PATTERNS = (
        "/dev/cu.usbmodem*",
        "/dev/serial/by-id/*STMicroelectronics*",
        "/dev/ttyACM*",
    )

    _BUSY = "another program has it open (close any serial monitor)"

    def _permission_hint(path: str) -> str:
        """Advice for a port the user is not allowed to open."""
        group = "dialout"
        try:
            owner = grp.getgrgid(os.stat(path).st_gid).gr_name
        except (OSError, KeyError):
            owner = ""
        if owner and owner not in ("root", "wheel"):
            group = owner
        return (
            f" (no permission: add yourself to the '{group}' group with "
            f"'sudo usermod -aG {group} $USER', then log out and in)"
        )

    class NativeLink:
        """A serial device opened with termios (macOS, Linux, BSD)."""

        def __init__(self, path: str) -> None:
            try:
                fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            except OSError as error:
                if error.errno == errno.EBUSY:
                    raise PortUnavailableError(
                        f"Cannot open {path}: {_BUSY}"
                    ) from error
                if error.errno == errno.EACCES:
                    raise PortUnavailableError(
                        f"Cannot open {path}: {error.strerror}"
                        f"{_permission_hint(path)}"
                    ) from error
                if error.errno == errno.ENOENT:
                    raise PortUnavailableError(
                        f"Cannot open {path}: {error.strerror}"
                    ) from error
                raise PortError(f"Cannot open {path}: {error.strerror}") from error

            # Two readers on one port each get fragments of the lines, so
            # refuse to share it (Windows does this by itself).
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                os.close(fd)
                raise PortUnavailableError(f"Cannot open {path}: {_BUSY}") from error
            except OSError:
                pass  # this device cannot be locked; exclusive mode still applies

            exclusive = getattr(termios, "TIOCEXCL", None)
            if exclusive is not None:
                try:
                    fcntl.ioctl(fd, exclusive)
                except OSError:
                    pass  # not every tty driver supports exclusive mode

            self._fd = fd

            try:
                self._configure(path)
            except PortError:
                os.close(fd)
                raise

        def _configure(self, path: str) -> None:
            try:
                attrs = termios.tcgetattr(self._fd)
            except termios.error as error:
                raise PortError(f"{path} is not a serial port") from error

            cflag = attrs[2]
            control = attrs[6]

            if not isinstance(cflag, int) or not isinstance(control, list):
                raise PortError(f"Unexpected terminal settings on {path}")

            # 8 data bits, no parity, 1 stop bit, no flow control, raw.
            hardware_flow = getattr(termios, "CRTSCTS", 0)
            cflag &= ~(
                termios.CSIZE | termios.PARENB | termios.CSTOPB | hardware_flow
            )
            cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD

            control[termios.VMIN] = 0
            control[termios.VTIME] = 0

            raw_attrs: list[int | list[bytes | int]] = [
                0,
                0,
                cflag,
                0,
                termios.B115200,
                termios.B115200,
                control,
            ]

            try:
                termios.tcsetattr(self._fd, termios.TCSANOW, raw_attrs)
            except termios.error as error:
                raise PortError(f"Cannot configure {path}") from error

        def read(self, wait_s: float) -> bytes:
            try:
                ready, _, _ = select.select([self._fd], [], [], max(wait_s, 0.0))
                if not ready:
                    return b""
                chunk = os.read(self._fd, 4096)
            except BlockingIOError:
                return b""
            except OSError as error:
                raise PortError("The board disconnected.") from error

            if not chunk:
                raise PortError(
                    "The board disconnected (or another program is reading "
                    "the same port)."
                )

            return chunk

        def write(self, data: bytes) -> None:
            view = memoryview(data)
            deadline = time.monotonic() + 2.0

            while view:
                try:
                    sent = os.write(self._fd, view)
                except BlockingIOError:
                    sent = 0
                except OSError as error:
                    raise PortError("The board disconnected.") from error

                view = view[sent:]

                if view and time.monotonic() > deadline:
                    raise PortError("Could not send to the board.")
                if view:
                    time.sleep(0.005)

        def close(self) -> None:
            os.close(self._fd)

    def native_ports() -> list[str]:
        """Serial devices that look like the Black Pill."""
        found: list[str] = []

        for pattern in _PORT_PATTERNS:
            for match in sorted(glob.glob(pattern)):
                real = os.path.realpath(match)
                if all(os.path.realpath(other) != real for other in found):
                    found.append(match)

        return found

    def _sysfs_usb_id(port: str) -> tuple[int, int] | None:
        """USB vendor and product id of a tty, from Linux sysfs."""
        name = os.path.basename(os.path.realpath(port))
        interface = os.path.realpath(f"/sys/class/tty/{name}/device")
        parent = os.path.dirname(interface)

        # ttyACM: the USB device is one level up. ttyUSB: two levels up.
        for folder in (parent, os.path.dirname(parent)):
            try:
                with open(os.path.join(folder, "idVendor"), encoding="ascii") as file:
                    vendor = int(file.read().strip(), 16)
                with open(os.path.join(folder, "idProduct"), encoding="ascii") as file:
                    product = int(file.read().strip(), 16)
            except (OSError, ValueError):
                continue
            return vendor, product

        return None

    def _parse_ioreg(listing: str) -> dict[str, tuple[int, int]]:
        """Map each callout device in `ioreg -r -c IOUSBHostDevice -l` output
        to the USB ids of the nearest USB device above it in the tree."""
        found: dict[str, tuple[int, int]] = {}
        stack: list[list[int]] = []  # [column of the node, vendor, product]

        for line in listing.splitlines():
            marker = line.find("+-o ")
            if marker >= 0 and not line[:marker].strip(" |"):
                while stack and stack[-1][0] >= marker:
                    stack.pop()
                stack.append([marker, -1, -1])
                continue

            key, _, value = line.partition(" = ")
            key = key.strip(' |"')
            value = value.strip()

            if not stack:
                continue
            if key == "idVendor" and value.isdigit():
                stack[-1][1] = int(value)
            elif key == "idProduct" and value.isdigit():
                stack[-1][2] = int(value)
            elif key == "IOCalloutDevice":
                for _, vendor, product in reversed(stack):
                    if vendor >= 0 and product >= 0:
                        found[value.strip('"')] = (vendor, product)
                        break

        return found

    @functools.lru_cache(maxsize=1)
    def _mac_usb_ids() -> dict[str, tuple[int, int]]:
        try:
            result = subprocess.run(
                ["/usr/sbin/ioreg", "-r", "-c", "IOUSBHostDevice", "-l", "-w", "0"],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                timeout=10,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return {}
        return _parse_ioreg(result.stdout.decode("utf-8", errors="replace"))

    def port_is_board(port: str) -> bool | None:
        """True if this port is a device with the board's USB id, False if it
        is some other USB device, None if that cannot be found out."""
        if sys.platform == "darwin":
            ids = _mac_usb_ids().get(port.replace("/dev/tty.", "/dev/cu.", 1))
        else:
            ids = _sysfs_usb_id(port)

        if ids is None:
            return None
        return ids == (BOARD_USB_VID, BOARD_USB_PID)


# --------------------------------------------------------------------------
# Optional: pyserial, if it is installed
# --------------------------------------------------------------------------


class _PySerialPort(Protocol):
    timeout: float

    @property
    def in_waiting(self) -> int: ...

    def read(self, size: int) -> bytes: ...

    def write(self, data: bytes) -> object: ...

    def close(self) -> None: ...


def pyserial_available() -> bool:
    """True if the pyserial package can be imported."""
    return importlib.util.find_spec("serial") is not None


class PySerialLink:
    """A serial port opened through pyserial."""

    def __init__(self, path: str) -> None:
        try:
            module = importlib.import_module("serial")
            port: _PySerialPort = module.Serial(path, BAUD, timeout=0)
        except Exception as error:  # pyserial raises its own error types
            raise PortError(f"Cannot open {path} with pyserial: {error}") from error

        self._port = port

    def read(self, wait_s: float) -> bytes:
        try:
            wanted = max(wait_s, 0.0)
            if self._port.timeout != wanted:
                self._port.timeout = wanted
            first = self._port.read(1)
            if not first:
                return b""
            waiting = self._port.in_waiting
            return first + (self._port.read(waiting) if waiting else b"")
        except Exception as error:
            raise PortError("The board disconnected.") from error

    def write(self, data: bytes) -> None:
        try:
            self._port.write(data)
        except Exception as error:
            raise PortError("Could not send to the board.") from error

    def close(self) -> None:
        self._port.close()


def pyserial_ports() -> list[str]:
    """Ports pyserial reports with the board's USB vendor id."""
    try:
        tools = importlib.import_module("serial.tools.list_ports")
    except Exception:
        return []

    found: list[str] = []

    for info in tools.comports():
        if getattr(info, "vid", None) == BOARD_USB_VID:
            found.append(str(info.device))

    return found


def open_link(path: str, backend: str) -> SerialLink:
    """Open a port with the chosen backend ('auto', 'native' or 'pyserial')."""
    if backend == "pyserial":
        if not pyserial_available():
            raise PortError("pyserial is not installed (pip install pyserial).")
        return PySerialLink(path)

    try:
        return NativeLink(path)
    except PortUnavailableError:
        raise
    except PortError as native_error:
        if backend == "auto" and pyserial_available():
            try:
                return PySerialLink(path)
            except PortError:
                raise native_error from None
        raise


def candidate_ports() -> list[str]:
    """Every port worth trying, most likely first, without duplicates."""
    ports: list[str] = []
    seen: set[str] = set()

    for port in pyserial_ports() + native_ports():
        key = port.upper() if sys.platform == "win32" else os.path.realpath(port)
        if key not in seen:
            seen.add(key)
            ports.append(port)

    return ports


# --------------------------------------------------------------------------
# Center (zero position)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Center:
    """The saved zero position and sign of each encoder.

    motor1 / motor2    driver angle of arm_roll / arm_twist at the center
    flip1 / flip2      reverse the sign of arm_roll / arm_twist
    rail               AS5600 angle (0-360) of the rail gear at the center,
                       or None if the rail has not been centered
    flip_rail          reverse the sign of arm_v
    rail_mm_per_rev    rail travel for one turn of its gear, in millimetres;
                       None = report arm_v in degrees of the gear
    """

    motor1: float
    motor2: float
    flip1: bool
    flip2: bool
    rail: float | None = None
    flip_rail: bool = False
    rail_mm_per_rev: float | None = None


@dataclass(frozen=True)
class Reading:
    """One set of raw values from the board (nan / None = not available)."""

    motor1: float
    motor2: float
    as5600: float
    board_ms: int | None = None  # the firmware's own clock, when it sends one
    joy_x: float = math.nan  # -1..+1
    joy_y: float = math.nan  # -1..+1
    button: int | None = None  # 1 = pressed
    trigger: int | None = None  # legacy 1 = on
    trigger_percent: float = math.nan  # 0..100; nan if unavailable/unset


def load_center() -> Center | None:
    """Read the saved center, or None if there isn't one."""
    if not CENTER_FILE.exists():
        return None

    try:
        data: object = json.loads(CENTER_FILE.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as error:
        raise CenterError(
            f"Cannot read {CENTER_FILE}: {error}. Delete that file, or run "
            "with --center to replace it."
        ) from error

    if not isinstance(data, dict):
        raise CenterError(f"{CENTER_FILE} is not a center file")

    motor1 = data.get("motor1")
    motor2 = data.get("motor2")
    flip1 = data.get("flip1", False)
    flip2 = data.get("flip2", False)
    flip_rail = data.get("flip_v", False)

    if (
        isinstance(motor1, bool)
        or isinstance(motor2, bool)
        or not isinstance(motor1, (int, float))
        or not isinstance(motor2, (int, float))
        or not isinstance(flip1, bool)
        or not isinstance(flip2, bool)
        or not isinstance(flip_rail, bool)
    ):
        raise CenterError(f"{CENTER_FILE} has missing or wrong values")

    rail = _optional_number(data.get("arm_v"), "arm_v")
    rail_mm_per_rev = _optional_number(
        data.get("arm_v_mm_per_rev"), "arm_v_mm_per_rev"
    )

    if rail_mm_per_rev is not None and rail_mm_per_rev <= 0:
        raise CenterError(f"{CENTER_FILE}: arm_v_mm_per_rev must be above zero")

    try:
        zero1, zero2 = float(motor1), float(motor2)
    except OverflowError:
        zero1 = zero2 = math.inf

    if not (math.isfinite(zero1) and math.isfinite(zero2)):
        raise CenterError(
            f"{CENTER_FILE} has a center that is not a usable number. Delete "
            "that file, or run with --center to replace it."
        )

    return Center(zero1, zero2, flip1, flip2, rail, flip_rail, rail_mm_per_rev)


def _optional_number(value: object, name: str) -> float | None:
    """A finite number from the center file, or None if the entry is absent."""
    if value is None:
        return None

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CenterError(f"{CENTER_FILE}: {name} is not a number")

    try:
        number = float(value)
    except OverflowError:
        number = math.inf

    if not math.isfinite(number):
        raise CenterError(f"{CENTER_FILE}: {name} is not a usable number")

    return number


def save_center(center: Center) -> None:
    """Write the center file."""
    payload: dict[str, float | bool] = {
        "motor1": center.motor1,
        "motor2": center.motor2,
        "flip1": center.flip1,
        "flip2": center.flip2,
        "flip_v": center.flip_rail,
    }
    if center.rail is not None:
        payload["arm_v"] = center.rail
    if center.rail_mm_per_rev is not None:
        payload["arm_v_mm_per_rev"] = center.rail_mm_per_rev
    # Write a new file and swap it in, so a crash or power cut part-way
    # cannot leave an empty or half-written center file.
    temporary = CENTER_FILE.with_name(CENTER_FILE.name + ".tmp")
    try:
        with open(temporary, "w", encoding="utf-8") as file:
            file.write(json.dumps(payload, indent=2) + "\n")
            file.flush()
            os.fsync(file.fileno())
        os.replace(temporary, CENTER_FILE)
    except OSError as error:
        raise CenterError(f"Cannot write {CENTER_FILE}: {error}") from error


_FLIP_NAMES = {
    "1": 0, "roll": 0, "arm_roll": 0,
    "2": 1, "twist": 1, "arm_twist": 1,
    "3": 2, "v": 2, "arm_v": 2, "rail": 2,
}


def parse_flip(text: str) -> tuple[bool, bool, bool]:
    """Turn 'roll', 'twist,v', '1,2', 'none' ... into (roll, twist, v) flips."""
    cleaned = text.strip().lower()
    if cleaned == "none":
        return False, False, False

    parts = [part.strip() for part in cleaned.split(",") if part.strip()]
    if not parts or any(part not in _FLIP_NAMES for part in parts):
        raise CenterError(
            "--flip takes roll, twist, v (or 1, 2, 3), several with commas, "
            "or none"
        )

    chosen = {_FLIP_NAMES[part] for part in parts}
    return 0 in chosen, 1 in chosen, 2 in chosen


# --------------------------------------------------------------------------
# Reading the board
# --------------------------------------------------------------------------


def to_angle(text: str) -> float:
    """Parse an angle; anything that is not a finite number becomes nan."""
    try:
        value = float(text)
    except ValueError:
        return math.nan
    return value if math.isfinite(value) else math.nan


def to_switch(text: str) -> int | None:
    """'1' -> 1, '0' -> 0, anything else -> None."""
    value = text.strip()
    return int(value) if value in ("0", "1") else None


def parse_stream_line(line: str) -> Reading | None:
    """Parse 9-column CTL2 or 8-column CTL1 controller CSV, or 4-column motor CSV."""
    parts = line.split(",")
    stamp = parts[0].strip()
    if len(parts) not in (4, 8, 9) or not (stamp.isascii() and stamp.isdigit()):
        return None

    reading = Reading(
        to_angle(parts[1]), to_angle(parts[2]), to_angle(parts[3]), int(stamp)
    )

    if len(parts) in (8, 9):
        reading = Reading(
            reading.motor1,
            reading.motor2,
            reading.as5600,
            reading.board_ms,
            joy_x=to_angle(parts[4]),
            joy_y=to_angle(parts[5]),
            button=to_switch(parts[6]),
            trigger=to_switch(parts[7]),
            trigger_percent=to_angle(parts[8]) if len(parts) == 9 else math.nan,
        )

    return reading


def angle_from_reply(line: str) -> float:
    """Pull the angle out of 'motor 1 encoder: 12.345 deg' (nan if none)."""
    _, _, rest = line.partition(":")
    words = rest.split()
    return to_angle(words[0]) if words else math.nan


class Source:
    """Yields readings from whichever firmware is on the board."""

    def __init__(self, link: SerialLink) -> None:
        self._link = link
        self._pending = bytearray()
        self._queued: list[Reading] = []
        self.recognized = False
        self._refused = False
        self._next_poll = 0.0
        self._streaming = self._detect()

        if not self._streaming:
            # Not streaming by itself: see whether it answers a request.
            reading = self._poll_once()
            if not self.recognized and self._refused:
                # Characters another program left in the board's input line
                # spoiled that request. The line is clear now: ask again.
                reading = self._poll_once()
            self._queued.append(reading)

    def _lines(self, wait_s: float) -> list[str]:
        """Return complete lines received within wait_s seconds."""
        chunk = self._link.read(wait_s)
        if not chunk:
            return []

        self._pending.extend(chunk)
        lines: list[str] = []

        while True:
            newline = self._pending.find(b"\n")
            if newline < 0:
                break
            text = self._pending[:newline].decode("utf-8", errors="replace").strip()
            del self._pending[: newline + 1]
            if text:
                lines.append(text)

        # A device that never sends a newline must not grow this forever.
        if len(self._pending) > 65536:
            self._pending.clear()

        return lines

    def _detect(self) -> bool:
        deadline = time.monotonic() + DETECT_S

        while time.monotonic() < deadline:
            for line in self._lines(0.1):
                reading = parse_stream_line(line)
                if reading is not None:
                    self._queued.append(reading)
            if self._queued:
                self.recognized = True
                del self._queued[:-1]
                return True

        return False

    def _poll_once(self) -> Reading:
        """Ask the motor test firmware for one reading of all three encoders."""
        self._next_poll = time.perf_counter() + POLL_PERIOD_S
        self._link.write(b"r\n")

        motor1 = motor2 = as5600 = math.nan
        deadline = time.monotonic() + POLL_TIMEOUT_S

        while time.monotonic() < deadline:
            for line in self._lines(0.05):
                if line.startswith("motor 1 encoder"):
                    motor1 = angle_from_reply(line)
                    self.recognized = True
                elif line.startswith("motor 2 encoder"):
                    motor2 = angle_from_reply(line)
                    self.recognized = True
                elif line.startswith("AS5600"):
                    as5600 = angle_from_reply(line)
                    self.recognized = True
                elif line.startswith("ok") or line.startswith("error"):
                    self._refused = line.startswith("error")
                    return Reading(motor1, motor2, as5600)

        return Reading(motor1, motor2, as5600)

    def discard_waiting(self) -> None:
        """Forget readings received before now, so the next one is fresh."""
        self._queued.clear()

        if not self._streaming:
            return

        # The stream firmware keeps sending while nobody reads. Read until
        # the port hands over only a few lines at a time again: that is the
        # live stream, not the backlog.
        deadline = time.monotonic() + 2.0

        while time.monotonic() < deadline:
            if len(self._lines(0.1)) <= 5:
                break

    def next(self) -> Reading:
        """Block until the next reading is available."""
        if self._queued:
            return self._queued.pop(0)

        if not self._streaming:
            # Wait out the rest of the period before asking, not after, so
            # the reading is handed over as soon as it arrives.
            remaining = self._next_poll - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
            return self._poll_once()

        while not self._queued:
            for line in self._lines(1.0):
                parsed = parse_stream_line(line)
                if parsed is not None:
                    self._queued.append(parsed)

        return self._queued.pop(0)


def no_port_hint() -> str:
    """Extra advice for environments that hide USB devices."""
    if sys.platform == "win32":
        return ""
    if sys.platform in ("cygwin", "msys"):
        return (
            " This is a Cygwin/MSYS Python, which does not see Windows COM "
            "ports under their usual names: run the script with Windows "
            "Python instead (py read_encoders.py)."
        )
    if "microsoft" in platform.release().lower():
        return (
            " This is WSL, which cannot see USB devices by itself: run the "
            "script with Windows Python instead (in PowerShell: "
            "py read_encoders.py), or attach the board with "
            "'usbipd attach --wsl --busid <id>'."
        )
    if os.path.exists("/.dockerenv"):
        return (
            " This is a Docker container: start it with "
            "--device=/dev/ttyACM0 so it can see the board."
        )
    return ""


class ReadingSource(Protocol):
    """Where readings come from: the USB serial port, or the ST-Link."""

    def next(self) -> Reading:
        """Block until the next reading is available."""
        ...

    def discard_waiting(self) -> None:
        """Forget readings received before now."""
        ...


class Closable(Protocol):
    def close(self) -> None: ...


class StlinkSource:
    """Readings of the controller firmware, read through the ST-Link (SWD).

    Uses read_controller.py (and read_trigger.py), which must sit next to
    this file. The board keeps running while it is read.
    """

    def __init__(self) -> None:
        try:
            from read_controller import ControllerReader
            from read_trigger import TriggerError
        except ImportError as error:
            raise PortError(
                "Reading through the ST-Link needs read_controller.py and "
                "read_trigger.py next to this file."
            ) from error

        self._error_type: type[Exception] = TriggerError

        try:
            self._reader = ControllerReader()
        except TriggerError as error:
            raise PortError(str(error)) from error

        self._last_ms: int | None = None

    def next(self) -> Reading:
        deadline = time.monotonic() + 2.0

        while True:
            try:
                r = self._reader.read()
            except self._error_type as error:
                raise PortError(str(error)) from error

            if r.time_ms != self._last_ms:
                self._last_ms = r.time_ms
                return Reading(
                    r.arm_roll,
                    r.arm_twist,
                    r.arm_v,
                    r.time_ms,
                    joy_x=r.joy_x,
                    joy_y=r.joy_y,
                    button=r.button,
                    trigger=r.trigger,
                    trigger_percent=r.trigger_percent,
                )

            if time.monotonic() > deadline:
                raise PortError("The board stopped sending new readings.")

            time.sleep(0.003)

    def discard_waiting(self) -> None:
        # Each read is the newest value: nothing is ever queued.
        return None

    def close(self) -> None:
        self._reader.close()


def connect(port: str | None, backend: str) -> tuple[SerialLink, Source, str]:
    """Open the board's port. With no port given, try each candidate."""
    if port is not None:
        link = open_link(port, backend)
        try:
            source = Source(link)
        except BaseException:
            link.close()
            raise

        if not source.recognized:
            print(
                f"Warning: {port} is open but is not answering like the board.",
                file=sys.stderr,
            )

        return link, source, port

    candidates = candidate_ports()

    if not candidates:
        raise PortError(
            "No serial port found. Plug the Black Pill in by USB-C, or give the "
            "port name as an argument (see --list)." + no_port_hint()
        )

    identity = {candidate: port_is_board(candidate) for candidate in candidates}
    board = [candidate for candidate in candidates if identity[candidate]]
    unknown = [candidate for candidate in candidates if identity[candidate] is None]

    if not board and len(unknown) == 1:
        board = unknown

    if not board and unknown:
        raise PortError(
            "Found these serial ports but cannot tell which one is the Black "
            "Pill:\n  " + "\n  ".join(unknown) + "\nGive the port name as an "
            "argument."
        )

    if not board:
        raise PortError(
            "No Black Pill found (USB id "
            f"{BOARD_USB_VID:04x}:{BOARD_USB_PID:04x}). Other serial ports "
            "here: " + ", ".join(candidates) + ". If one of them is the board, "
            "give its name as an argument."
        )

    problems: list[str] = []

    for candidate in board:
        try:
            link = open_link(candidate, backend)
        except PortError as error:
            problems.append(str(error))
            continue

        try:
            source = Source(link)
        except PortError as error:
            link.close()
            problems.append(f"{candidate}: {error}")
            continue
        except BaseException:
            link.close()
            raise

        if source.recognized:
            return link, source, candidate

        link.close()
        problems.append(f"{candidate}: opened, but it did not answer like the board")

    raise PortError(
        "Could not find the board on any serial port:\n  " + "\n  ".join(problems)
    )


def wrap180(degrees: float) -> float:
    """The same angle expressed between -180 (inclusive) and 180."""
    return (degrees + 180.0) % 360.0 - 180.0


def circular_mean(angles: list[float]) -> float:
    """Average of angles in degrees that copes with the 359 -> 0 wrap."""
    east = sum(math.cos(math.radians(angle)) for angle in angles)
    north = sum(math.sin(math.radians(angle)) for angle in angles)
    return math.degrees(math.atan2(north, east)) % 360.0


class RailTracker:
    """Turns the AS5600's 0-360 angle into one that keeps counting turns.

    The rail's gear goes round many times over the rail's travel, and the
    sensor only knows the angle within one turn. This adds up the changes
    between readings, so the result keeps growing (or shrinking) past 360.
    It assumes the gear moves less than half a turn between two readings,
    and that it starts within half a turn of the zero.

    uncertain becomes True (and one warning is printed) when that can no
    longer be relied on: the gear seemed to jump more than a third of a
    turn, or two readings were too far apart in time. The count may then be
    off by whole turns until the rail is centered again.
    """

    def __init__(self, zero: float | None) -> None:
        self._zero = zero
        self._last: float | None = None
        self._last_time: float | None = None
        self._total = 0.0
        self.uncertain = False

    def update(self, angle: float, time_s: float | None = None) -> float:
        """Feed one sensor angle and when it was read (seconds, optional).

        Returns the continuous angle (nan if none).
        """
        if math.isnan(angle):
            return math.nan

        if self._last is None:
            # First reading: the position nearest the zero, or the plain
            # sensor angle when there is no zero.
            self._total = angle if self._zero is None else wrap180(angle - self._zero)
        else:
            step = wrap180(angle - self._last)
            self._total += step

            if abs(step) > RAIL_SUSPECT_STEP_DEG:
                self._doubt(
                    f"its gear turned {abs(step):.0f} degrees between two readings"
                )
            elif (
                time_s is not None
                and self._last_time is not None
                and time_s - self._last_time > RAIL_SUSPECT_GAP_S
            ):
                self._doubt(
                    f"two rail readings were {time_s - self._last_time:.2f} s apart"
                )

        self._last = angle
        self._last_time = time_s
        return self._total

    def _doubt(self, why: str) -> None:
        """Remember, and say once, that whole turns may have been miscounted."""
        if self.uncertain:
            return
        self.uncertain = True
        print(
            f"Warning: arm_v may now be off by whole gear turns ({why}). The "
            "rail sensor must be read at least twice per half turn of its "
            "gear: read faster (or use start() and latest()), or move the "
            "rail slower, then center again.",
            file=sys.stderr,
        )


def record_center(source: ReadingSource, previous: Center | None) -> tuple[Center, bool]:
    """Average a few readings of each encoder and return them as the center.

    Both motors must answer. The rail (AS5600) is centered too when it has a
    reading; otherwise its earlier zero, if any, is kept. Signs and the
    rail's millimetres-per-turn carry over from the previous center.

    Returns the center and whether the rail was measured this time.
    """
    # Readings from before this call belong to an earlier pose.
    source.discard_waiting()

    samples1: list[float] = []
    samples2: list[float] = []
    rail_samples: list[float] = []
    deadline = time.monotonic() + CENTER_TIMEOUT_S

    while time.monotonic() < deadline:
        reading = source.next()
        if not math.isnan(reading.motor1) and len(samples1) < CENTER_SAMPLES:
            samples1.append(reading.motor1)
        if not math.isnan(reading.motor2) and len(samples2) < CENTER_SAMPLES:
            samples2.append(reading.motor2)
        if not math.isnan(reading.as5600) and len(rail_samples) < CENTER_SAMPLES:
            rail_samples.append(reading.as5600)
        if len(samples1) >= CENTER_SAMPLES and len(samples2) >= CENTER_SAMPLES:
            break

    missing = [
        name
        for name, samples in (("motor 1", samples1), ("motor 2", samples2))
        if len(samples) < CENTER_SAMPLES
    ]
    if missing:
        raise CenterError(
            f"{' and '.join(missing)} did not answer, so the center was not saved."
        )

    rail_measured = bool(rail_samples)

    if rail_measured:
        rail: float | None = circular_mean(rail_samples)
    else:
        rail = previous.rail if previous is not None else None

    center = Center(
        sum(samples1) / len(samples1),
        sum(samples2) / len(samples2),
        previous.flip1 if previous else False,
        previous.flip2 if previous else False,
        rail,
        previous.flip_rail if previous else False,
        previous.rail_mm_per_rev if previous else None,
    )
    return center, rail_measured


def relative(value: float, zero: float, flip: bool) -> float:
    """Degrees away from the center, with the sign reversed if flipped."""
    offset = value - zero
    return -offset if flip else offset


# --------------------------------------------------------------------------
# The part to import from other code
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Angles:
    """One reading of the three encoders.

    arm_roll    wrist roll motor (MKS 1), degrees from the saved center
    arm_twist   wrist twist motor (MKS 2), degrees from the saved center
    arm_v       the linear rail: degrees of its gear from the saved center,
                counted across full turns, or millimetres when the center
                file has a millimetres-per-turn value

    gear_deg    the rail sensor's own angle, 0 to 360, never re-zeroed,
                flipped or scaled
    joy_x       joystick X, -1.0 .. +1.0 (0 = middle), nan if not available
    joy_y       joystick Y, -1.0 .. +1.0 (0 = middle), nan if not available
    button      1 = pressed, 0 = not, None if not available
    trigger     legacy trigger above 50%: 1 or 0; None if unavailable
    trigger_percent  analog trigger percentage 0..100; nan until calibrated

    With no saved center, or raw=True, arm_roll and arm_twist are the
    drivers' own angles. When no rail zero is in use (no center, a center
    saved without a rail reading, or raw=True) arm_v starts at the sensor's
    own angle instead of 0 and counts turns from there. A value is nan when
    that encoder did not answer: test with math.isnan().
    """

    time_s: float  # seconds since the stream was opened
    arm_roll: float
    arm_twist: float
    arm_v: float
    gear_deg: float = math.nan
    joy_x: float = math.nan
    joy_y: float = math.nan
    button: int | None = None
    trigger: int | None = None
    trigger_percent: float = math.nan

    # The earlier names, kept so existing code keeps working.
    @property
    def motor1(self) -> float:
        return self.arm_roll

    @property
    def motor2(self) -> float:
        return self.arm_twist

    @property
    def as5600(self) -> float:
        # As before: the sensor's own single-turn angle, not arm_v.
        return self.gear_deg


class EncoderStream:
    """A connection to the board that hands out Angles.

    port     serial port name, or None to find the board automatically
    raw      True = the drivers' own angles, ignoring any saved center
    backend  "auto", "native" or "pyserial"
    stlink   True = read through the ST-Link (SWD) instead of USB serial;
             needs the controller firmware on the board

    Raises PortError if the board cannot be found or opened, and CenterError
    if center.json exists but is damaged.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        raw: bool = False,
        backend: str = "auto",
        replace_center: bool = False,
        stlink: bool = False,
    ) -> None:
        self._raw = raw
        self._center: Center | None = None

        try:
            self._center = load_center()
        except CenterError as error:
            # A damaged file must not stop a raw stream, or its own replacement.
            if not (raw or replace_center):
                raise
            print(f"Warning: {error}", file=sys.stderr)
        self._link: Closable
        self._source: ReadingSource

        if stlink:
            source = StlinkSource()
            self._link, self._source, self.port = source, source, "ST-Link"
        else:
            self._link, self._source, self.port = connect(port, backend)
        self._start = time.perf_counter()
        self._board_zero: tuple[int, float] | None = None
        self._rail = RailTracker(self._rail_zero())
        # Whether the last set_center() measured the rail (False: its sensor
        # had no reading, so an earlier rail zero, if any, was kept).
        self.rail_was_centered = False

        self._lock = threading.Lock()
        self._latest: Angles | None = None
        self._failure: PortError | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False

    # -- context manager and iteration -------------------------------------

    def __enter__(self) -> EncoderStream:
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    def __iter__(self) -> EncoderStream:
        return self

    def __next__(self) -> Angles:
        return self.read()

    # -- center --------------------------------------------------------------

    @property
    def center(self) -> Center | None:
        """The center in use (None if none is saved)."""
        return self._center

    def _rail_zero(self) -> float | None:
        """The rail's zero angle in use (None in raw mode or if not centered)."""
        if self._raw or self._center is None:
            return None
        return self._center.rail

    @property
    def rail_uncertain(self) -> bool:
        """True once arm_v may be off by whole gear turns (center again)."""
        return self._rail.uncertain

    def _use(self, center: Center, *, new_rail_zero: bool) -> None:
        """Save a center and start using it.

        The rail's turn count starts again only when its zero was measured
        anew; a change of sign or unit keeps the turns counted so far.
        """
        save_center(center)
        self._center = center
        if new_rail_zero and not self._raw:
            self._rail = RailTracker(self._rail_zero())

    def set_center(self) -> Center:
        """Save the current pose as zero. Hold the arm still: takes about a second.

        Both motors must answer. The rail is centered too if its sensor has a
        reading: rail_was_centered says so afterwards. If it has none, the
        rail keeps its earlier zero (center.rail stays None if it never had one).
        """
        self._require_foreground("set_center()")
        center, rail_measured = record_center(self._source, self._center)
        self.rail_was_centered = rail_measured
        self._use(center, new_rail_zero=rail_measured)
        return center

    def set_flip(
        self, flip1: bool, flip2: bool, flip_rail: bool | None = None
    ) -> None:
        """Reverse the sign of arm_roll, arm_twist and/or arm_v, and save it.

        Leave flip_rail out (None) to keep the rail's saved sign.
        """
        self._require_foreground("set_flip()")
        if self._center is None:
            raise CenterError("Set a center first (set_center() or --center).")
        old = self._center
        self._use(
            Center(
                old.motor1, old.motor2, flip1, flip2,
                old.rail,
                old.flip_rail if flip_rail is None else flip_rail,
                old.rail_mm_per_rev,
            ),
            new_rail_zero=False,
        )

    def set_rail_mm_per_rev(self, mm_per_rev: float | None) -> None:
        """Rail travel for one turn of its gear, in millimetres, and save it.

        arm_v is then reported in millimetres. None goes back to degrees.
        """
        self._require_foreground("set_rail_mm_per_rev()")
        if self._center is None:
            raise CenterError("Set a center first (set_center() or --center).")
        if mm_per_rev is not None and not (
            math.isfinite(mm_per_rev) and mm_per_rev > 0
        ):
            raise CenterError("Millimetres per gear turn must be above zero.")
        old = self._center
        self._use(
            Center(
                old.motor1, old.motor2, old.flip1, old.flip2,
                old.rail, old.flip_rail, mm_per_rev,
            ),
            new_rail_zero=False,
        )

    @property
    def arm_v_unit(self) -> str:
        """'mm' or 'deg': what arm_v is measured in."""
        center = None if self._raw else self._center
        return "mm" if center is not None and center.rail_mm_per_rev else "deg"

    # -- reading -------------------------------------------------------------

    def _convert(self, reading: Reading) -> Angles:
        center = None if self._raw else self._center

        time_s = time.perf_counter() - self._start

        if reading.board_ms is not None:
            # The board stamped this sample itself: keep its spacing (the
            # mask covers the 49-day wrap of its millisecond counter).
            if self._board_zero is None:
                self._board_zero = (reading.board_ms, round(time_s, 3))
            first_ms, first_s = self._board_zero
            time_s = first_s + ((reading.board_ms - first_ms) & 0xFFFFFFFF) / 1000.0

        # The rail's gear turns many times: keep counting across turns.
        arm_v = self._rail.update(reading.as5600, time_s)

        if center is None:
            motor1, motor2 = reading.motor1, reading.motor2
        else:
            motor1 = relative(reading.motor1, center.motor1, center.flip1)
            motor2 = relative(reading.motor2, center.motor2, center.flip2)

            if center.flip_rail:
                arm_v = -arm_v
            if center.rail_mm_per_rev is not None:
                arm_v = arm_v / 360.0 * center.rail_mm_per_rev

        return Angles(
            time_s,
            motor1,
            motor2,
            arm_v,
            reading.as5600,
            joy_x=reading.joy_x,
            joy_y=reading.joy_y,
            button=reading.button,
            trigger=reading.trigger,
            trigger_percent=reading.trigger_percent,
        )

    def _require_foreground(self, what: str) -> None:
        if self._thread is not None:
            raise RuntimeError(
                f"{what} cannot be used after start(): use latest() instead."
            )

    def read(self) -> Angles:
        """Wait for the next reading and return it."""
        self._require_foreground("read()")
        return self._convert(self._source.next())

    def start(self) -> None:
        """Keep reading in the background so latest() never waits."""
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="encoder-stream", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                angles = self._convert(self._source.next())
            except PortError as error:
                with self._lock:
                    self._failure = error
                return
            except OSError as error:
                with self._lock:
                    self._failure = PortError(f"The board disconnected ({error}).")
                return

            with self._lock:
                self._latest = angles

    def latest(self) -> Angles | None:
        """The newest reading from the background reader (None until the first).

        Raises PortError if the board has disconnected.
        """
        with self._lock:
            if self._failure is not None:
                raise self._failure
            return self._latest

    def close(self) -> None:
        """Stop reading and release the port."""
        if self._closed:
            return
        self._closed = True
        self._stop.set()

        if self._thread is not None:
            self._thread.join(timeout=2.5)

        self._link.close()


# --------------------------------------------------------------------------
# Command line
# --------------------------------------------------------------------------


def fmt(value: float) -> str:
    """Format a value to three decimals ('nan' for a missing one)."""
    if math.isnan(value):
        return "nan"
    text = f"{value:.3f}"
    return "0.000" if text == "-0.000" else text


def _json_number(value: float) -> float | None:
    """JSON has no nan: a missing value becomes null."""
    return None if math.isnan(value) else round(value, 3) + 0.0


def fmt_switch(value: int | None) -> str:
    """1 / 0, or 'nan' when the firmware does not send it."""
    return "nan" if value is None else str(value)


def print_stream(encoders: EncoderStream, style: str) -> None:
    """Print readings until interrupted, as 'csv', 'labeled' or 'json'."""
    if style == "csv":
        print(HEADER, flush=True)

    for angles in encoders:
        time_ms = round(angles.time_s * 1000)

        if style == "json":
            line = json.dumps(
                {
                    "time_ms": time_ms,
                    "arm_roll": _json_number(angles.arm_roll),
                    "arm_twist": _json_number(angles.arm_twist),
                    "arm_v": _json_number(angles.arm_v),
                    "joy_x": _json_number(angles.joy_x),
                    "joy_y": _json_number(angles.joy_y),
                    "button": angles.button,
                    "trigger": angles.trigger,
                    "trigger_percent": _json_number(angles.trigger_percent),
                }
            )
        elif style == "labeled":
            line = (
                f"time_ms={time_ms:<8} arm_roll={fmt(angles.arm_roll):>9} "
                f"arm_twist={fmt(angles.arm_twist):>9} "
                f"arm_v={fmt(angles.arm_v):>9} "
                f"joy_x={fmt(angles.joy_x):>6} joy_y={fmt(angles.joy_y):>6} "
                f"button={fmt_switch(angles.button)} "
                f"trigger={fmt_switch(angles.trigger)} "
                f"trigger_percent={fmt(angles.trigger_percent)}"
            )
        else:
            line = (
                f"{time_ms},{fmt(angles.arm_roll)},"
                f"{fmt(angles.arm_twist)},{fmt(angles.arm_v)},"
                f"{fmt(angles.joy_x)},{fmt(angles.joy_y)},"
                f"{fmt_switch(angles.button)},{fmt_switch(angles.trigger)},"
                f"{fmt(angles.trigger_percent)}"
            )

        print(line, flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Stream encoder angles from the Black Pill, measured from a "
        "saved center."
    )
    parser.add_argument(
        "port", nargs="?", help="serial port, e.g. COM5 or /dev/ttyACM0 (found "
        "automatically if left out)"
    )
    parser.add_argument(
        "--center",
        action="store_true",
        help="save the current pose as the center (zero) before streaming",
    )
    parser.add_argument(
        "--flip",
        metavar="WHICH",
        help="reverse the sign of these and save it: roll, twist, v (several "
        "with commas), or none",
    )
    parser.add_argument(
        "--rail-mm-per-rev",
        type=float,
        metavar="MM",
        help="rail travel for one turn of its gear, in millimetres (saved); "
        "arm_v is then in millimetres. 0 goes back to degrees",
    )
    parser.add_argument(
        "--format",
        choices=("csv", "labeled", "json"),
        default="csv",
        help="how each reading is printed (default: csv)",
    )
    parser.add_argument(
        "--raw", action="store_true", help="print the drivers' own angles, no center"
    )
    parser.add_argument(
        "--list", action="store_true", help="list the serial ports and exit"
    )
    parser.add_argument(
        "--stlink",
        action="store_true",
        help="read through the ST-Link instead of USB (controller firmware)",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "native", "pyserial"),
        default="auto",
        help="how to open the port (default: built-in, then pyserial if installed)",
    )
    return parser.parse_args()


def install_stop_signals() -> None:
    """Turn Ctrl-C, a kill and a closed terminal into KeyboardInterrupt, so
    the port (or OpenOCD, with --stlink) is always closed on the way out."""

    def stop(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    for name in ("SIGINT", "SIGTERM", "SIGHUP"):
        number = getattr(signal, name, None)
        if number is not None:
            signal.signal(number, stop)


def main() -> int:
    args = parse_args()
    install_stop_signals()
    port_arg: str | None = args.port
    want_center: bool = args.center
    flip_arg: str | None = args.flip
    raw: bool = args.raw
    backend: str = args.backend
    style: str = args.format
    mm_per_rev: float | None = args.rail_mm_per_rev
    stlink: bool = args.stlink

    if args.list:
        ports = candidate_ports()
        print("\n".join(ports) if ports else "No serial ports found.")
        return 0

    encoders: EncoderStream | None = None

    try:
        flips = parse_flip(flip_arg) if flip_arg is not None else None

        encoders = EncoderStream(
            port_arg,
            raw=raw,
            backend=backend,
            replace_center=want_center,
            stlink=stlink,
        )
        print(f"Connected to {encoders.port}", file=sys.stderr)

        if want_center:
            center = encoders.set_center()
            if center.rail is None:
                rail_text = "not centered (the rail sensor has no reading)"
            elif encoders.rail_was_centered:
                rail_text = f"{center.rail:.3f}"
            else:
                rail_text = (
                    f"{center.rail:.3f}, kept from the earlier center "
                    "because the rail sensor has no reading now"
                )
            print(
                f"Center saved: arm_roll = {center.motor1:.3f}, "
                f"arm_twist = {center.motor2:.3f}, arm_v = {rail_text} "
                "(the encoders' own angles)",
                file=sys.stderr,
            )

        if flips is not None:
            encoders.set_flip(flips[0], flips[1], flips[2])
            flipped = [
                name
                for name, on in zip(("arm_roll", "arm_twist", "arm_v"), flips)
                if on
            ]
            print(
                "Signs reversed now: " + (", ".join(flipped) or "none")
                + " (--flip replaces the saved choice for all three).",
                file=sys.stderr,
            )

        if mm_per_rev is not None:
            encoders.set_rail_mm_per_rev(mm_per_rev if mm_per_rev != 0 else None)

        saved = encoders.center

        if saved is None and not raw:
            print(
                "No center saved yet: showing the encoders' own angles. "
                "Run with --center at the center pose.",
                file=sys.stderr,
            )
        elif saved is not None and saved.rail is None and not raw:
            print(
                "The rail has no center yet: arm_v is its sensor's own angle, "
                "not measured from the center. Run with --center while the "
                "rail sensor sees its magnet.",
                file=sys.stderr,
            )

        print(
            "arm_roll, arm_twist: degrees. arm_v: "
            + (
                "millimetres of rail travel."
                if encoders.arm_v_unit == "mm"
                else "degrees of the rail's gear (counted across turns)."
            ),
            file=sys.stderr,
        )

        print_stream(encoders, style)
    except KeyboardInterrupt:
        # End the "^C" line on the terminal, not in the CSV on stdout.
        print(file=sys.stderr)
    except OSError as error:
        # The program reading our output went away (for example `| head`).
        # Windows can report that as EINVAL instead of a broken pipe.
        closed = isinstance(error, BrokenPipeError) or (
            sys.platform == "win32" and error.errno == errno.EINVAL
        )
        if not closed:
            raise
        # Python flushes stdout again when it exits: send that nowhere, or
        # it fails a second time and the exit status becomes 120.
        try:
            os.dup2(os.open(os.devnull, os.O_WRONLY), sys.stdout.fileno())
        except (OSError, ValueError):
            pass
        return 0
    except (PortError, CenterError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    finally:
        if encoders is not None:
            encoders.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

