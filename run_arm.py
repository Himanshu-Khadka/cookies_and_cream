#!/usr/bin/env python3
"""Set up any computer and drive the SO-101 arm from the controller. One file.

Copy this file anywhere (or use it inside the repo) and run:

    python3 run_arm.py                 set up, then DRY RUN (prints, moves nothing)
    python3 run_arm.py --live          set up, then MOVE THE REAL ARM
    python3 run_arm.py --stlink        read the controller through the ST-Link
    python3 run_arm.py --setup-only    only set up
    python3 run_arm.py --live -- --no-clutch --gripper-open 100 --gripper-closed 0
                                       anything after -- goes to control_arm.py

What it does, each step only when needed:

  1. Gets the code: if this file is not inside the repo, clones it next to it.
  2. Makes a Python environment (.venv in the repo) and installs
     requirements.txt (lerobot, numpy, pyserial). Large the first time.
  3. Linux: makes sure you may use serial ports (dialout group).
  4. --stlink: makes sure OpenOCD is there.
  5. Finds the arm's port and the controller's port (asks if unsure).
  6. Starts working_code/control_arm.py with the arm calibration stored in
     the repo, so the arm does not need calibrating again.

Needs Python 3.10+ and git. Works on macOS, Linux and Windows.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_URL = "https://github.com/Himanshu-Khadka/cookies_and_cream.git"
REPO_NAME = "cookies_and_cream"

# USB ids: the controller board (STM32 Black Pill) and the arm's servo
# adapter (a WCH CH340/CH343 USB-serial chip on the SO-101 board).
BOARD_ID = (0x0483, 0x5740)
ARM_VENDORS = {0x1A86}
STLINK_VENDOR = 0x0483

HERE = Path(__file__).resolve().parent


def say(text: str) -> None:
    print(f"\n==> {text}", flush=True)


def fail(text: str) -> "None":
    print(f"\nERROR: {text}", file=sys.stderr)
    sys.exit(1)


def ask(question: str) -> bool:
    try:
        return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


def run(cmd: list[str], **kwargs: object) -> int:
    print("   $ " + " ".join(cmd), flush=True)
    return subprocess.call(cmd, **kwargs)  # type: ignore[arg-type]


# ---------------------------------------------------------------- 1. code


def find_repo() -> Path:
    for folder in (HERE, *HERE.parents):
        if (folder / "working_code" / "control_arm.py").is_file():
            return folder
    target = HERE / REPO_NAME
    if (target / "working_code" / "control_arm.py").is_file():
        return target
    if not shutil.which("git"):
        fail("git is not installed. Install it, or download the repo from GitHub.")
    say(f"Getting the code into {target}")
    if target.exists():
        if run(["git", "-C", str(target), "pull", "--ff-only"]) != 0:
            fail(f"Could not update {target}")
    elif run(["git", "clone", REPO_URL, str(target)]) != 0:
        fail("Could not clone the repo (internet? access to the GitHub repo?)")
    return target


# ---------------------------------------------------------------- 2. python


def venv_python(venv: Path) -> Path:
    return venv / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")


def has_packages(python: Path) -> bool:
    check = "import lerobot, numpy, serial"
    return subprocess.call([str(python), "-c", check],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0


def setup_python(repo: Path, venv: Path) -> Path:
    python = venv_python(venv)
    if not python.is_file():
        say(f"Making the Python environment in {venv}")
        if sys.version_info < (3, 10):
            fail("Python 3.10 or newer is needed (3.12 recommended).")
        if run([sys.executable, "-m", "venv", str(venv)]) != 0:
            fail("Could not make the environment (on Debian/Ubuntu: sudo apt install python3-venv).")
    if has_packages(python):
        say("Python packages already installed")
        return python
    say("Installing lerobot and friends (large the first time: a few minutes)")
    run([str(python), "-m", "pip", "install", "-q", "--upgrade", "pip"])
    if run([str(python), "-m", "pip", "install", "-r", str(repo / "requirements.txt")]) != 0:
        fail("Installing the packages failed (see above).")
    if not has_packages(python):
        fail("The packages installed but still cannot be imported.")
    return python


# ---------------------------------------------------------------- 3. permissions


def check_serial_permission() -> None:
    if not sys.platform.startswith("linux"):
        return
    groups = subprocess.run(["id", "-nG"], capture_output=True, text=True).stdout.split()
    if "dialout" in groups:
        return
    say("Your user may not open serial ports yet (not in the 'dialout' group)")
    user = os.environ.get("USER", "")
    if user and ask(f"Add {user} to 'dialout' now (needs sudo)?"):
        if run(["sudo", "usermod", "-aG", "dialout", user]) == 0:
            fail("Done. Log out and back in (or reboot), then run this again.")
    fail("Run:  sudo usermod -aG dialout $USER   then log out and back in.")


# ---------------------------------------------------------------- 4. openocd


def has_openocd() -> bool:
    if shutil.which("openocd"):
        return True
    for home in (Path.home() / "Library" / "Arduino15", Path.home() / ".arduino15",
                 Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "Arduino15"):
        if list((home / "packages" / "STMicroelectronics" / "tools" / "xpack-openocd").glob("*/bin/openocd*")):
            return True
    return False


def setup_openocd() -> None:
    if has_openocd():
        return
    say("OpenOCD is needed to read the controller through the ST-Link")
    if sys.platform == "darwin" and shutil.which("brew") and ask("Install it with Homebrew now?"):
        if run(["brew", "install", "open-ocd"]) == 0:
            return
    if sys.platform.startswith("linux") and shutil.which("apt-get") and ask("Install it with apt now (sudo)?"):
        if run(["sudo", "apt-get", "install", "-y", "openocd"]) == 0:
            return
    fail("Install OpenOCD (macOS: brew install open-ocd, Linux: sudo apt install openocd, "
         "Windows: xpack-openocd), or plug the controller's USB-C in and leave out --stlink.")


# ---------------------------------------------------------------- 5. ports


def list_ports(python: Path) -> list[dict[str, object]]:
    code = ("import json, serial.tools.list_ports as l; print(json.dumps([{'device': p.device, "
            "'vid': p.vid, 'pid': p.pid, 'desc': p.description} for p in l.comports()]))")
    out = subprocess.run([str(python), "-c", code], capture_output=True, text=True)
    try:
        ports: list[dict[str, object]] = json.loads(out.stdout)
    except ValueError:
        return []
    # macOS lists each port twice (cu. and tty.): keep cu.
    return [p for p in ports if "/dev/tty." not in str(p["device"])]


def choose(kind: str, found: list[dict[str, object]], others: list[dict[str, object]]) -> str:
    if len(found) == 1:
        return str(found[0]["device"])
    options = found or others
    if not options:
        fail(f"No {kind} found. Is it plugged in (and powered)?")
    print(f"\nWhich port is the {kind}?")
    for i, p in enumerate(options, 1):
        print(f"  {i}. {p['device']}  ({p['desc']})")
    try:
        pick = int(input("Number: ")) - 1
        return str(options[pick]["device"])
    except (ValueError, IndexError, EOFError):
        fail("No port chosen.")
    return ""


def find_ports(python: Path, need_board: bool) -> tuple[str, str | None]:
    ports = list_ports(python)
    board = [p for p in ports if (p["vid"], p["pid"]) == BOARD_ID]
    arm = [p for p in ports if p["vid"] in ARM_VENDORS]
    usb_other = [p for p in ports if p["vid"] is not None and p not in board
                 and p["vid"] != STLINK_VENDOR]

    arm_port = choose("arm (SO-101)", arm, usb_other)
    board_port = None
    if need_board:
        board_port = choose("controller board (Black Pill USB-C)", board,
                            [p for p in usb_other if p["device"] != arm_port])
    return arm_port, board_port


# ---------------------------------------------------------------- 6. run


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--live", action="store_true", help="move the real arm (default: dry run)")
    p.add_argument("--stlink", action="store_true", help="read the controller through the ST-Link")
    p.add_argument("--setup-only", action="store_true", help="set up, do not run")
    p.add_argument("--arm-port", help="skip finding the arm's port")
    p.add_argument("--encoder-port", help="skip finding the controller's port")
    p.add_argument("--venv", type=Path, help="use this Python environment (default: <repo>/.venv)")
    p.add_argument("extra", nargs=argparse.REMAINDER, help="after --: options for control_arm.py")
    args = p.parse_args()
    extra = [a for a in args.extra if a != "--"]

    repo = find_repo()
    python = setup_python(repo, args.venv or repo / ".venv")
    check_serial_permission()
    if args.stlink:
        setup_openocd()
    if args.setup_only:
        say(f"Set up. Run again without --setup-only to start. (Python: {python})")
        return 0

    command = [str(python), str(repo / "working_code" / "control_arm.py")]

    if args.live:
        need_board = not args.stlink and not args.encoder_port
        if args.arm_port:
            arm_port, board_port = args.arm_port, None
            if need_board:
                _, board_port = find_ports(python, True)
        else:
            arm_port, board_port = find_ports(python, need_board)
        command += ["--live", "--arm-port", arm_port, "--calibration-dir", str(repo / "calibration_file")]
    else:
        board_port = None
        if not args.stlink and not args.encoder_port:
            ports = list_ports(python)
            board = [q for q in ports if (q["vid"], q["pid"]) == BOARD_ID]
            board_port = choose("controller board (Black Pill USB-C)", board, [])

    if args.stlink:
        command.append("--stlink")
    elif args.encoder_port or board_port:
        command += ["--encoder-port", args.encoder_port or str(board_port)]
    command += extra

    say("Starting the arm controller" + (" (LIVE: hand on the arm's power)" if args.live else " (dry run)"))
    try:
        return run(command)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
