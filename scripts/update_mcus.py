#!/usr/bin/env python3
# Rebuild this printer's micro-controllers with the Klipper checked out here
# and flash the ones that run another version, through Katapult.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""Rebuild and flash a printer's MCUs after a Klipper update.

Run it on the printer with Klipper's own Python (it has pyserial, which
Katapult's flashtool needs), after updating Klipper:

    ~/klippy-env/bin/python ~/klipper/scripts/update_mcus.py            # build, compare, ask, flash
    ~/klippy-env/bin/python ~/klipper/scripts/update_mcus.py --dry-run  # build and compare only
    ... --yes            flash without asking
    ... --only mcu,ebb36 just these boards (names from the config file)
    ... --force          flash a board even when it already runs this version
    ... --config FILE    default ~/printer_data/config/mcu_update.cfg

What it does:
 1. Refuses while a print is printing or paused.
 2. Builds each board from its own saved `make menuconfig` file into
    ~/klipper_mcu_builds/<board>/ (new Kconfig options take their defaults;
    the saved file is never changed).
 3. Compares each build's version with the version the board reports to
    Klipper, and lists what it will flash.
 4. Stops Klipper, flashes each board that differs, starts Klipper again
    (with a FIRMWARE_RESTART if an MCU shut down while it was stopped), and
    checks every board now reports the built version.

The config file lists the boards, one section each:

    [settings]
    katapult: ~/katapult                  # a Katapult checkout (scripts/flashtool.py)
    moonraker: http://127.0.0.1:7125

    [board octopus]
    klipper_name: mcu                     # the Klipper [mcu <name>] it is ("mcu" for [mcu])
    build_config: mcu_update/octopus.config   # relative to the config file
    flash: katapult_usb
    device: /dev/serial/by-id/usb-Klipper_stm32f446xx_...-if00

    [board ebb36]
    klipper_name: ebb36
    build_config: mcu_update/ebb36.config
    flash: katapult_can
    interface: can0
    uuid: f75ae74f839d

    [board host]                          # the Linux process MCU
    klipper_name: host
    build_config: mcu_update/host.config
    flash: linux                          # `make flash`: sudo installs klipper_mcu

flash methods:
  katapult_usb  a USB board with Katapult: asks the running Klipper firmware
                for the bootloader, waits for the Katapult device (the
                Klipper by-id name with usb-katapult_, or katapult_device:),
                flashes, waits for the board to come back as Klipper. The
                device's by-id name must name the build's MCU (stm32f446xx),
                so a misconfigured path never flashes the wrong board.
  katapult_can  a CAN node with Katapult: flashtool -i <interface> -u <uuid>.
                Not for the board that is itself the USB-to-CAN bridge.
  linux         the host MCU.
"""
import argparse
import configparser
import json
import os
import shutil
import subprocess
import sys
import time
import urllib.parse
import urllib.request
from pathlib import Path

KLIPPER_DIR = Path(__file__).resolve().parent.parent
DEFAULT_CONFIG = Path("~/printer_data/config/mcu_update.cfg").expanduser()
BUILD_ROOT = Path("~/klipper_mcu_builds").expanduser()
BUSY_STATES = ("printing", "paused")
METHODS = ("katapult_usb", "katapult_can", "linux")


class UpdateError(Exception):
    pass


def say(msg=""):
    print(msg, flush=True)


# ------------------------------------------------------------------ Moonraker

class Moonraker:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def _call(self, path, method="GET", timeout=10.0):
        req = urllib.request.Request(self.url + path, method=method)
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.load(resp)["result"]

    def query(self, objects):
        q = "&".join(urllib.parse.quote(o) for o in objects)
        return self._call("/printer/objects/query?" + q)["status"]

    def state(self):
        try:
            return self._call("/printer/info")["state"]
        except Exception:
            return None

    def service(self, action):
        self._call("/machine/services/%s?service=klipper" % action, "POST",
                   timeout=60.0)

    def firmware_restart(self):
        self._call("/printer/firmware_restart", "POST", timeout=30.0)

    def wait_ready(self, timeout=90.0):
        """Wait for Klipper to be ready; one FIRMWARE_RESTART when it comes up
        in error or shutdown (an MCU that shut down while Klipper was stopped,
        such as one with a host watchdog)."""
        deadline = time.time() + timeout
        restarted = False
        while time.time() < deadline:
            state = self.state()
            if state == "ready":
                return True
            if state in ("error", "shutdown") and not restarted:
                time.sleep(2.0)
                if self.state() in ("error", "shutdown"):
                    say("  Klipper is in %s; sending FIRMWARE_RESTART" % state)
                    self.firmware_restart()
                    restarted = True
            time.sleep(2.0)
        return False


def object_name(klipper_name):
    return "mcu" if klipper_name == "mcu" else "mcu " + klipper_name


# --------------------------------------------------------------------- boards

class Board:
    def __init__(self, name, section, config_dir):
        self.name = name
        get = section.get
        self.klipper_name = get("klipper_name", name)
        build_config = get("build_config")
        if not build_config:
            raise UpdateError("[board %s] needs build_config" % name)
        self.build_config = (config_dir / os.path.expanduser(build_config)).resolve()
        self.flash = get("flash", "")
        if self.flash not in METHODS:
            raise UpdateError("[board %s] flash: must be one of %s"
                              % (name, ", ".join(METHODS)))
        self.device = get("device")
        self.katapult_device = get("katapult_device")
        self.interface = get("interface", "can0")
        self.uuid = get("uuid")
        if self.flash == "katapult_usb":
            if not self.device:
                raise UpdateError("[board %s] katapult_usb needs device" % name)
            if not self.katapult_device:
                if "usb-Klipper_" not in self.device:
                    raise UpdateError("[board %s] give katapult_device: (device"
                                      " is not a usb-Klipper_ by-id name)" % name)
                self.katapult_device = self.device.replace(
                    "usb-Klipper_", "usb-katapult_")
        if self.flash == "katapult_can" and not self.uuid:
            raise UpdateError("[board %s] katapult_can needs uuid" % name)
        self.out = BUILD_ROOT / name
        self.version = None     # built
        self.mcu = None         # built MCU type
        self.running = None     # reported by Klipper

    @property
    def binary(self):
        return self.out / ("klipper.elf" if self.flash == "linux"
                           else "klipper.bin")


def make(board, *targets, log=None):
    cmd = ["make", "-C", str(KLIPPER_DIR),
           "KCONFIG_CONFIG=%s" % (board.out / "config"),
           "OUT=%s/" % board.out] + list(targets)
    with open(log, "a") if log else open(os.devnull, "w") as out:
        return subprocess.run(cmd, stdout=out, stderr=subprocess.STDOUT).returncode


def build(board, jobs):
    if not board.build_config.is_file():
        raise UpdateError("%s: no build config at %s" % (board.name, board.build_config))
    if board.out.exists():
        shutil.rmtree(board.out)
    board.out.mkdir(parents=True)
    shutil.copyfile(board.build_config, board.out / "config")
    log = board.out / "build.log"
    if make(board, "olddefconfig", log=log) != 0:
        raise UpdateError("%s: olddefconfig failed, see %s" % (board.name, log))
    added = sorted(set(_options(board.out / "config"))
                   - set(_options(board.build_config)))
    if make(board, "-j%d" % jobs, log=log) != 0:
        tail = log.read_text().splitlines()[-15:]
        raise UpdateError("%s: build failed:\n    %s"
                          % (board.name, "\n    ".join(tail)))
    info = json.loads((board.out / "klipper.dict").read_text())
    board.version = info.get("version")
    board.mcu = (info.get("config") or {}).get("MCU")
    say("  %-8s built %s for %s%s" % (
        board.name, board.version, board.mcu,
        ("; new options at their defaults: " + ", ".join(added)) if added else ""))


def _options(path):
    return [line.split("=", 1)[0] for line in path.read_text().splitlines()
            if line.startswith("CONFIG_") and "=" in line]


def toolchain_ok(boards):
    if any(b.flash != "linux" for b in boards) and not shutil.which("arm-none-eabi-gcc"):
        raise UpdateError(
            "arm-none-eabi-gcc is not installed. Install it once with:\n"
            "    sudo apt install gcc-arm-none-eabi binutils-arm-none-eabi"
            " libnewlib-arm-none-eabi")


# --------------------------------------------------------------------- flash

def flashtool(katapult, args, timeout=300):
    cmd = [sys.executable, str(katapult / "scripts" / "flashtool.py")] + args
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          timeout=timeout)
    text = proc.stdout.decode("utf-8", "replace")
    printable = "".join(c for c in text if c == "\n" or c.isprintable())
    return proc.returncode, printable


def wait_for(path, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline:
        if os.path.exists(path):
            return True
        time.sleep(0.5)
    return False


def flash_usb(board, katapult):
    if board.mcu and board.mcu not in board.device:
        raise UpdateError("%s: the device %s does not name %s; refusing to flash it"
                          % (board.name, board.device, board.mcu))
    if not os.path.exists(board.katapult_device):
        if not os.path.exists(board.device):
            raise UpdateError("%s: neither %s nor %s is present"
                              % (board.name, board.device, board.katapult_device))
        rc, out = flashtool(katapult, ["-d", board.device, "-r"], timeout=60)
        if not wait_for(board.katapult_device, 15):
            raise UpdateError("%s: no Katapult device after the bootloader request"
                              " (is Katapult installed?)\n%s" % (board.name, out.strip()))
    rc, out = flashtool(katapult, ["-d", board.katapult_device, "-f", str(board.binary)])
    _check_flash(board, rc, out)
    if not wait_for(board.device, 20):
        raise UpdateError("%s: did not come back as %s after flashing"
                          % (board.name, board.device))


def flash_can(board, katapult):
    rc, out = flashtool(katapult, ["-i", board.interface, "-u", board.uuid,
                                   "-f", str(board.binary)])
    _check_flash(board, rc, out)


def flash_linux(board):
    log = board.out / "flash.log"
    if make(board, "flash", log=log) != 0:
        raise UpdateError("%s: make flash failed, see %s" % (board.name, log))


def _check_flash(board, rc, out):
    if rc != 0 or "Programming Complete" not in out:
        raise UpdateError("%s: flashing failed (exit %d):\n%s"
                          % (board.name, rc, out.strip()))
    sha = [l.strip() for l in out.splitlines() if "Verification Complete" in l]
    say("  %-8s flashed%s" % (board.name, (" (%s)" % sha[0]) if sha else ""))


# ----------------------------------------------------------------------- main

def load_config(path):
    if not path.is_file():
        raise UpdateError("no config at %s (see the top of %s)" % (path, __file__))
    cp = configparser.ConfigParser(inline_comment_prefixes=("#", ";"))
    cp.read(path)
    settings = cp["settings"] if cp.has_section("settings") else {}
    katapult = Path(os.path.expanduser(settings.get("katapult", "~/katapult")))
    moonraker = settings.get("moonraker", "http://127.0.0.1:7125")
    boards = [Board(s.split(None, 1)[1].strip(), cp[s], path.parent)
              for s in cp.sections() if s.startswith("board ")]
    if not boards:
        raise UpdateError("%s lists no [board <name>] sections" % path)
    return katapult, Moonraker(moonraker), boards


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--only", help="comma-separated board names")
    ap.add_argument("--dry-run", action="store_true", help="build and compare only")
    ap.add_argument("--force", action="store_true",
                    help="flash boards that already run the built version")
    ap.add_argument("--yes", action="store_true", help="do not ask")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    args = ap.parse_args(argv)

    katapult, moon, boards = load_config(args.config.expanduser())
    if args.only:
        wanted = {n.strip() for n in args.only.split(",") if n.strip()}
        unknown = wanted - {b.name for b in boards}
        if unknown:
            raise UpdateError("no board named %s" % ", ".join(sorted(unknown)))
        boards = [b for b in boards if b.name in wanted]
    if any(b.flash.startswith("katapult") for b in boards) and \
            not (katapult / "scripts" / "flashtool.py").is_file():
        raise UpdateError("no Katapult flashtool at %s" % katapult)
    toolchain_ok(boards)

    state = moon.state()
    status = {}
    if state == "ready":
        status = moon.query(["print_stats"] + [object_name(b.klipper_name)
                                              for b in boards])
        print_state = (status.get("print_stats") or {}).get("state")
        if print_state in BUSY_STATES:
            raise UpdateError("the printer is %s; update the MCUs when it is idle"
                              % print_state)
    else:
        say("Klipper is %s: the boards' running versions are unknown" % state)
    for b in boards:
        b.running = (status.get(object_name(b.klipper_name)) or {}).get("mcu_version")

    say("Klipper %s" % subprocess.run(
        ["git", "-C", str(KLIPPER_DIR), "describe", "--always", "--tags",
         "--long", "--dirty"], stdout=subprocess.PIPE).stdout.decode().strip())
    say("Building:")
    for b in boards:
        build(b, args.jobs)

    todo = [b for b in boards if args.force or b.running != b.version]
    say("Boards:")
    for b in boards:
        say("  %-8s runs %-28s built %-28s %s" % (
            b.name, b.running or "(unknown)", b.version,
            "FLASH" if b in todo else "up to date"))
    if not todo:
        say("Nothing to flash.")
        return 0
    if args.dry_run:
        say("Dry run: nothing flashed.")
        return 0
    if not args.yes:
        if input("Stop Klipper and flash %s? [y/N] "
                 % ", ".join(b.name for b in todo)).strip().lower() != "y":
            say("Nothing flashed.")
            return 1

    say("Stopping Klipper")
    moon.service("stop")
    time.sleep(3.0)
    failed = None
    try:
        for b in todo:
            if b.flash == "katapult_usb":
                flash_usb(b, katapult)
            elif b.flash == "katapult_can":
                flash_can(b, katapult)
            else:
                flash_linux(b)
    except (UpdateError, subprocess.TimeoutExpired) as e:
        failed = e
    finally:
        say("Starting Klipper")
        moon.service("start")
        ready = moon.wait_ready()
    if failed:
        raise UpdateError("%s\nThe boards after it were not flashed." % failed)
    if not ready:
        raise UpdateError("Klipper is not ready after the update (state %s)"
                          % moon.state())
    status = moon.query([object_name(b.klipper_name) for b in boards])
    bad = []
    say("Now:")
    for b in boards:
        now = (status.get(object_name(b.klipper_name)) or {}).get("mcu_version")
        ok = now == b.version
        say("  %-8s %s%s" % (b.name, now, "" if ok else "   (expected %s)" % b.version))
        if not ok:
            bad.append(b.name)
    if bad:
        raise UpdateError("not on the built version: %s" % ", ".join(bad))
    say("Done.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except UpdateError as e:
        print("update_mcus: %s" % e, file=sys.stderr)
        sys.exit(2)
