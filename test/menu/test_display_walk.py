# Walk the printer's LCD menu on the real display stack, as the screen shows it
#
# klippy/extras/display/display.py (PrinterLCD) and menu.py run with this
# tree's menu.cfg and display.cfg, plus an owner's menu file and OpenAMS's
# oams_menu.cfg when given. Behind them sits a headless UC1701 (128x64, the
# 16x4 text grid Klipper uses on a mini12864): the real DisplayBase
# framebuffer, font and flush, with its SPI writes applied to an emulated
# controller RAM. Every screen below is read back from that RAM, so it is
# what the panel shows. Input goes through the callbacks menu_keys registers
# with the buttons module (the encoder's two directions and the click pin),
# so a long press is the real 0.8 s timer.
#
# The knob: menu_keys maps Klipper's encoder_cw_callback to "up" and
# encoder_ccw_callback to "down"; which physical direction fires which
# depends on the order of encoder_pins. On voron350's mini12864
# (encoder_pins: ^EXP2_5, ^EXP2_3, as in the fixture) a physical clockwise
# turn fires encoder_ccw_callback: clockwise moves the cursor down,
# counter-clockwise up. Bench.turn() counts physical clockwise detents.
#
# Run (from the top of the tree, with the Python Klipper runs on):
#   ~/klippy-env/bin/python -m unittest discover -s test/menu -p 'test_display_walk.py' -v
#   ~/klippy-env/bin/python test/menu/test_display_walk.py --show
#
# Configs (defaults are the fixtures next to this file):
#   OWNER_PRINTER_CFG  a printer.cfg whose [display] section is used
#   OWNER_MENU_CFG     the owner's menu file, included before oams_menu.cfg
#   OAMS_MENU_CFG      OpenAMS's oams_menu.cfg; the OpenAMS walks need it
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import contextlib
import logging
import os
import re
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
KLIPPY = os.path.abspath(os.path.join(HERE, "..", "..", "klippy"))
sys.path.insert(0, KLIPPY)

import configfile                                   # noqa: E402
import gcode as gcode_module                        # noqa: E402
from extras import gcode_macro, display_status      # noqa: E402
from extras.display import display, uc1701, font8x14  # noqa: E402

FIXTURES = os.path.join(HERE, "fixtures")
OWNER_PRINTER_CFG = os.environ.get(
    "OWNER_PRINTER_CFG", os.path.join(FIXTURES, "mini12864_display.cfg"))
OWNER_MENU_CFG = os.environ.get(
    "OWNER_MENU_CFG", os.path.join(FIXTURES, "owner_menu.cfg"))
OAMS_MENU_CFG = os.environ.get("OAMS_MENU_CFG")
SHOW = False


# ---------------------------------------------------------------------------
# A deterministic reactor
# ---------------------------------------------------------------------------

class Timer:
    def __init__(self, callback, waketime, oneshot=False):
        self.callback = callback
        self.waketime = waketime
        self.oneshot = oneshot


class Reactor:
    NOW = 0.
    NEVER = 9.9e99

    def __init__(self):
        self.now = 1000.
        self.timers = []

    def monotonic(self):
        return self.now

    def register_timer(self, callback, waketime=NEVER):
        timer = Timer(callback, waketime)
        self.timers.append(timer)
        return timer

    def update_timer(self, timer, waketime):
        timer.waketime = waketime

    def unregister_timer(self, timer):
        if timer in self.timers:
            self.timers.remove(timer)

    def register_callback(self, callback, waketime=NOW):
        def run(eventtime):
            callback(eventtime)
            return self.NEVER
        timer = Timer(run, waketime, oneshot=True)
        self.timers.append(timer)
        return timer

    def mutex(self):
        return contextlib.nullcontext()

    def assert_no_pause(self):
        return contextlib.nullcontext()

    def advance(self, seconds):
        """Run every timer that falls due within `seconds`, in order. A
        timer callback that raised would end this run: on a printer an
        exception out of a reactor timer shuts Klipper down."""
        end = self.now + seconds
        while True:
            due = [t for t in self.timers if t.waketime <= end]
            if not due:
                break
            timer = min(due, key=lambda t: t.waketime)
            self.now = max(self.now, timer.waketime)
            timer.waketime = timer.callback(self.now)
            if timer.oneshot:
                self.timers.remove(timer)
        self.now = end


# ---------------------------------------------------------------------------
# The headless UC1701
# ---------------------------------------------------------------------------

class ControllerRAM:
    """What a UC1701 does with the bytes Klipper sends it: page and column
    address commands, then data written at the column, which advances."""

    def __init__(self):
        self.pages = [bytearray(132) for _ in range(8)]
        self.page = self.column = 0
        self.skip = 0
        self.flushes = 0

    def send(self, data, is_data=False):
        if is_data:
            for byte in bytearray(data):
                if self.column < 132:
                    self.pages[self.page][self.column] = byte
                self.column += 1
            return
        for byte in bytearray(data):
            if self.skip:                   # the value after 0x81 / 0xF8
                self.skip -= 1
            elif byte in (0x81, 0xF8):
                self.skip = 1
            elif byte & 0xF0 == 0xB0:
                self.page = byte & 0x0F
            elif byte & 0xF0 == 0x10:
                self.column = (self.column & 0x0F) | ((byte & 0x0F) << 4)
            elif byte & 0xF0 == 0x00:
                self.column = (self.column & 0xF0) | (byte & 0x0F)


class HeadlessUC1701(uc1701.DisplayBase):
    """lcd_type: uc1701 with no MCU: it reads the options the real driver
    reads (so the owner's [display] section passes Klipper's check), keeps
    the real framebuffer, font and flush, and sends to ControllerRAM."""

    def __init__(self, config):
        config.get('cs_pin')
        config.get('a0_pin')
        config.get('rst_pin', None)
        config.getint('spi_speed', 10000000, minval=100000)
        for name in ('spi_software_sclk_pin', 'spi_software_mosi_pin',
                     'spi_software_miso_pin', 'spi_bus'):
            config.get(name, None)
        self.contrast = config.getint('contrast', 40, minval=0, maxval=63)
        self.ram = ControllerRAM()
        uc1701.DisplayBase.__init__(self, self.ram)
        cells = {}
        for code, (top, bot) in enumerate(self.font):
            cells[(bytes(top), bytes(bot))] = chr(code) if 32 <= code < 127 \
                else '#'
        self.cells = cells

    def init(self):
        self.send([0xE2, 0x40, 0xA0, 0xC8, 0xA2, 0x2C, 0x2E, 0x2F, 0xF8, 0x00,
                   0x23, 0x81, self.contrast, 0xAC, 0x00, 0xA6, 0xAF])
        self.flush()

    def flush(self):
        uc1701.DisplayBase.flush(self)
        self.ram.flushes += 1

    # XOR patterns write_graphics leaves over text (draw_progress_bar): the
    # bar's frame (top and bottom pixel rows, and its end columns), filled
    # or not. A cell under the bar is read through them.
    # Per pixel column the bar is either its frame alone (top and bottom
    # pixel) or filled; the filled columns of a cell form a run, plus the
    # bar's end columns.
    MASKS = None

    @staticmethod
    def bar_masks():
        runs = set()
        for a in range(9):
            for b in range(a, 9):
                for ends in ((), (0,), (7,), (0, 7)):
                    runs.add(frozenset(range(a, b)) | frozenset(ends))
        return [(bytes(0xff if c in filled else 0x01 for c in range(8)),
                 bytes(0xff if c in filled else 0x80 for c in range(8)))
                for filled in sorted(runs, key=sorted)]

    def text_rows(self):
        """The 16x4 text grid as the panel shows it. A cell under the
        progress bar is read through the bar; a cell that is no character
        of the font (an icon) shows as '%'."""
        rows = []
        for row in range(4):
            top, bot = self.ram.pages[row * 2], self.ram.pages[row * 2 + 1]
            line = ""
            for col in range(16):
                key = (bytes(top[col * 8:col * 8 + 8]),
                       bytes(bot[col * 8:col * 8 + 8]))
                if not any(key[0]) and not any(key[1]):
                    line += " "
                    continue
                char = self.cells.get(key)
                if HeadlessUC1701.MASKS is None:
                    HeadlessUC1701.MASKS = self.bar_masks()
                if char is None:
                    # Under the bar: a printable character (or a blank)
                    # wins over a block-like glyph a wrong mask would give.
                    for mtop, mbot in self.MASKS:
                        under = (bytes(a ^ m for a, m in zip(key[0], mtop)),
                                 bytes(a ^ m for a, m in zip(key[1], mbot)))
                        found = (" " if not any(under[0]) and not any(under[1])
                                 else self.cells.get(under))
                        if found is not None and found != "#":
                            char = found
                            break
                line += char if char is not None else "%"
            rows.append(line)
        return rows


# ---------------------------------------------------------------------------
# The printer around it
# ---------------------------------------------------------------------------

# Klipper's toolhead and gcode_move publish positions as a namedtuple.
Coord = collections.namedtuple("Coord", ("x", "y", "z", "e"))


class Status:
    def __init__(self, **values):
        self.values = values

    def get_status(self, eventtime):
        return dict(self.values)


class SDCard(Status):
    def __init__(self, files):
        super(SDCard, self).__init__(file_path=None, is_active=False,
                                     progress=0., file_position=0)
        self.files = list(files)

    def is_active(self):
        return self.values["is_active"]

    def get_file_list(self, check_subdirs=False, sort_by_date=False):
        return [(name, 1000) for name in self.files]


class OpenAMS(Status):
    """printer.oams_manager as the menu reads it: voron350's layout, an
    AMS 1 (T0-T3) and an AMS 2 Pro with a dryer (T4-T7) on one lane."""

    def __init__(self):
        super(OpenAMS, self).__init__()
        self.settings = {"read_tag_on_insertion": True,
                         "read_tags_at_startup": True,
                         "auto_reload_on_runout": True,
                         "auto_create_spoolman_spools": True,
                         "apply_pa_on_load": False,
                         "sync_unit_tray_info": True}
        self.running = False
        self.broken = False

    def get_status(self, eventtime):
        bays = lambda actions: [
            {"index": i, "ready": True, "loaded": False,
             "calibration_state": "configured", "positioning": False,
             "supported_actions": list(actions)} for i in range(4)]
        units = [{"id": str(idx), "name": name, "lane": "fps", "kind": "oams",
                  "slots": [{"id": base + i, "bay": i, "ready": True,
                             "loaded": False} for i in range(4)]}
                 for idx, name, base in ((1, "ams1", 0), (2, "ams2", 4))]
        status = {
            "ready": True,
            "topology": {
                "fps": ["fps"],
                "oams": {"ams1": {"idx": 1, "lane": "fps", "family": "ams1"},
                         "ams2": {"idx": 2, "lane": "fps", "family": "ams2"}},
                "groups": dict(
                    ("T%d" % (u * 4 + b),
                     {"lane": "fps", "bays": ["ams%d-%d" % (u + 1, b)]})
                    for u in range(2) for b in range(4))},
            "groups": [{"name": "T%d" % (u * 4 + b), "lane": "fps",
                        "next_bay": [u + 1, b]}
                       for u in range(2) for b in range(4)],
            "units": units,
            "devices": {
                "ams1": {"bays": bays(["load", "calibrate_ptfe"]),
                         "supported_actions": ["clear_errors", "load",
                                               "unload", "follower",
                                               "calibrate_ptfe"],
                         "dryer": None, "capabilities": {"dryer": False},
                         "faults": []},
                "ams2": {"bays": bays(["load", "calibrate_ptfe"]),
                         "supported_actions": ["clear_errors", "load",
                                               "unload", "follower",
                                               "calibrate_ptfe", "dryer_start",
                                               "dryer_stop"],
                         "dryer": {"state": "off", "target_c": None,
                                   "remaining_s": None, "adapter": True,
                                   "fault": "none"},
                         "capabilities": {"dryer": True,
                                          "dryer_target_min_c": 45,
                                          "dryer_target_max_c": 65},
                         "faults": []}},
            "lanes_by_fps": {"fps": {
                "op": "UNLOADED", "group": None, "unit": None,
                "current_slot": None, "motion_settled": not self.running,
                "runout": "idle", "following": False,
                "stage": ({"plan": ["heat", "feed"], "index": 0,
                           "failed": None} if self.running else None)}},
            "settings": dict(self.settings)}
        if self.broken:
            # A status key missing for a moment (a unit reconnecting, say):
            # every enable that reads the lanes fails while it lasts.
            del status["lanes_by_fps"]
        return status


class GCode:
    error = gcode_module.CommandError

    def __init__(self, printer):
        self.printer = printer
        self.scripts = []
        self.messages = []

    def register_command(self, *args, **kwargs):
        pass

    def register_mux_command(self, *args, **kwargs):
        pass

    def respond_info(self, msg, log=True):
        self.messages.append(msg)

    def run_script(self, script):
        for line in script.split("\n"):
            line = line.strip()
            if line:
                self.scripts.append(line)
                self.printer.simulate(line)

    run_script_from_command = run_script


class ConfigFile(Status):
    def __init__(self, printer):
        super(ConfigFile, self).__init__(
            save_config_pending=False, settings={},
            config={"extruder": {"max_temp": "300",
                                 "max_extrude_only_distance": "100"},
                    "heater_bed": {"max_temp": "120"}})
        self.printer = printer

    def read_config(self, filename):
        reader = configfile.ConfigFileReader()
        fileconfig = reader.build_fileconfig(
            reader.read_config_file(filename), filename)
        return configfile.ConfigWrapper(self.printer, fileconfig, {},
                                        "printer")


class Buttons:
    """The buttons module as menu_keys uses it: keep the callbacks the
    real encoder and click pin would call."""

    def __init__(self):
        self.encoder = None
        self.click = None

    def register_rotary_encoder(self, pin1, pin2, cw_callback, ccw_callback,
                                steps_per_detent):
        self.encoder = (cw_callback, ccw_callback)
        self.encoder_pins = (pin1, pin2)

    def register_buttons(self, pins, callback):
        self.click = callback

    def register_button_push(self, pin, callback):
        pass


class Printer:
    config_error = configfile.error
    command_error = gcode_module.CommandError

    def __init__(self, files):
        self.reactor = Reactor()
        self.events = {}
        self.objects = {}
        self.objects["gcode"] = self.gcode = GCode(self)
        self.objects["configfile"] = ConfigFile(self)
        self.objects["buttons"] = Buttons()
        self.objects["idle_timeout"] = Status(state="Idle", printing_time=0.)
        self.objects["print_stats"] = Status(state="standby", filename="")
        self.objects["pause_resume"] = Status(is_paused=False)
        self.objects["virtual_sdcard"] = SDCard(files)
        self.objects["toolhead"] = Status(
            homed_axes="xyz", position=Coord(175., 175., 10., 0.),
            axis_minimum=Coord(0., 0., 0., 0.), axis_maximum=Coord(350., 350., 340., 0.),
            extruder="extruder", estimated_print_time=0.)
        self.objects["gcode_move"] = Status(
            speed_factor=1., extrude_factor=1.,
            homing_origin=Coord(0., 0., 0., 0.), gcode_position=Coord(175., 175., 10., 0.))
        self.objects["extruder"] = Status(temperature=24., target=0.,
                                          can_extrude=False)
        self.objects["heater_bed"] = Status(temperature=23., target=0.)
        self.objects["fan"] = Status(speed=0.)
        self.objects["heaters"] = Status(
            available_heaters=["extruder", "heater_bed"])
        self.objects["heater_generic the_filter"] = Status(
            temperature=25., target=0.)
        self.objects["quad_gantry_level"] = Status(applied=False)
        self.objects["bed_mesh"] = Status(profile_name="")

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown object '%s'" % (name,))
        return default

    def lookup_objects(self, module=None):
        return [(name, obj) for name, obj in self.objects.items()
                if module is None or name.split()[0] == module]

    def load_object(self, config, name):
        return self.objects[name]

    def add_object(self, name, obj):
        self.objects[name] = obj

    def register_event_handler(self, name, callback):
        self.events.setdefault(name, []).append(callback)

    def send_event(self, name, *args):
        return [cb(*args) for cb in self.events.get(name, [])]

    def state(self, state):
        """The print as Mainsail or the print file would leave it."""
        o = self.objects
        o["print_stats"].values["state"] = state
        o["pause_resume"].values["is_paused"] = state == "paused"
        o["idle_timeout"].values["state"] = (
            "Printing" if state == "printing" else "Ready")
        sd = o["virtual_sdcard"].values
        sd["is_active"] = state == "printing"
        if state in ("standby", "complete", "cancelled"):
            sd["file_path"] = None

    def simulate(self, line):
        """The few commands whose effect the walks look at."""
        word = line.split()[0]
        if word == "SDCARD_PRINT_FILE":
            name = re.search(r'FILENAME="([^"]*)"', line).group(1)
            self.objects["virtual_sdcard"].values["file_path"] = "/gcodes/" + name
            self.objects["print_stats"].values["filename"] = name
            self.state("printing")
        elif word == "PAUSE":
            self.state("paused")
        elif word == "RESUME":
            self.state("printing")
        elif word == "CANCEL_PRINT":
            self.state("cancelled")
        elif word == "OAMSM_SET_OPTION" and "oams_manager" in self.objects:
            params = dict(p.split("=", 1) for p in line.split()[1:])
            self.objects["oams_manager"].settings[params["NAME"]] = \
                params["VALUE"] == "1"


# ---------------------------------------------------------------------------
# The bench: the real display stack, and the panel's knob
# ---------------------------------------------------------------------------

def display_section(printer_cfg):
    """The [display] section of a printer.cfg, as text."""
    text = open(printer_cfg).read()
    match = re.search(r"^\[display\]\n(.*?)(?=^\[)", text + "\n[end]",
                      re.M | re.S)
    if match is None:
        raise unittest.SkipTest("no [display] section in %s" % printer_cfg)
    return "[display]\n" + match.group(1)


class Flaky(Status):
    """A status whose get_status raises while `broken` is set, as an
    object can for a moment (a unit reconnecting)."""

    def __init__(self):
        super(Flaky, self).__init__(ok=True)
        self.broken = False

    def get_status(self, eventtime):
        if self.broken:
            raise RuntimeError("status not available")
        return dict(self.values)


# An entry of the walk's own, whose enable reads the Flaky object.
PROBE_MENU = """
[menu __main __probe]
type: command
enable: {printer.flaky.ok}
name: Probe entry
gcode: M117 probe
"""


class Bench:
    def __init__(self, oams=True, owner=True, probe=False):
        self.temp = tempfile.mkdtemp(prefix="menu-walk-")
        self.printer = printer = Printer(
            ["newest.gcode", "benchy.gcode", "calibration_cube.gcode"])
        text = display_section(OWNER_PRINTER_CFG) + "\n"
        if owner and OWNER_MENU_CFG:
            shutil.copy(OWNER_MENU_CFG, os.path.join(self.temp, "menu.cfg"))
            text += "[include menu.cfg]\n"
        self.oams = None
        if oams and OAMS_MENU_CFG:
            shutil.copy(OAMS_MENU_CFG, os.path.join(self.temp, "oams_menu.cfg"))
            text += "[include oams_menu.cfg]\n"
            self.oams = printer.objects["oams_manager"] = OpenAMS()
        if probe:
            self.flaky = printer.objects["flaky"] = Flaky()
            text += PROBE_MENU
        path = os.path.join(self.temp, "printer.cfg")
        with open(path, "w") as handle:
            handle.write(text)
        reader = configfile.ConfigFileReader()
        self.fileconfig = reader.build_fileconfig_with_includes(text, path)
        self.validate = configfile.ConfigValidate(printer)
        self.access = self.validate.start_access_tracking(
            reader.build_fileconfig("", "autosave"))
        config = configfile.ConfigWrapper(printer, self.fileconfig,
                                          self.access, "printer")
        printer.objects["gcode_macro"] = gcode_macro.PrinterGCodeMacro(config)
        printer.objects["display_status"] = display_status.load_config(config)
        for section in config.get_prefix_sections("gcode_macro "):
            printer.objects[section.get_name()] = gcode_macro.GCodeMacro(
                section)
        chips = display.LCD_chips
        saved = chips["uc1701"]
        chips["uc1701"] = HeadlessUC1701
        try:
            self.lcd = display.PrinterLCD(config.getsection("display"))
        finally:
            chips["uc1701"] = saved
        self.chip = self.lcd.lcd_chip
        self.menu = self.lcd.menu
        printer.send_event("klippy:ready")
        self.reactor = printer.reactor
        self.reactor.advance(0.6)
        self.actions = 0
        self.log = []

    def close(self):
        shutil.rmtree(self.temp, ignore_errors=True)

    def check_unused(self):
        self.validate.check_unused(self.fileconfig)

    # The knob -----------------------------------------------------------

    def screen(self, title=None):
        rows = self.chip.text_rows()
        if title is not None:
            self.log.append((title, rows))
            if SHOW:
                print("\n  " + title)
                for row in rows:
                    print("  |%s|" % row)
        return rows

    def settle(self):
        """Let the display timer redraw, as it does within 0.1 s of a key."""
        self.reactor.advance(0.15)

    def turn(self, detents, title=None):
        """Turn the knob: positive is physical clockwise, which on this
        wiring fires Klipper's encoder_ccw_callback and moves the cursor
        down; negative is counter-clockwise (encoder_cw_callback, up).
        Each detent 0.1 s apart, slower than menu_keys' fast rate."""
        klipper_cw, klipper_ccw = self.printer.objects["buttons"].encoder
        for _ in range(abs(detents)):
            self.reactor.advance(0.1)
            (klipper_ccw if detents > 0 else klipper_cw)(self.reactor.now)
            self.actions += 1
        self.settle()
        return self.screen(title)

    def click(self, title=None):
        press = self.printer.objects["buttons"].click
        press(self.reactor.now, 1)
        self.reactor.advance(0.05)
        press(self.reactor.now, 0)
        self.actions += 1
        self.settle()
        return self.screen(title)

    def long_press(self, title=None):
        press = self.printer.objects["buttons"].click
        press(self.reactor.now, 1)
        self.reactor.advance(1.0)           # menu_keys fires at 0.8 s
        press(self.reactor.now, 0)
        self.actions += 1
        self.settle()
        return self.screen(title)

    def cursor_row(self):
        rows = self.chip.text_rows()
        marks = [i for i, row in enumerate(rows) if row[:1] in (">", "*")]
        return entry_name(rows[marks[0]]) if marks else None

    def goto(self, name, title=None):
        """Turn until the cursor is on `name` (down, then up)."""
        for direction in (1, -1):
            for _ in range(40):
                if self.cursor_row() == name:
                    return self.screen(title)
                before = self.chip.text_rows()
                self.turn(direction)
                if self.chip.text_rows() == before:
                    break
        raise AssertionError("%r not reachable; screen %r"
                             % (name, self.chip.text_rows()))

    def all_rows(self):
        """Every row of the open list, read by turning to its end and back
        to the top, the way a person would."""
        seen = []
        for _ in range(60):
            for row in self.chip.text_rows():
                name = entry_name(row)
                if name and name not in seen:
                    seen.append(name)
            before = self.chip.text_rows()
            self.turn(1)
            if self.chip.text_rows() == before:
                break
        self.goto("..")
        return seen

    def menu_open(self):
        return self.menu.is_running()


def entry_name(row):
    """A menu row without its cursor column and its list mark ('>' in the
    last column)."""
    name = row[1:]
    if name.endswith(">"):
        name = name[:-1]
    return name.rstrip()


def row_names(rows):
    return [entry_name(row) for row in rows]


# ---------------------------------------------------------------------------
# The walks
# ---------------------------------------------------------------------------

class WalkTests(unittest.TestCase):
    def setUp(self):
        self.b = Bench()
        self.addCleanup(self.b.close)

    def open_main(self, title="Main"):
        b = self.b
        if b.menu_open():
            b.menu.exit(True)
            b.settle()
        return b.click(title)

    def test_the_config_passes_klippers_startup_check(self):
        self.b.check_unused()

    def test_the_status_screen_renders_under_the_template_sandbox(self):
        """gcode_macro's sandbox rejects str.format. With Jinja 3.1.6 the
        old status screen still rendered; with an older Jinja (2.x) it
        failed. display.cfg no longer uses str.format, so it renders on
        both."""
        b = self.b
        b.printer.state("printing")
        b.printer.objects["virtual_sdcard"].values["progress"] = 0.42
        b.printer.objects["extruder"].values.update(temperature=245.,
                                                    target=245.)
        b.printer.objects["fan"].values["speed"] = 0.5
        b.printer.objects["gcode_move"].values["speed_factor"] = 1.25
        logger = logging.getLogger()
        records = []
        handler = logging.Handler()
        handler.emit = records.append
        logger.addHandler(handler)
        try:
            b.reactor.advance(1.0)
        finally:
            logger.removeHandler(handler)
        self.assertEqual([r.getMessage() for r in records
                          if r.levelno >= logging.ERROR], [])
        rows = b.screen("Status screen, printing at 42%")
        self.assertIn("245", rows[0])
        self.assertIn(" 50%", rows[0])
        self.assertIn("125%", rows[1])
        self.assertIn("42%", rows[2])

    def test_a_physical_clockwise_detent_moves_the_cursor_down(self):
        """voron350's owner: clockwise moves down, counter-clockwise up.
        With its encoder_pins that is Klipper's ccw callback."""
        b = self.b
        self.assertEqual(
            tuple(p.strip() for p in b.printer.objects["buttons"].encoder_pins),
            ("^EXP2_5", "^EXP2_3"))
        self.open_main("Main, cursor on ..")
        b.turn(1, "One clockwise detent: down one row")
        self.assertEqual(b.chip.text_rows()[1][0], ">")
        b.turn(-1, "One counter-clockwise detent: back up")
        self.assertEqual(b.chip.text_rows()[0][0], ">")

    def test_a_print_starting_closes_each_confirm_left_open(self):
        """The menu left on a confirm (no menu_timeout on voron350), and a
        print started from Mainsail: the next redraw leaves the list whose
        gate closed, so "Yes, restart" is not one click away."""
        b = self.b
        b.printer.objects["configfile"].values["save_config_pending"] = True
        for path, yes in ((("Setup", "Restart", "Restart FW"), "Yes, restart"),
                          (("Setup", "Restart", "Restart host"), "Yes, restart"),
                          (("Setup", "Save config"), "Save & restart")):
            b.printer.state("standby")
            self.open_main()
            for entry in path:
                b.goto(entry)
                b.click()
            b.goto(yes, "%s open, cursor on %s" % (path[-1], yes))
            b.printer.state("printing")         # started from Mainsail
            b.reactor.advance(0.6)
            rows = b.screen("A print starts: %s is left" % path[-1])
            self.assertEqual(rows[0].rstrip(), ">..")
            self.assertEqual(row_names(rows)[:3],
                             ["..", "Pause print", "Cancel print"])
            self.assertNotIn(yes, row_names(rows))
        self.assertEqual(b.printer.gcode.scripts, [])

    def test_print_the_newest_file_in_eight_actions_with_no_going_back(self):
        b = self.b
        b.screen("Status screen, idle")
        b.actions = 0
        self.open_main("Main")
        b.goto("SD Card", "Turn clockwise to SD Card")
        rows = b.click("SD Card: newest file first")
        self.assertEqual(row_names(rows)[:3],
                         ["..", "newest.gcode", "benchy.gcode"])
        b.turn(1)
        rows = b.click("Click the file: its confirm")
        self.assertEqual(rows[0].rstrip(), ">..")
        self.assertEqual(row_names(rows)[:2], ["..", "Print it"])
        b.turn(1, "One clockwise detent to Print it")
        rows = b.click("Click Print it: the status screen")
        self.assertIn('SDCARD_PRINT_FILE FILENAME="newest.gcode"',
                      b.printer.gcode.scripts)
        self.assertFalse(b.menu_open())
        self.assertEqual(b.printer.objects["print_stats"].values["state"],
                         "printing")
        # Open, turn to SD Card (two with OpenAMS above it), click, turn,
        # click, turn, click.
        self.assertEqual(b.actions, 8 if b.oams is not None else 7)
        if SHOW:
            print("  actions: %d" % b.actions)

    def test_print_it_refuses_at_the_click_when_the_print_was_paused(self):
        b = self.b
        self.open_main()
        b.goto("SD Card")
        b.click()
        b.turn(1)
        b.click()
        b.turn(1, "Print it, a print idle")
        # Mainsail starts and pauses a print between two redraws; the
        # click comes before the next one shows it.
        b.printer.state("paused")
        b.printer.objects["virtual_sdcard"].values["file_path"] = "/gcodes/x"
        rows = b.click("Click Print it in that moment: refused, back")
        self.assertNotIn("SDCARD_PRINT_FILE",
                         " ".join(b.printer.gcode.scripts))
        self.assertIn("newest.gcode was not started: a print is paused",
                      b.printer.gcode.messages)
        self.assertTrue(b.menu_open())
        b.reactor.advance(0.6)
        rows = b.screen("Next redraw: SD Card's own gate closed, so the menu"
                        " left it: Main, cursor on ..")
        self.assertEqual(row_names(rows)[:3],
                         ["..", "Resume print", "Cancel print"])
        self.assertEqual(rows[0].rstrip(), ">..")

    def test_a_paused_print_main_offers_resume_and_hides_what_ends_it(self):
        b = self.b
        b.printer.state("paused")
        b.printer.objects["virtual_sdcard"].values["file_path"] = "/gcodes/x"
        rows = self.open_main("Main, print paused")
        names = b.all_rows()
        self.assertEqual(names[:3], ["..", "Resume print", "Cancel print"])
        for hidden in ("Pause print", "SD Card", "Setup", "Filament"):
            self.assertNotIn(hidden, names)
        b.goto("Control")
        control = (b.click("Control, print paused"), b.all_rows())[1]
        for hidden in ("Home All", "Home Z", "Home X/Y", "Quad Gantry Lvl",
                       "Bed Mesh", "Steppers off", "Move 10mm", "Move 1mm",
                       "Move 0.1mm"):
            self.assertNotIn(hidden, control)
        b.click()                                   # ".." back to Main
        b.goto("Temperature")
        temp = (b.click("Temperature, print paused"), b.all_rows())[1]
        self.assertNotIn("Cooldown", temp)
        start = b.menu.lookup_menuitem("__main __sdcard __start")
        b.menu.update_context(b.reactor.now)
        self.assertFalse(start.is_enabled())

    def test_printing_main_leads_with_pause_and_cancel(self):
        b = self.b
        b.printer.state("printing")
        rows = self.open_main("Main, printing")
        self.assertEqual(row_names(rows)[:3],
                         ["..", "Pause print", "Cancel print"])

    def test_cancel_lands_on_its_confirm_with_the_cursor_on_back(self):
        b = self.b
        b.printer.state("printing")
        self.open_main()
        b.goto("Cancel print")
        rows = b.click("Cancel print: its confirm")
        self.assertEqual(rows[0].rstrip(), ">..")
        self.assertEqual(row_names(rows)[:2], ["..", "Yes, cancel it"])
        b.turn(1)
        b.click("Yes, cancel it: the status screen")
        self.assertIn("CANCEL_PRINT", b.printer.gcode.scripts)
        self.assertFalse(b.menu_open())

    def test_restart_and_save_config_land_on_their_confirms(self):
        b = self.b
        b.printer.objects["configfile"].values["save_config_pending"] = True
        self.open_main()
        b.goto("Setup")
        b.click("Setup")
        rows = (b.goto("Save config"), b.click("Save config: its confirm"))[1]
        self.assertEqual(row_names(rows)[:2], ["..", "Save & restart"])
        self.assertEqual(rows[0].rstrip(), ">..")
        b.click()                                   # ".."
        b.goto("Restart")
        b.click()
        for entry in ("Restart host", "Restart FW"):
            b.goto(entry)
            rows = b.click("%s: its confirm" % entry)
            self.assertEqual(row_names(rows)[:2], ["..", "Yes, restart"])
            self.assertEqual(rows[0].rstrip(), ">..")
            b.click()                               # ".."
        self.assertEqual(b.printer.gcode.scripts, [])

    def test_an_entry_vanishing_under_the_cursor_moves_it_to_back(self):
        b = self.b
        b.printer.state("printing")
        self.open_main()
        b.goto("Cancel print", "Printing, cursor on Cancel print")
        b.printer.state("complete")                 # the print ends
        b.reactor.advance(0.6)
        rows = b.screen("The print ends: rows follow, cursor on ..")
        self.assertEqual(rows[0].rstrip(), ">..")
        self.assertNotIn("Cancel print", row_names(rows))
        b.click()
        self.assertFalse(b.menu_open())             # ".." at Main leaves

    def test_a_raising_enable_keeps_the_rows_and_klippy_runs_on(self):
        self.b = b = Bench(probe=True)
        self.addCleanup(b.close)
        self.open_main()
        before = b.goto("SD Card", "Main with a probe entry, cursor on SD Card")
        self.assertIn("Probe entry", b.all_rows())
        b.goto("SD Card")
        b.flaky.broken = True
        with self.assertLogs(level="ERROR") as logs:
            b.reactor.advance(3.0)                  # six redraws
        failures = [r for r in logs.records
                    if "an enable template failed" in r.getMessage()]
        self.assertEqual(len(failures), 1)
        rows = b.screen("Its status raises for 3 s: same rows, one log line")
        self.assertEqual(rows, before)
        self.assertLess(b.lcd.screen_update_timer.waketime, Reactor.NEVER)
        b.flaky.broken = False
        b.flaky.values["ok"] = False
        b.reactor.advance(0.6)
        self.assertNotIn("Probe entry", b.all_rows())
        b.turn(1, "Status back, the entry off: the knob works")
        self.assertEqual(b.cursor_row(), "OpenAMS" if b.oams is not None
                         else "SD Card")

    def test_a_missing_openams_status_key_keeps_the_rows(self):
        b = self.b
        if b.oams is None:
            self.skipTest("set OAMS_MENU_CFG for the OpenAMS enables")
        self.open_main()
        before = b.goto("SD Card", "Main, cursor on SD Card")
        b.oams.broken = True
        with self.assertLogs(level="ERROR") as logs:
            b.reactor.advance(3.0)                  # six redraws
        failures = [r for r in logs.records
                    if "an enable template failed" in r.getMessage()]
        self.assertEqual(len(failures), 1)
        rows = b.screen("lanes_by_fps missing for 3 s: same rows")
        self.assertEqual(rows, before)
        # The display timer is still scheduled, and the knob still works.
        self.assertLess(b.lcd.screen_update_timer.waketime, Reactor.NEVER)
        b.oams.broken = False
        b.turn(1, "Status back: the knob works")
        self.assertEqual(b.cursor_row(), "Control")

    def test_long_press_goes_back_and_back_still_works(self):
        b = self.b
        self.open_main()
        b.goto("Control")
        b.click("Control")
        b.goto("Steppers off", "Cursor on Steppers off")
        rows = b.long_press("Long press: back in Main, nothing ran")
        self.assertEqual(b.cursor_row(), "Control")
        self.assertEqual(b.printer.gcode.scripts, [])
        b.click()
        b.goto("..", "The '..' row")
        b.click("Click '..': back in Main")
        self.assertEqual(b.cursor_row(), "Control")
        b.long_press("Long press in Main: the status screen")
        self.assertFalse(b.menu_open())


class OpenAMSWalkTests(unittest.TestCase):
    def setUp(self):
        if not OAMS_MENU_CFG:
            self.skipTest("set OAMS_MENU_CFG to OpenAMS's oams_menu.cfg")
        self.b = Bench()
        self.addCleanup(self.b.close)

    def test_openams_sits_after_the_print_rows(self):
        b = self.b
        rows = b.click("Main, idle")
        self.assertEqual(row_names(rows)[:3], ["..", "OpenAMS", "SD Card"])
        b.menu.exit(True)
        b.printer.state("paused")
        rows = b.click("Main, print paused")
        self.assertEqual(row_names(rows)[:4],
                         ["..", "Resume print", "Cancel print", "OpenAMS"])

    def test_stop_openams_leads_main_while_a_sequence_runs(self):
        b = self.b
        b.oams.running = True
        rows = b.click("Main, a toolchange running")
        self.assertEqual(row_names(rows)[1], "Stop OpenAMS")

    def test_dryer_presets_start_in_one_go(self):
        b = self.b
        b.click()
        b.goto("OpenAMS")
        b.click("OpenAMS")
        b.goto("Dryer")
        b.click("Dryer")
        b.goto("Start dryer")
        b.click("Click Start dryer: the picker opens")
        labels = []
        for _ in range(4):
            rows = b.turn(1)
            labels.append(b.cursor_row())
        self.assertEqual(labels, ["PLA 55C 6h?", "PETG 65C 6h?",
                                  "ABS 65C 6h?", "TPU 55C 6h?"])
        b.turn(-2, "Two counter-clockwise detents back")
        b.click("Click: started")
        self.assertIn("OAMS_DRYER_START OAMS=2 TARGET=65 DURATION=21600",
                      b.printer.gcode.scripts)

    def test_a_setting_is_one_click(self):
        b = self.b
        b.click()
        b.goto("OpenAMS")
        b.click()
        b.goto("Settings")
        b.click("Settings")
        b.goto("Autoreload: on")
        b.click("One click on Autoreload")
        self.assertIn("OAMSM_SET_OPTION NAME=auto_reload_on_runout VALUE=0",
                      b.printer.gcode.scripts)
        self.assertEqual(b.cursor_row(), "Autoreload: off")


if __name__ == "__main__":
    if "--show" in sys.argv:
        sys.argv.remove("--show")
        SHOW = True
    unittest.main()
