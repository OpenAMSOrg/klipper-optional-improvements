# Host-side tests of the LCD menu engine and the default menu
#
# Run with the Python Klipper runs on (jinja2 is needed):
#   ~/klippy-env/bin/python -m unittest discover -s test/menu -v
#
# The real klippy/extras/display/menu.py and menu.cfg are loaded with fake
# printer objects: no MCU, no display hardware.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import contextlib
import logging
import os
import sys
import unittest

KLIPPY = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "..", "..", "klippy")
sys.path.insert(0, os.path.abspath(KLIPPY))

import configfile                                   # noqa: E402
import gcode as gcode_module                        # noqa: E402
from extras import gcode_macro                      # noqa: E402
from extras.display import menu                     # noqa: E402

TEST_MENU = """
[display]
lcd_type: st7920
%(display_options)s

[menu __main __test]
type: list
name: Test

[menu __main __test __a]
type: command
enable: {printer.flags.a}
name: A
gcode: RUN_A

[menu __main __test __b]
type: command
enable: {printer.flags.b}
name: B
gcode: RUN_B

[menu __main __test __c]
type: command
enable: {printer.flags.c}
name: C
gcode: RUN_C

[menu __main __test __flaky]
type: command
enable: {printer.flags.boom.value.deeper}
name: Flaky
gcode: RUN_FLAKY

[menu __main __test __input]
type: input
name: In {'%%d' %% menu.input}
input: 0
input_min: 0
input_max: 5
input_step: 1
gcode: TEST_{menu.event|upper}

[menu __main __files]
type: vsdlist
name: Files
file_click: confirm

[menu __main __picks]
type: vsdlist
name: Picks
"""


class Reactor:
    NOW = 0.
    NEVER = 1.e30

    def __init__(self):
        self.now = 100.

    def monotonic(self):
        return self.now

    def register_timer(self, callback, when=NOW):
        return [callback, when]

    def update_timer(self, timer, when):
        timer[1] = when

    def mutex(self):
        return contextlib.nullcontext()

    def assert_no_pause(self):
        return contextlib.nullcontext()


class Status:
    def __init__(self, **values):
        self.values = values

    def get_status(self, eventtime):
        return dict(self.values)


class SDCard(Status):
    def __init__(self, files):
        super(SDCard, self).__init__(file_path=None, is_active=False,
                                     progress=0.)
        self.files = files

    def is_active(self):
        return self.values["is_active"]

    def get_file_list(self, check_subdirs=False, sort_by_date=False):
        return [(name, 100) for name in self.files]


class GCode:
    error = gcode_module.CommandError

    def __init__(self):
        self.messages = []

    def respond_info(self, msg, log=True):
        self.messages.append(msg)

    def register_command(self, *args, **kwargs):
        pass


class ConfigFile(Status):
    def __init__(self, printer):
        super(ConfigFile, self).__init__(
            save_config_pending=False, settings={},
            config={"extruder": {"max_temp": "300"},
                    "heater_bed": {"max_temp": "120"}})
        self.printer = printer

    def read_config(self, filename):
        reader = configfile.ConfigFileReader()
        fileconfig = reader.build_fileconfig(
            reader.read_config_file(filename), filename)
        return configfile.ConfigWrapper(self.printer, fileconfig, {},
                                        "printer")


class Printer:
    config_error = configfile.error
    command_error = gcode_module.CommandError

    def __init__(self):
        self.reactor = Reactor()
        self.events = {}
        self.objects = {}
        self.objects["gcode"] = self.gcode = GCode()
        self.objects["configfile"] = ConfigFile(self)
        self.objects["buttons"] = object()
        self.objects["idle_timeout"] = Status(state="Idle",
                                              printing_time=0.)
        self.objects["print_stats"] = Status(state="standby")
        self.objects["pause_resume"] = Status(is_paused=False)
        self.objects["virtual_sdcard"] = SDCard(["a.gcode", "b.gcode"])
        self.objects["toolhead"] = Status(
            homed_axes="xyz", position=[0., 0., 10., 0.],
            axis_minimum=[0., 0., 0., 0.], axis_maximum=[300., 300., 300., 0.])
        self.objects["flags"] = Status(a=True, b=True, c=True,
                                       boom={"value": {"deeper": True}})
        self.objects["heaters"] = Status(available_heaters=["extruder"])
        self.objects["extruder"] = Status(temperature=20., target=0.)
        self.objects["fan"] = Status(speed=0.)

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

    def set(self, name, **values):
        self.objects[name].values.update(values)


class Display:
    def __init__(self, cols=16, rows=4):
        self.cols, self.rows = cols, rows

    def get_dimensions(self):
        return self.cols, self.rows

    def draw_text(self, row, col, text, eventtime):
        return col + len(text)

    def request_redraw(self):
        pass


class Bench:
    def __init__(self, display_options=""):
        self.printer = printer = Printer()
        fileconfig = configfile.ConfigFileReader().build_fileconfig(
            TEST_MENU % {"display_options": display_options}, "printer.cfg")
        config = configfile.ConfigWrapper(printer, fileconfig, {}, "printer")
        printer.objects["gcode_macro"] = gcode_macro.PrinterGCodeMacro(config)
        self.menu = menu.MenuManager(config.getsection("display"),
                                     Display())
        self.scripts = []
        self.menu.queue_gcode = self._queue

    def _queue(self, script):
        if script:
            self.scripts.append(script)

    def key(self, name):
        self.menu.key_event(name, self.printer.reactor.monotonic())

    def redraw(self):
        self.printer.reactor.now += 0.5
        self.menu.screen_update_event(self.printer.reactor.monotonic())

    def container(self):
        return self.menu.stack_peek()

    def rows(self):
        self.menu.update_context(self.printer.reactor.monotonic())
        return [item.render_name() for item in self.container()]

    def selected(self):
        self.menu.update_context(self.printer.reactor.monotonic())
        item = self.container().selected_item()
        return item.render_name() if item is not None else None

    def open(self, *path):
        if not self.menu.is_running():
            self.key("click")
        for name in path:
            self.select(name)
            self.key("click")
        return self.rows()

    def select(self, name):
        rows = self.rows()
        target = rows.index(name)
        while self.container().selected < target:
            self.key("down")
        while self.container().selected > target:
            self.key("up")


# ---------------------------------------------------------------------------
# The default menu (menu.cfg): what shows when, and the confirms
# ---------------------------------------------------------------------------

class DefaultMenuTests(unittest.TestCase):
    def main_rows(self, bench):
        rows = bench.open()
        return [row for row in rows if row not in ("Test", "Files", "Picks")]

    def test_idle_main_has_no_print_rows(self):
        b = Bench()
        self.assertEqual(self.main_rows(b), [
            "..", "OctoPrint", "SD Card", "Control", "Temperature",
            "Filament", "Setup"])

    def test_printing_main_leads_with_pause_and_cancel(self):
        b = Bench()
        b.printer.set("print_stats", state="printing")
        b.printer.set("idle_timeout", state="Printing")
        b.printer.set("virtual_sdcard", is_active=True, file_path="/x/a.gcode")
        self.assertEqual(self.main_rows(b), [
            "..", "Pause print", "Cancel print", "Tune", "OctoPrint",
            "Control", "Temperature"])
        # Motion and motors off are gone; the fan stays.
        self.assertEqual(b.open("Control"),
                         ["..", "Fan: OFF", "Fan speed:   0%"])

    def test_a_pause_offers_resume_and_hides_what_ends_the_print(self):
        """idle_timeout is not "Printing" in a pause; print_stats is the
        gate now, so homing, motors off, restarts, filament moves, cooldown
        and the SD list are not offered on a paused print."""
        b = Bench()
        b.printer.set("print_stats", state="paused")
        b.printer.set("idle_timeout", state="Ready")
        b.printer.set("virtual_sdcard", is_active=False,
                      file_path="/x/a.gcode")
        self.assertEqual(self.main_rows(b), [
            "..", "Resume print", "Cancel print", "OctoPrint", "Control",
            "Temperature"])
        control = b.open("Control")
        for hidden in ("Home All", "Home Z", "Home X/Y", "Steppers off",
                       "Move 10mm", "Move 1mm", "Move 0.1mm"):
            self.assertNotIn(hidden, control)
        b.key("click")          # '..'
        self.assertNotIn("Cooldown", b.open("Temperature"))

    def test_start_printing_is_never_offered_in_a_pause(self):
        b = Bench()
        start = b.menu.lookup_menuitem("__main __sdcard __start")
        b.menu.update_context(0.)
        b.printer.set("virtual_sdcard", file_path="/x/a.gcode")
        b.menu.update_context(1.)
        self.assertTrue(start.is_enabled())
        b.printer.set("print_stats", state="paused")
        b.menu.update_context(2.)
        self.assertFalse(start.is_enabled())

    def test_cancel_needs_a_confirm_and_leaves_the_menu(self):
        b = Bench()
        b.printer.set("print_stats", state="printing")
        b.open()
        self.assertEqual(b.open("Cancel print"), ["..", "Yes, cancel it"])
        self.assertEqual(b.selected(), "..")
        b.select("Yes, cancel it")
        b.key("click")
        self.assertEqual([s.split() for s in b.scripts], [["CANCEL_PRINT"]])
        self.assertFalse(b.menu.is_running())

    def test_pause_and_resume_act_and_leave_the_menu(self):
        b = Bench()
        b.printer.set("print_stats", state="printing")
        b.open("Pause print")
        self.assertEqual([s.split() for s in b.scripts], [["PAUSE"]])
        self.assertFalse(b.menu.is_running())
        b.printer.set("print_stats", state="paused")
        b.open("Resume print")
        self.assertEqual(b.scripts[-1].split(), ["RESUME"])

    def test_save_config_shows_when_pending_and_confirms(self):
        b = Bench()
        self.assertNotIn("Save config", b.open("Setup"))
        b.printer.set("configfile", save_config_pending=True)
        b.key("click")
        self.assertEqual(b.open("Setup", "Save config"),
                         ["..", "Save & restart"])
        b.select("Save & restart")
        b.key("click")
        self.assertEqual(b.scripts, ["SAVE_CONFIG"])

    def test_restarts_need_a_confirm(self):
        b = Bench()
        self.assertEqual(b.open("Setup", "Restart", "Restart FW"),
                         ["..", "Yes, restart"])
        b.select("Yes, restart")
        b.key("click")
        self.assertEqual(b.scripts, ["FIRMWARE_RESTART"])


# ---------------------------------------------------------------------------
# vsdlist: a file click confirms and prints; never on a print in progress
# ---------------------------------------------------------------------------

class FileClickTests(unittest.TestCase):
    def test_a_file_opens_print_it_which_starts_and_leaves_the_menu(self):
        b = Bench()
        self.assertEqual(b.open("SD Card"), ["..", "a.gcode", "b.gcode"])
        self.assertEqual(b.open("b.gcode"), ["..", "Print it"])
        self.assertEqual(b.selected(), "..")
        b.select("Print it")
        b.key("click")
        self.assertEqual(b.scripts, ['SDCARD_PRINT_FILE FILENAME="b.gcode"'])
        self.assertFalse(b.menu.is_running())

    def test_print_it_refuses_at_the_click_when_a_print_was_paused_meanwhile(self):
        b = Bench()
        b.open("Files", "a.gcode")
        b.select("Print it")
        # A print started and paused from elsewhere while the menu was open;
        # no redraw has happened yet, so the row is still on screen.
        b.printer.set("print_stats", state="paused")
        b.key("click")
        self.assertEqual(b.scripts, [])
        self.assertEqual(b.printer.gcode.messages,
                         ["a.gcode was not started: a print is paused"])
        self.assertTrue(b.menu.is_running())
        self.assertEqual(b.rows()[0], "..")      # back in the file list
        # The next redraw hides the file rows and puts the cursor on '..'.
        b.redraw()
        self.assertEqual(b.rows(), [".."])
        self.assertEqual(b.container().selected, 0)

    def test_print_it_refuses_while_the_sd_worker_runs(self):
        b = Bench()
        b.open("Files", "a.gcode")
        b.select("Print it")
        b.printer.set("virtual_sdcard", is_active=True)
        b.key("click")
        self.assertEqual(b.scripts, [])
        self.assertIn("a print is running", b.printer.gcode.messages[0])

    def test_no_file_rows_while_printing_or_paused(self):
        for state in ("printing", "paused"):
            b = Bench()
            b.printer.set("print_stats", state=state)
            self.assertEqual(b.open("Files"), [".."], state)
            b.key("click")
            self.assertEqual(b.open("Picks"), [".."], state)

    def test_select_mode_rows_still_select_when_idle(self):
        b = Bench()
        b.open("Picks", "a.gcode")
        self.assertEqual(b.scripts, ["M23 /a.gcode"])

    def test_a_name_with_a_double_quote_is_left_out_of_confirm_mode(self):
        b = Bench()
        b.printer.objects["virtual_sdcard"].files = ['a"b.gcode', "c.gcode"]
        self.assertEqual(b.open("Files"), ["..", "c.gcode"])


# ---------------------------------------------------------------------------
# Live rows on redraw
# ---------------------------------------------------------------------------

class LiveRowsTests(unittest.TestCase):
    def test_an_entry_that_becomes_available_appears_without_leaving(self):
        b = Bench()
        b.printer.set("flags", c=False)
        self.assertEqual(b.open("Test")[:4], ["..", "A", "B", "Flaky"])
        b.printer.set("flags", c=True)
        b.redraw()
        self.assertEqual(b.rows()[:5], ["..", "A", "B", "C", "Flaky"])

    def test_the_cursor_stays_on_its_entry_when_rows_above_it_go(self):
        b = Bench()
        b.open("Test")
        b.select("B")
        b.printer.set("flags", a=False)
        b.redraw()
        self.assertEqual(b.selected(), "B")
        self.assertEqual(b.container().selected, 1)

    def test_when_the_selected_entry_goes_the_cursor_goes_to_back(self):
        """Never to whatever slid into its row: a click already under way
        then only goes back."""
        b = Bench()
        b.open("Test")
        b.select("B")
        b.printer.set("flags", b=False)
        b.redraw()
        self.assertEqual(b.container().selected, 0)
        self.assertEqual(b.selected(), "..")
        b.key("click")
        self.assertEqual(b.scripts, [])
        self.assertEqual(b.selected(), "Test")    # went back

    def test_a_failing_enable_keeps_the_rows_and_logs_once(self):
        b = Bench()
        self.assertEqual(b.open("Test")[:5], ["..", "A", "B", "C", "Flaky"])
        b.select("C")
        b.printer.set("flags", boom=None, a=False)
        with self.assertLogs(level="ERROR") as logs:
            for _ in range(5):
                b.redraw()          # must not raise out of the timer
        failures = [r for r in logs.records
                    if "an enable template failed" in r.getMessage()]
        self.assertEqual(len(failures), 1, [r.getMessage() for r in logs.records])
        self.assertFalse([r for r in logs.records
                          if "Error evaluating" in r.getMessage()])
        # The previous rows and cursor stay.
        self.assertEqual(b.rows()[:5], ["..", "A", "B", "C", "Flaky"])
        self.assertEqual(b.selected(), "C")
        # Fixed: the rows follow again, and a new failure is logged again.
        b.printer.set("flags", boom={"value": {"deeper": False}})
        b.redraw()
        self.assertEqual(b.rows()[:3], ["..", "B", "C"])
        b.printer.set("flags", boom=None)
        with self.assertLogs(level="ERROR") as logs:
            b.redraw()
            b.redraw()
        self.assertEqual(len([r for r in logs.records
                              if "an enable template failed"
                              in r.getMessage()]), 1)

    def test_an_open_input_is_not_refreshed_under_the_user(self):
        b = Bench()
        b.open("Test")
        b.select("In 0")
        b.key("click")                      # open the input
        b.printer.set("flags", a=False)
        b.redraw()
        self.assertIn("A", b.rows())
        self.assertTrue(b.container().selected_item().is_editing())

    def test_live_updates_can_be_turned_off(self):
        b = Bench("menu_live_update: False")
        b.printer.set("flags", c=False)
        b.open("Test")
        b.printer.set("flags", c=True)
        b.redraw()
        self.assertNotIn("C", b.rows())

    def test_a_list_with_no_rows_left_does_not_raise(self):
        b = Bench()
        b.open("Test")
        container = b.container()
        container._allitems = []
        container.update_items()
        self.assertEqual(len(container), 0)


# ---------------------------------------------------------------------------
# Long press = back
# ---------------------------------------------------------------------------

class LongPressTests(unittest.TestCase):
    def test_a_long_press_goes_back_and_never_runs_the_entry(self):
        b = Bench()
        b.open("Test")
        b.select("A")
        b.key("long_click")
        self.assertEqual(b.scripts, [])
        self.assertEqual(b.selected(), "Test")    # back in Main, on Test
        self.assertEqual(b.menu.stack_size(), 1)

    def test_a_long_press_on_a_confirm_backs_out(self):
        b = Bench()
        b.printer.set("print_stats", state="printing")
        b.open("Cancel print")
        b.select("Yes, cancel it")
        b.key("long_click")
        self.assertEqual(b.scripts, [])
        self.assertEqual(b.selected(), "Cancel print")

    def test_a_long_press_at_the_root_leaves_the_menu(self):
        b = Bench()
        b.open()
        b.key("long_click")
        self.assertFalse(b.menu.is_running())

    def test_an_open_input_keeps_its_long_press(self):
        b = Bench()
        b.open("Test")
        b.select("In 0")
        b.key("click")
        b.key("down")
        b.key("long_click")
        self.assertEqual(b.scripts, ["TEST_LONG_CLICK"])
        self.assertTrue(b.container().selected_item().is_editing())
        b.key("click")
        self.assertEqual(b.scripts, ["TEST_LONG_CLICK", "TEST_CLICK"])

    def test_the_option_off_keeps_the_old_long_press(self):
        b = Bench("menu_long_press_back: False")
        b.open("Test")
        b.select("A")
        b.key("long_click")
        self.assertEqual(b.scripts, ["RUN_A"])


if __name__ == "__main__":
    unittest.main()
