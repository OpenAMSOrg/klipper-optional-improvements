# virtual_sdcard refuses to load a file while a print is paused
#
# Run with the Python Klipper runs on:
#   ~/klippy-env/bin/python -m unittest discover -s test/menu -v
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import unittest

from test_menu import Printer, configfile, gcode_macro, gcode_module
from extras import virtual_sdcard


class SDPausedGuardTests(unittest.TestCase):
    """virtual_sdcard refuses to load a file while a print is paused:
    loading resets the print state, which threw the paused print away. It
    protects every caller (the menu, macros, a host)."""

    class Stats(object):
        def __init__(self):
            self.state = "standby"
            self.resets = 0
            self.current = None

        def reset(self):
            self.resets += 1

        def set_current_file(self, name):
            self.current = name

        def get_status(self, eventtime):
            return {"state": self.state}

    class Executor(object):
        def submit(self, fn, *args, **kwargs):
            return fn(*args, **kwargs)

        def wrap_obj(self, obj):
            return obj

    class Aio(object):
        def allocate_executor(self, name=""):
            return SDPausedGuardTests.Executor()

    class GCmd(object):
        error = gcode_module.CommandError

        def __init__(self, params, raw=""):
            self.params, self.raw = params, raw

        def get(self, name, default=None):
            return self.params.get(name, default)

        def get_raw_command_parameters(self):
            return self.raw

        def respond_raw(self, msg):
            pass

    def setUp(self):
        import tempfile
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        with open(os.path.join(self.folder.name, "a.gcode"), "w") as f:
            f.write("G4 P1\n")
        printer = self.printer = Printer()
        self.stats = printer.objects["print_stats"] = self.Stats()
        printer.objects["aio_executor"] = self.Aio()
        fileconfig = configfile.ConfigFileReader().build_fileconfig(
            "[virtual_sdcard]\npath: %s\n" % self.folder.name, "printer.cfg")
        config = configfile.ConfigWrapper(printer, fileconfig, {}, "printer")
        printer.objects["gcode_macro"] = gcode_macro.PrinterGCodeMacro(config)
        self.sd = virtual_sdcard.load_config(
            config.getsection("virtual_sdcard"))

    def test_print_file_and_m23_are_refused_in_a_pause(self):
        self.stats.state = "paused"
        with self.assertRaisesRegex(gcode_module.CommandError,
                                    "a print is paused"):
            self.sd.cmd_SDCARD_PRINT_FILE(self.GCmd({"FILENAME": "a.gcode"}))
        with self.assertRaisesRegex(gcode_module.CommandError,
                                    "a print is paused"):
            self.sd.cmd_M23(self.GCmd({}, raw="/a.gcode"))
        self.assertEqual(self.stats.resets, 0)       # nothing was reset
        self.assertIsNone(self.sd.current_file)

    def test_a_file_still_loads_and_starts_with_no_print(self):
        self.sd.cmd_SDCARD_PRINT_FILE(self.GCmd({"FILENAME": "a.gcode"}))
        self.assertEqual(self.stats.current, "a.gcode")
        self.assertTrue(self.sd.is_active())


if __name__ == "__main__":
    unittest.main()
