# Heater/sensor verification code
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

HINT_THERMAL = """
See the 'verify_heater' section in docs/Config_Reference.md
for the parameters that control this check.
"""

class HeaterCheck:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_shutdown)
        self.heater_name = config.get_name().split()[1]
        self.heater = None
        self.enabled = config.getboolean('enable', True)
        self.hysteresis = config.getfloat('hysteresis', 5., minval=0.)
        self.max_error = config.getfloat('max_error', 120., minval=0.)
        self.heating_gain = config.getfloat('heating_gain', 2., above=0.)
        default_gain_time = 20.
        if self.heater_name == 'heater_bed':
            default_gain_time = 60.
        self.check_gain_time = config.getfloat(
            'check_gain_time', default_gain_time, minval=1.)
        self.approaching_target = self.starting_approach = False
        self.last_target = self.goal_temp = self.error = 0.
        self.goal_systime = self.printer.get_reactor().NEVER
        self.check_timer = None
        # Register command to enable/disable verification at runtime
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            "SET_HEATER_VERIFICATION", "HEATER", self.heater_name,
            self.cmd_SET_HEATER_VERIFICATION,
            desc=self.cmd_SET_HEATER_VERIFICATION_help)
    def handle_connect(self):
        if self.printer.get_start_args().get('debugoutput') is not None:
            # Disable verify_heater if outputting to a debug file
            return
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        reactor = self.printer.get_reactor()
        self.check_timer = reactor.register_timer(self.check_event)
        self.update_check_state()
    def update_check_state(self):
        # Reset checks and start/stop the timer based on self.enabled
        if self.check_timer is None:
            return
        reactor = self.printer.get_reactor()
        self.approaching_target = self.starting_approach = False
        self.last_target = self.goal_temp = self.error = 0.
        self.goal_systime = reactor.NEVER
        if self.enabled:
            logging.info("Starting heater checks for %s", self.heater_name)
            reactor.update_timer(self.check_timer, reactor.NOW)
        else:
            logging.info("Heater checks disabled for %s", self.heater_name)
            reactor.update_timer(self.check_timer, reactor.NEVER)
    def handle_shutdown(self):
        if self.check_timer is not None:
            reactor = self.printer.get_reactor()
            reactor.update_timer(self.check_timer, reactor.NEVER)
    def check_event(self, eventtime):
        temp, target = self.heater.get_temp(eventtime)
        if temp >= target - self.hysteresis or target <= 0.:
            # Temperature near target - reset checks
            if self.approaching_target and target:
                logging.info("Heater %s within range of %.3f",
                             self.heater_name, target)
            self.approaching_target = self.starting_approach = False
            if temp <= target + self.hysteresis:
                self.error = 0.
            self.last_target = target
            return eventtime + 1.
        self.error += (target - self.hysteresis) - temp
        if not self.approaching_target:
            if target != self.last_target:
                # Target changed - reset checks
                logging.info("Heater %s approaching new target of %.3f",
                             self.heater_name, target)
                self.approaching_target = self.starting_approach = True
                self.goal_temp = temp + self.heating_gain
                self.goal_systime = eventtime + self.check_gain_time
            elif self.error >= self.max_error:
                # Failure due to inability to maintain target temperature
                return self.heater_fault()
        elif temp >= self.goal_temp:
            # Temperature approaching target - reset checks
            self.starting_approach = False
            self.error = 0.
            self.goal_temp = temp + self.heating_gain
            self.goal_systime = eventtime + self.check_gain_time
        elif eventtime >= self.goal_systime:
            # Temperature is no longer approaching target
            self.approaching_target = False
            logging.info("Heater %s no longer approaching target %.3f",
                         self.heater_name, target)
        elif self.starting_approach:
            self.goal_temp = min(self.goal_temp, temp + self.heating_gain)
        self.last_target = target
        return eventtime + 1.
    def heater_fault(self):
        msg = "Heater %s not heating at expected rate" % (self.heater_name,)
        logging.error(msg)
        self.printer.invoke_shutdown(msg + HINT_THERMAL)
        return self.printer.get_reactor().NEVER
    cmd_SET_HEATER_VERIFICATION_help = \
        "Enable/disable heater verification for a heater"
    def cmd_SET_HEATER_VERIFICATION(self, gcmd):
        self.enabled = gcmd.get_int('ENABLE', 1) != 0
        self.update_check_state()
        gcmd.respond_info("Heater verification for '%s' %s"
                          % (self.heater_name,
                             "enabled" if self.enabled else "disabled"))

def load_config_prefix(config):
    return HeaterCheck(config)
