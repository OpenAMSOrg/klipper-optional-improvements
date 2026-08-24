# Report canbus connection status
#
# Copyright (C) 2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, zlib

class PrinterCANBusStats:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.mcu = None
        self.get_canbus_status_cmd = None
        self.get_canbus_diagnostics_cmd = None
        self.status = {'rx_error': None, 'tx_error': None, 'tx_retries': None,
                       'bus_state': None, 'rx_overflows': None,
                       'tx_discards': None, 'usb_invalid': None,
                       'bus_off': None}
        self.diagnostics_countdown = 0
        # Every canbus_stats object is registered during the same connection
        # phase.  If each first callback returns monotonic()+1, all later status
        # commands remain phase-locked and create a periodic request/response
        # burst across every CAN MCU.  Keep the initial query immediate so
        # status remains available promptly, then use a stable MCU-name hash to
        # add a one-time sub-second phase offset.  Subsequent callbacks retain
        # the existing one-second period, but their CAN traffic no longer
        # competes at the same bus-idle boundary on every cycle.
        name_crc = zlib.crc32(self.name.encode('utf-8')) & 0xffffffff
        self.query_phase = (name_crc & 0xffff) / 65536.
        self.printer.register_event_handler("klippy:connect",
                                            self.handle_connect)
        self.printer.register_event_handler("klippy:shutdown",
                                            self.handle_shutdown)
    def handle_shutdown(self):
        status = self.status.copy()
        if status['bus_state'] is not None:
            # Clear bus_state on shutdown to note that the values may be stale
            status['bus_state'] = 'unknown'
            self.status = status
    def handle_connect(self):
        # Lookup mcu
        mcu_name = self.name
        if mcu_name != 'mcu':
            mcu_name = 'mcu ' + mcu_name
        self.mcu = self.printer.lookup_object(mcu_name)
        # Lookup status query command
        if self.mcu.try_lookup_command("get_canbus_status") is None:
            configfile = self.printer.lookup_object('configfile')
            configfile.deprecate_mcu_code(self.mcu, 'get_canbus_status')
            return
        self.get_canbus_status_cmd = self.mcu.lookup_query_command(
            "get_canbus_status",
            "canbus_status rx_error=%u tx_error=%u tx_retries=%u"
            " canbus_bus_state=%u")
        # Do not add fields to the established canbus_status response.  Command
        # and response formats are part of Klipper's host/MCU compatibility
        # contract: changing that response would make an old host reject new
        # firmware during message-dictionary validation.  A separately
        # discoverable query permits every mixed-version combination instead.
        # A new host simply omits diagnostics when connected to old firmware,
        # while an old host never sends the additional command to new firmware.
        if self.mcu.try_lookup_command("get_canbus_diagnostics") is not None:
            self.get_canbus_diagnostics_cmd = self.mcu.lookup_query_command(
                "get_canbus_diagnostics",
                "canbus_diagnostics rx_overflows=%u tx_discards=%u"
                " usb_invalid=%u bus_off=%u")
        # Register usb_canbus_state message handling (for usb to canbus bridge)
        if self.mcu.check_valid_response("usb_canbus_state discard=%u"):
            self.mcu.register_serial_response(self.handle_usb_canbus_state,
                                              "usb_canbus_state discard=%u")
        # Register periodic query timer
        self.reactor.register_timer(self.query_event, self.reactor.NOW)
    def handle_usb_canbus_state(self, params):
        discard = params['discard']
        if discard:
            logging.warning("USB CANBUS bridge '%s' is discarding!"
                            % (self.name,))
        else:
            logging.warning("USB CANBUS bridge '%s' is no longer discarding."
                            % (self.name,))
    def query_event(self, eventtime):
        prev_rx = self.status['rx_error']
        prev_tx = self.status['tx_error']
        prev_retries = self.status['tx_retries']
        if prev_rx is None:
            prev_rx = prev_tx = prev_retries = 0
        params = self.get_canbus_status_cmd.send()
        rx = prev_rx + ((params['rx_error'] - prev_rx) & 0xffffffff)
        tx = prev_tx + ((params['tx_error'] - prev_tx) & 0xffffffff)
        retries = prev_retries + ((params['tx_retries'] - prev_retries)
                                  & 0xffffffff)
        state = params['canbus_bus_state']
        status = self.status.copy()
        status.update({'rx_error': rx, 'tx_error': tx,
                       'tx_retries': retries, 'bus_state': state})

        # Queue loss and bus-off episodes are exceptional evidence, not values
        # needed for one-second control or display decisions.  Querying them at
        # the legacy status cadence would permanently double management traffic
        # on every healthy CAN network to observe counters which normally stay
        # at zero.  Reuse this already phase-staggered callback every five
        # cycles instead.  The first query remains immediate so a pre-existing
        # fault is visible promptly, and 32-bit extension below retains events
        # across arbitrary counter wrap without requiring expensive 64-bit
        # arithmetic in an MCU interrupt path.
        if self.get_canbus_diagnostics_cmd is not None:
            self.diagnostics_countdown -= 1
            if self.diagnostics_countdown <= 0:
                diag = self.get_canbus_diagnostics_cmd.send()
                self.diagnostics_countdown = 5
                for key in ('rx_overflows', 'tx_discards', 'usb_invalid',
                            'bus_off'):
                    previous = status[key]
                    if previous is None:
                        previous = 0
                    status[key] = previous + ((diag[key] - previous)
                                               & 0xffffffff)
        self.status = status
        waketime = self.reactor.monotonic() + 1.
        if self.query_phase is not None:
            waketime += self.query_phase
            self.query_phase = None
        return waketime
    def stats(self, eventtime):
        status = self.status
        if status['rx_error'] is None:
            return (False, '')
        msg = ('canstat_%s: bus_state=%s rx_error=%d'
               ' tx_error=%d tx_retries=%d'
               % (self.name, status['bus_state'], status['rx_error'],
                  status['tx_error'], status['tx_retries']))
        if status['rx_overflows'] is not None:
            msg += (' rx_overflows=%d tx_discards=%d usb_invalid=%d'
                    ' bus_off=%d'
                    % (status['rx_overflows'], status['tx_discards'],
                       status['usb_invalid'], status['bus_off']))
        return (False, msg)
    def get_status(self, eventtime):
        return self.status

def load_config_prefix(config):
    return PrinterCANBusStats(config)
