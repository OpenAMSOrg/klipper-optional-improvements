# Support for tracking canbus node ids
#
# Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

NODEID_FIRST = 4

class PrinterCANBus:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.ids = {}
    def add_uuid(self, config, canbus_uuid, canbus_iface):
        # The administration protocol carries a UUID as exactly six bytes;
        # configuration text is only a human-readable spelling of that number.
        # Using the original string as the allocator key would treat case and
        # leading-zero variants as different devices, even though connect_canbus
        # later encodes them into identical bytes.  That can assign two node ids
        # to one MCU and make each connection attempt displace the other.  Parse
        # and range-check at registration time so duplicate detection operates
        # on the identity that is actually placed on the wire.
        try:
            uuid = int(canbus_uuid, 16)
        except ValueError:
            uuid = -1
        if uuid < 0 or uuid > 0xffffffffffff:
            raise config.error("Invalid canbus_uuid")
        if uuid in self.ids:
            raise config.error("Duplicate canbus_uuid")
        new_id = len(self.ids) + NODEID_FIRST
        self.ids[uuid] = new_id
        return new_id
    def get_nodeid(self, canbus_uuid):
        uuid = int(canbus_uuid, 16)
        if uuid not in self.ids:
            raise self.printer.config_error("Unknown canbus_uuid %s"
                                            % (canbus_uuid,))
        return self.ids[uuid]

def load_config(config):
    return PrinterCANBus(config)
