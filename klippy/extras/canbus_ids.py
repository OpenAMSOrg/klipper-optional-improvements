# Support for tracking canbus node ids
#
# Copyright (C) 2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

NODEID_FIRST = 4
NODEID_LAST = 0xff

class PrinterCANBus:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.ids = {}
        self.next_nodeids = {}
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
        # A node id becomes an arbitration id only on its selected CAN
        # interface; it has no process-wide meaning.  Two isolated buses may
        # legitimately contain the same factory UUID, and both should begin
        # allocation at NODEID_FIRST instead of consuming one global sequence.
        # Include the interface in the identity key and keep its next id in a
        # matching namespace.  This still rejects a duplicate on the one bus
        # where assigning two ids to that UUID would be ambiguous.
        key = (canbus_iface, uuid)
        if key in self.ids:
            raise config.error("Duplicate canbus_uuid")
        new_id = self.next_nodeids.get(canbus_iface, NODEID_FIRST)
        # CMD_SET_KLIPPER_NODEID has one byte for this value, and both command
        # and response arbitration identifiers are derived from that same byte.
        # Letting Python's unbounded integer advance past 0xff does not create
        # more protocol address space; it fails later while python-can encodes
        # the assignment payload, after configuration has otherwise succeeded.
        # Reject exhaustion here so the error names the affected interface and
        # no partially connected set of MCUs is left behind.
        if new_id > NODEID_LAST:
            raise config.error("Too many CAN bus MCUs on interface '%s'"
                               % (canbus_iface,))
        self.ids[key] = new_id
        self.next_nodeids[canbus_iface] = new_id + 1
        return new_id
    def get_nodeid(self, canbus_uuid, canbus_iface):
        key = (canbus_iface, int(canbus_uuid, 16))
        if key not in self.ids:
            raise self.printer.config_error("Unknown canbus_uuid %s"
                                            % (canbus_uuid,))
        return self.ids[key]

def load_config(config):
    return PrinterCANBus(config)
