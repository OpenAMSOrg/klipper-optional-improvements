# CANBUS protocol

This document describes the protocol Klipper uses to communicate over
[CAN bus](https://en.wikipedia.org/wiki/CAN_bus). See
[CANBUS.md](CANBUS.md) for information on configuring Klipper with CAN
bus.

## Micro-controller id assignment

Klipper uses only CAN 2.0A standard size CAN bus packets, which are
limited to 8 data bytes and an 11-bit CAN bus identifier. In order to
support efficient communication, each micro-controller is assigned at
run-time a unique 1-byte CAN bus nodeid (`canbus_nodeid`) for general
Klipper command and response traffic. Klipper command messages going
from host to micro-controller use the CAN bus id of `canbus_nodeid *
2 + 256`, while Klipper response messages from micro-controller to
host use `canbus_nodeid * 2 + 256 + 1`.

Each micro-controller has a factory assigned unique chip identifier
that is used during id assignment. This identifier can exceed the
length of one CAN packet, so a hash function is used to generate a
unique 6-byte id (`canbus_uuid`) from the factory id.

## Security and trust model

The Klipper CAN protocol is designed for a physically controlled,
trusted machine network. It provides neither sender authentication nor
confidentiality. CAN identifiers, admin commands, UUIDs, and Klipper
protocol data are transmitted in clear text, and any device able to
transmit on the bus can construct frames using the same identifiers.

The CAN hardware CRC and the CRC in Klipper protocol message blocks
detect accidental transmission errors. They are not keyed message
authentication codes: a device intentionally constructing a frame can
calculate both CRC values. Similarly, a `canbus_uuid` is a truncated
hash used to select a micro-controller during node-id assignment. It is
observable during discovery and is not proof of the sender's or
recipient's identity.

As a result, access to the CAN wiring, a CAN adapter, or a host process
permitted to transmit SocketCAN frames must be treated as trusted
access to the printer. Such access can be used to:

* impersonate a host or micro-controller and inject or replay Klipper
  protocol data;
* assign node ids, cause node-id conflict shutdowns, or request entry
  into a supported bootloader;
* consume bus bandwidth, force retransmissions, or otherwise prevent
  timely communication; and
* observe all application and administration traffic.

Physical access controls, host device permissions, and network
isolation must enforce this trust boundary. The CAN bus should not be
directly bridged to an untrusted network or shared with untrusted
devices. Klipper's checksums, retransmission logic, and shutdown
handling improve reliability in the presence of faults, but they are
not security controls against a malicious bus participant. Adding such
controls would require a new authenticated protocol together with key
provisioning and compatibility rules; it can not be added by treating
the existing UUID or CRC fields as credentials.

## Admin messages

Admin messages are used for id assignment. Admin messages sent from
host to micro-controller use the CAN bus id `0x3f0` and messages sent
from micro-controller to host use the CAN bus id `0x3f1`. All
micro-controllers listen to messages on id `0x3f0`; that id can be
thought of as a "broadcast address".

### CMD_QUERY_UNASSIGNED message

This command queries all micro-controllers that have not yet been
assigned a `canbus_nodeid`. Unassigned micro-controllers will respond
with a RESP_NEED_NODEID response message. Klipper micro-controllers
delay that response by an interval derived from their `canbus_uuid` so
that multiple unassigned nodes do not normally transmit the response
with the same CAN bus id at the same time. Response slots are separated
by enough time to transmit the complete response at the configured CAN
bus frequency, and the slot selection changes on later queries so that
a collision in one discovery round does not persist indefinitely.

The CMD_QUERY_UNASSIGNED message format is:
`<1-byte message_id = 0x00>`

### CMD_SET_KLIPPER_NODEID message

This command assigns a `canbus_nodeid` to the micro-controller with a
given `canbus_uuid`.

The CMD_SET_KLIPPER_NODEID message format is:
`<1-byte message_id = 0x01><6-byte canbus_uuid><1-byte canbus_nodeid>`

### RESP_NEED_NODEID message

The RESP_NEED_NODEID message format is:
`<1-byte message_id = 0x20><6-byte canbus_uuid><1-byte set_klipper_nodeid = 0x01>`

### CMD_REQUEST_BOOTLOADER message

This command asks the micro-controller with a given `canbus_uuid` to
enter its bootloader. It is accepted only by firmware builds that
support a bootloader request. The command may be sent before a nodeid
has been assigned because, like the other admin commands, it uses the
broadcast CAN bus id `0x3f0` and selects its destination by UUID.

The CMD_REQUEST_BOOTLOADER message format is:
`<1-byte message_id = 0x02><6-byte canbus_uuid>`

The `canbus_uuid` selects the target but does not authenticate the
sender. See [Bootloader Entry](Bootloader_Entry.md) for the supported
bootloader transition mechanisms.

## Data Packets

A micro-controller that has been assigned a nodeid via the
CMD_SET_KLIPPER_NODEID command can send and receive data packets.

The packet data in messages using the node's receive CAN bus id
(`canbus_nodeid * 2 + 256`) are simply appended to a buffer, and when
a complete [mcu protocol message](Protocol.md) is found its contents
are parsed and processed. The data is treated as a byte stream - there
is no requirement for the start of a Klipper message block to align
with the start of a CAN bus packet.

Similarly, mcu protocol message responses are sent from
micro-controller to host by copying the message data into one or more
packets with the node's transmit CAN bus id (`canbus_nodeid * 2 +
256 + 1`).
