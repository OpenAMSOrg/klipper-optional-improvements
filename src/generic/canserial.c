// Generic handling of serial over CAN support
//
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_HAVE_BOOTLOADER_REQUEST
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "canbus.h" // canbus_send
#include "canserial.h" // canserial_notify_tx
#include "command.h" // DECL_CONSTANT
#include "fasthash.h" // fasthash64
#include "sched.h" // sched_wake_task

#define CANBUS_UUID_LEN 6

#define CAN_QUERY_RESPONSE_BITS 160
#define CAN_QUERY_RESPONSE_WINDOW_US 1500000
#define CAN_RX_COMMAND_BATCH 8

enum {
    CQS_IDLE, CQS_WAITING, CQS_READY
};

/*
 * Unassigned nodes all transmit their discovery response with the same CAN
 * identifier.  Arbitration therefore can not select one response by UUID:
 * nodes whose payload bits differ instead detect an error during the data
 * field and retry together.  Sending directly from the query handler would
 * turn a normal broadcast into a synchronized collision storm on a bus with
 * several unassigned devices.
 *
 * Keep explicit ownership of the one coalesced response while its randomized
 * delay crosses scheduler contexts.  CQS_WAITING means the timer owns it and
 * suppresses repeated queries, CQS_READY means the normal transmit task owns
 * it (including controller-full retries), and CQS_IDLE means no discovery
 * response is pending.  The timer only transfers ownership and wakes the
 * transmit task; it must not bypass the CAN driver's task/wakeup contract by
 * attempting I/O from the timer callback.
 */
static uint_fast8_t can_query_event(struct timer *timer);

// Global storage
static struct canbus_data {
    uint32_t assigned_id;
    uint8_t uuid[CANBUS_UUID_LEN];

    // Tx data
    struct task_wake tx_wake;
    struct timer query_timer;
    uint32_t query_number;
    uint8_t transmit_pos, transmit_max, query_state, query_queued;

    // Rx data
    struct task_wake rx_wake;
    uint8_t receive_pos, invalid_bitrate;
    // Single-producer IRQ ring; publish indexes only after slot data is stable
    uint32_t admin_pull_pos, admin_push_pos;

    // Transfer buffers
    struct canbus_msg admin_queue[8];
    uint8_t transmit_buf[192];
    uint8_t receive_buf[192];
} CanData = {
    .query_timer.func = can_query_event,
};

static int can_send_query_response(void);


/****************************************************************
 * Data transmission over CAN
 ****************************************************************/

void
canserial_notify_tx(void)
{
    sched_wake_task(&CanData.tx_wake);
}

static uint_fast8_t
can_query_event(struct timer *timer)
{
    CanData.query_state = CQS_READY;
    canserial_notify_tx();
    return SF_DONE;
}

void
canserial_tx_task(void)
{
    if (!sched_check_wake(&CanData.tx_wake))
        return;
    uint32_t id = CanData.assigned_id;
    /*
     * canbus_send() returning a negative value means that no frame was accepted
     * and the controller will call canserial_notify_tx() when a transmit slot
     * becomes available.  Busy-waiting here is therefore both unnecessary and
     * dangerous: this is a cooperative scheduler task, so a full controller on
     * a disconnected bus could otherwise prevent every other task, including
     * shutdown processing, from running.  Retain one pending admin response and
     * return to the scheduler.  Repeated queries coalesce into that response,
     * and assigning the node makes the response obsolete instead of sending a
     * stale unassigned-node announcement.
     */
    if (CanData.query_state == CQS_READY) {
        if (!id && can_send_query_response() < 0)
            return;
        CanData.query_state = CQS_IDLE;
    }
    if (!id) {
        CanData.transmit_pos = CanData.transmit_max = 0;
        return;
    }
    struct canbus_msg msg;
    msg.id = id + 1;
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    for (;;) {
        int avail = tmax - tpos, now = avail > 8 ? 8 : avail;
        if (avail <= 0)
            break;
        msg.dlc = now;
        memcpy(msg.data, &CanData.transmit_buf[tpos], now);
        int ret = canbus_send(&msg);
        if (ret <= 0)
            break;
        tpos += now;
    }
    CanData.transmit_pos = tpos;
}
DECL_TASK(canserial_tx_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    uint32_t tpos = CanData.transmit_pos, tmax = CanData.transmit_max;
    if (tpos >= tmax)
        CanData.transmit_pos = CanData.transmit_max = tpos = tmax = 0;
    if (tmax + ce->max_size > sizeof(CanData.transmit_buf)) {
        if (tmax - tpos + ce->min_size > sizeof(CanData.transmit_buf))
            // Not enough space for message
            return;
        // Move buffer
        tmax -= tpos;
        memmove(&CanData.transmit_buf[0], &CanData.transmit_buf[tpos], tmax);
        CanData.transmit_pos = tpos = 0;
        CanData.transmit_max = tmax;
    }

    // Generate message
    uint32_t msglen = command_encode_and_frame(
        &CanData.transmit_buf[tmax], sizeof(CanData.transmit_buf) - tmax
        , ce, args);
    if (!msglen)
        return;

    // Start message transmit
    CanData.transmit_max = tmax + msglen;
    canserial_notify_tx();
}


/****************************************************************
 * CAN "admin" command handling
 ****************************************************************/

// Available commands and responses
#define CANBUS_CMD_QUERY_UNASSIGNED 0x00
#define CANBUS_CMD_SET_KLIPPER_NODEID 0x01
#define CANBUS_CMD_REQUEST_BOOTLOADER 0x02
#define CANBUS_RESP_NEED_NODEID 0x20

// Helper to verify a UUID in a command matches this chip's UUID
static int
can_check_uuid(struct canbus_msg *msg)
{
    return (msg->dlc >= 7
            && memcmp(&msg->data[1], CanData.uuid, sizeof(CanData.uuid)) == 0);
}

// Helpers to encode/decode a CAN identifier to a 1-byte "nodeid"
static int
can_get_nodeid(void)
{
    if (!CanData.assigned_id)
        return 0;
    return (CanData.assigned_id - 0x100) >> 1;
}
static uint32_t
can_decode_nodeid(int nodeid)
{
    return (nodeid << 1) + 0x100;
}

static int
can_send_query_response(void)
{
    struct canbus_msg send;
    send.id = CANBUS_ID_ADMIN_RESP;
    send.dlc = 8;
    send.data[0] = CANBUS_RESP_NEED_NODEID;
    memcpy(&send.data[1], CanData.uuid, sizeof(CanData.uuid));
    send.data[7] = CANBUS_CMD_SET_KLIPPER_NODEID;
    return canbus_send(&send);
}

static void
can_process_query_unassigned(struct canbus_msg *msg)
{
    if (CanData.assigned_id || CanData.query_state != CQS_IDLE)
        return;
    /*
     * Every discovery response uses CANBUS_ID_ADMIN_RESP.  CAN arbitration can
     * therefore not select a winner when multiple responses become ready at
     * the same bus-idle boundary; the controllers continue into the different
     * UUID payloads and produce a data-bit collision instead.  A timer spacing
     * shorter than one complete response does not prevent that condition,
     * because every timer which expires while an earlier frame owns the bus
     * becomes a simultaneous contender as soon as that frame finishes.
     *
     * Reserve 160 bit times per slot.  An eight-byte standard CAN data frame,
     * including worst-case bit stuffing, acknowledgement, end-of-frame, and
     * inter-frame spacing, fits within that conservative bound.  Scaling the
     * slot duration by the configured bitrate makes the protection valid below
     * the usual 1Mbit/s setting as well.  The fixed response window bounds
     * discovery latency; at least one slot is retained for unusually low
     * build-time bitrate settings.
     */
    uint32_t slot_us = DIV_ROUND_UP(CAN_QUERY_RESPONSE_BITS * 1000000u,
                                    CONFIG_CANBUS_FREQUENCY);
    uint32_t slot_count = CAN_QUERY_RESPONSE_WINDOW_US / slot_us;
    if (!slot_count)
        slot_count = 1;
    /*
     * A UUID-only slot is deterministic.  If two nodes happen to select the
     * same slot, every later discovery query would reproduce the collision and
     * neither node could ever be discovered.  Use the accepted-query count as
     * a hash seed so each node follows a different deterministic slot sequence.
     * This does not require a hardware random-number source, and a collision in
     * one discovery round no longer forces the same collision on every retry.
     */
    uint32_t uuid_slot = fasthash64(CanData.uuid, sizeof(CanData.uuid),
                                    CanData.query_number++);
    uint32_t delay = 100 + uuid_slot % slot_count * slot_us;
    CanData.query_state = CQS_WAITING;
    CanData.query_timer.waketime = timer_read_time() + timer_from_us(delay);
    sched_add_timer(&CanData.query_timer);
}

static void
can_id_conflict(void)
{
    CanData.assigned_id = 0;
    canbus_set_filter(CanData.assigned_id);
    shutdown("Another CAN node assigned this ID");
}

static void
can_process_set_klipper_nodeid(struct canbus_msg *msg)
{
    if (msg->dlc < 8)
        return;
    uint32_t newid = can_decode_nodeid(msg->data[7]);
    if (can_check_uuid(msg)) {
        if (newid != CanData.assigned_id) {
            CanData.assigned_id = newid;
            canbus_set_filter(CanData.assigned_id);
        }
    } else if (newid == CanData.assigned_id) {
        can_id_conflict();
    }
}

static void
can_process_request_bootloader(struct canbus_msg *msg)
{
    if (!CONFIG_HAVE_BOOTLOADER_REQUEST || !can_check_uuid(msg))
        return;
    bootloader_request();
}

// Handle an "admin" command
static void
can_process_admin(struct canbus_msg *msg)
{
    if (!msg->dlc)
        return;
    switch (msg->data[0]) {
    case CANBUS_CMD_QUERY_UNASSIGNED:
        can_process_query_unassigned(msg);
        break;
    case CANBUS_CMD_SET_KLIPPER_NODEID:
        can_process_set_klipper_nodeid(msg);
        break;
    case CANBUS_CMD_REQUEST_BOOTLOADER:
        can_process_request_bootloader(msg);
        break;
    }
}


/****************************************************************
 * CAN packet reading
 ****************************************************************/

static void
canserial_notify_rx(void)
{
    sched_wake_task(&CanData.rx_wake);
}

void
canserial_set_invalid_bitrate(void)
{
    /*
     * Board initialization runs before sched_main() establishes the recovery
     * point used by shutdown().  A CAN driver therefore can not safely call
     * shutdown directly when its runtime peripheral clock can not represent
     * the configured bitrate.  Publish the failure and wake this existing task;
     * its first scheduler invocation occurs after shutdown handling is live.
     * The driver returns before it programs or starts the CAN controller, so an
     * invalid configuration can never place approximately timed frames on the
     * bus even if its peripheral clock and pin mux were already prepared.
     */
    writeb(&CanData.invalid_bitrate, 1);
    canserial_notify_rx();
}

DECL_CONSTANT("RECEIVE_WINDOW", ARRAY_SIZE(CanData.receive_buf));

// Handle incoming data (called from IRQ handler)
void
canserial_process_data(struct canbus_msg *msg)
{
    uint32_t id = msg->id;
    if (CanData.assigned_id && id == CanData.assigned_id) {
        // Add to incoming data buffer
        int rpos = CanData.receive_pos;
        uint32_t len = CANMSG_DATA_LEN(msg);
        if (len > sizeof(CanData.receive_buf) - rpos)
            return;
        memcpy(&CanData.receive_buf[rpos], msg->data, len);
        CanData.receive_pos = rpos + len;
        canserial_notify_rx();
    } else if (id == CANBUS_ID_ADMIN
               || (CanData.assigned_id && id == CanData.assigned_id + 1)) {
        // Add to admin command queue
        int is_query = (id == CANBUS_ID_ADMIN && msg->dlc
                        && msg->data[0] == CANBUS_CMD_QUERY_UNASSIGNED);
        if (is_query) {
            /*
             * Discovery is a broadcast and scanners commonly repeat it while
             * collecting delayed responses.  Queueing every copy until the
             * task handles the first one can fill all eight admin slots with
             * interchangeable work, causing a node-id assignment or conflict
             * frame to be dropped.  Once one query is queued, timer-owned, or
             * transmit-owned, it represents the complete discovery obligation
             * for this node.  Assigned nodes have no obligation at all and can
             * reject the broadcast before it consumes queue capacity.
             *
             * query_queued covers the interval before can_process_admin() has
             * established query_state.  The receive interrupt is the only
             * producer, and the task clears this marker only after processing
             * that exact queue entry, so a newly accepted query can not hide
             * an older response or disturb the randomized retry sequence.
             */
            if (CanData.assigned_id || readb(&CanData.query_queued)
                || readb(&CanData.query_state) != CQS_IDLE)
                return;
        }
        uint32_t pushp = CanData.admin_push_pos;
        /*
         * The queue positions are monotonically increasing uint32_t counters,
         * so they necessarily wrap on a sufficiently busy or long-lived bus.
         * Adding the queue depth to admin_pull_pos before comparing can wrap
         * that right-hand side first.  Near UINT32_MAX an empty queue would
         * then appear full; because the producer stops advancing when full,
         * admin reception could remain wedged permanently.  Unsigned
         * subtraction preserves the small producer-consumer distance across
         * rollover, which is safe because that distance is bounded by this
         * eight-entry queue.
         */
        uint32_t pullp = readl(&CanData.admin_pull_pos);
        if (pushp - pullp >= ARRAY_SIZE(CanData.admin_queue))
            // No space - drop message
            return;
        uint32_t pos = pushp % ARRAY_SIZE(CanData.admin_queue);
        memcpy(&CanData.admin_queue[pos], msg, sizeof(*msg));
        if (is_query)
            writeb(&CanData.query_queued, 1);
        /*
         * The task treats push_pos as proof that the complete queue slot is
         * readable.  Use the I/O accessor as a compiler release barrier so the
         * structure copy can not move after that publication.  This is an SPSC
         * ring: only this receive interrupt advances push_pos, while only the
         * task advances pull_pos, so no read-modify-write lock is required.
         */
        writel(&CanData.admin_push_pos, pushp + 1);
        canserial_notify_rx();
    }
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(int len)
{
    int copied = 0;
    for (;;) {
        int rpos = readb(&CanData.receive_pos);
        int needcopy = rpos - len;
        if (needcopy) {
            memmove(&CanData.receive_buf[copied]
                    , &CanData.receive_buf[copied + len], needcopy - copied);
            copied = needcopy;
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&CanData.receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        CanData.receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Task to process incoming commands and admin messages
void
canserial_rx_task(void)
{
    if (!sched_check_wake(&CanData.rx_wake))
        return;
    if (readb(&CanData.invalid_bitrate))
        shutdown("Invalid CAN bitrate");

    // Process pending admin messages
    for (;;) {
        uint32_t pushp = readl(&CanData.admin_push_pos);
        uint32_t pullp = CanData.admin_pull_pos;
        if (pushp == pullp)
            break;
        uint32_t pos = pullp % ARRAY_SIZE(CanData.admin_queue);
        struct canbus_msg *msg = &CanData.admin_queue[pos];
        uint32_t id = msg->id;
        if (CanData.assigned_id && id == CanData.assigned_id + 1)
            can_id_conflict();
        else if (id == CANBUS_ID_ADMIN) {
            can_process_admin(msg);
            if (msg->dlc && msg->data[0] == CANBUS_CMD_QUERY_UNASSIGNED)
                writeb(&CanData.query_queued, 0);
        }
        /*
         * Return this slot only after task-side processing no longer uses it.
         */
        writel(&CanData.admin_pull_pos, pullp + 1);
    }

    /*
     * A Klipper command block commonly spans several eight-byte CAN frames,
     * and a SocketCAN sender can place several complete blocks on the bus in
     * one scheduling interval.  Processing only the first block made every
     * already-buffered successor wait for another scheduler pass.
     *
     * Dispatch a bounded group in one task invocation.  Remove each block
     * before invoking the next command so the interrupt regains receive-buffer
     * capacity promptly; console_pop_input() retains its interrupt-protected
     * retry if a CAN frame arrives during compaction.  The task itself now owns
     * the decision to continue, so compaction no longer schedules a redundant
     * future pass merely because bytes remain.  Acknowledgements remain per
     * protocol block and in sequence.  Bound the group so command input can not
     * monopolize a cooperative scheduler pass when other MCU tasks are ready.
     */
    uint_fast8_t command_count;
    for (command_count = 0; command_count < CAN_RX_COMMAND_BATCH;
         command_count++) {
        uint_fast8_t rpos = readb(&CanData.receive_pos), pop_count;
        int ret = command_find_block(CanData.receive_buf, rpos, &pop_count);
        if (!ret)
            break;
        if (ret > 0)
            command_dispatch(CanData.receive_buf, pop_count);
        console_pop_input(pop_count);
        if (ret > 0)
            command_send_ack();
    }
    if (command_count == CAN_RX_COMMAND_BATCH
        && readb(&CanData.receive_pos))
        canserial_notify_rx();
}
DECL_TASK(canserial_rx_task);


/****************************************************************
 * Setup and shutdown
 ****************************************************************/

DECL_ENUMERATION("canbus_bus_state", "active", CANBUS_STATE_ACTIVE);
DECL_ENUMERATION("canbus_bus_state", "warn", CANBUS_STATE_WARN);
DECL_ENUMERATION("canbus_bus_state", "passive", CANBUS_STATE_PASSIVE);
DECL_ENUMERATION("canbus_bus_state", "off", CANBUS_STATE_OFF);

void
command_get_canbus_status(uint32_t *args)
{
    struct canbus_status status;
    memset(&status, 0, sizeof(status));
    canhw_get_status(&status);
    sendf("canbus_status rx_error=%u tx_error=%u tx_retries=%u"
          " canbus_bus_state=%u"
          , status.rx_error, status.tx_error, status.tx_retries
          , status.bus_state);
}
DECL_COMMAND_FLAGS(command_get_canbus_status, HF_IN_SHUTDOWN
                   , "get_canbus_status");

void
command_get_canbus_id(uint32_t *args)
{
    sendf("canbus_id canbus_uuid=%.*s canbus_nodeid=%u"
          , sizeof(CanData.uuid), CanData.uuid, can_get_nodeid());
}
DECL_COMMAND_FLAGS(command_get_canbus_id, HF_IN_SHUTDOWN, "get_canbus_id");

void
canserial_set_uuid(uint8_t *raw_uuid, uint32_t raw_uuid_len)
{
    uint64_t hash = fasthash64(raw_uuid, raw_uuid_len, 0xA16231A7);
    memcpy(CanData.uuid, &hash, sizeof(CanData.uuid));
    canserial_notify_rx();
}

void
canserial_shutdown(void)
{
    /*
     * Shutdown removes scheduler timers before running shutdown callbacks, so
     * a CQS_WAITING response can no longer become ready.  Reset the logical
     * ownership as well as relying on timer removal: shutdown still wakes the
     * transmit task, and leaving READY set could emit a stale discovery frame
     * while the MCU is reporting its shutdown state.
     */
    CanData.query_state = CQS_IDLE;
    canserial_notify_tx();
    canserial_notify_rx();
}
DECL_SHUTDOWN(canserial_shutdown);
