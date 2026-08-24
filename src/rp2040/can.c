// Serial over CAN emulation for rp2040 using can2040 software canbus
//
// Copyright (C) 2022-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include <string.h> // memcpy
#include "autoconf.h" // CONFIG_CANBUS_FREQUENCY
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/io.h" // readl
#include "board/irq.h" // irq_save
#include "can2040.h" // can2040_setup
#include "command.h" // DECL_CONSTANT_STR
#include "fasthash.h" // fasthash64
#include "generic/canbus.h" // canbus_notify_tx
#include "generic/canserial.h" // CANBUS_ID_ADMIN
#include "hardware/structs/resets.h" // RESETS_RESET_PIO0_BITS
#include "internal.h" // DMA_IRQ_0_IRQn
#include "sched.h" // DECL_INIT

#define GPIO_STR_CAN_RX "gpio" __stringify(CONFIG_RPXXXX_CANBUS_GPIO_RX)
#define GPIO_STR_CAN_TX "gpio" __stringify(CONFIG_RPXXXX_CANBUS_GPIO_TX)
DECL_CONSTANT_STR("RESERVE_PINS_CAN", GPIO_STR_CAN_RX "," GPIO_STR_CAN_TX);

static struct can2040 cbus;
static uint8_t can_tx_waiting;

// Transmit a packet
int
canhw_send(struct canbus_msg *msg)
{
    /*
     * can2040 reports every successful transmit through its callback, but the
     * routing task needs that notification only after can2040_transmit() has
     * rejected a frame because its four-entry software queue is full.  Record
     * that condition with PIO interrupts masked.  The library advances its
     * consumer index and invokes the callback in that interrupt, so publishing
     * the waiter before interrupts are restored closes the completion-versus-
     * rejection race without changing can2040's queue implementation.
     */
    irqstatus_t flag = irq_save();
    int ret = can2040_transmit(&cbus, (void*)msg);
    if (ret < 0) {
        writeb(&can_tx_waiting, 1);
        irq_restore(flag);
        return -1;
    }
    writeb(&can_tx_waiting, 0);
    irq_restore(flag);
    return CANMSG_DATA_LEN(msg);
}

// Setup the receive packet filter
void
canhw_set_filter(uint32_t id)
{
    // Filter not implemented (and not necessary)
}

static uint32_t last_tx_retries;

// Report interface status
void
canhw_get_status(struct canbus_status *status)
{
    struct can2040_stats stats;
    can2040_get_statistics(&cbus, &stats);
    uint32_t tx_extra = stats.tx_attempt - stats.tx_total;
    if (last_tx_retries != tx_extra)
        last_tx_retries = tx_extra - 1;

    status->rx_error = stats.parse_error;
    status->tx_retries = last_tx_retries;
    status->bus_state = CANBUS_STATE_ACTIVE;
}

// can2040 callback function - handle rx and tx notifications
static void
can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify & CAN2040_NOTIFY_TX) {
        /*
         * A completion normally requires no scheduler work: can2040 already
         * owns and transmits every entry accepted into its internal queue.
         * Release one recorded producer waiter and clear it before waking so a
         * burst of later completions can not recreate per-frame task dispatch.
         */
        if (readb(&can_tx_waiting)) {
            writeb(&can_tx_waiting, 0);
            canbus_notify_tx();
        }
        return;
    }
    if (notify & CAN2040_NOTIFY_RX)
        canbus_process_data((void*)msg);
}

// Main PIO irq handler
void
PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

void
can_init(void)
{
    // Setup canbus
    can2040_setup(&cbus, 0);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    armcm_enable_irq(PIOx_IRQHandler, PIO0_IRQ_0_IRQn, 1);

    // Start canbus
    uint32_t pclk = get_pclock_frequency(RESETS_RESET_PIO0_RESET);
    can2040_start(&cbus, pclk, CONFIG_CANBUS_FREQUENCY
                  , CONFIG_RPXXXX_CANBUS_GPIO_RX, CONFIG_RPXXXX_CANBUS_GPIO_TX);
}
DECL_INIT(can_init);
