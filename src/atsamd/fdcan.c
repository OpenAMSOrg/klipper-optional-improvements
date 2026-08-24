// CANbus support on atsame51 chips
//
// Copyright (C) 2021-2025  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2019 Eug Krashtan <eug.krashtan@gmail.com>
// Copyright (C) 2020 Pontus Borg <glpontus@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_save
#include "command.h" // DECL_CONSTANT_STR
#include "generic/armcm_boot.h" // armcm_enable_irq
#include "generic/canbus.h" // canbus_notify_tx
#include "generic/canserial.h" // CANBUS_ID_ADMIN
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT


/****************************************************************
 * Pin configuration
 ****************************************************************/

#if CONFIG_ATSAMD_CANBUS_PA23_PA22
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PA23,PA22");
 #define GPIO_Rx GPIO('A', 23)
 #define GPIO_Tx GPIO('A', 22)
 #define CANx_GCLK_ID CAN0_GCLK_ID
#elif CONFIG_ATSAMD_CANBUS_PA25_PA24
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PA25,PA24");
 #define GPIO_Rx GPIO('A', 25)
 #define GPIO_Tx GPIO('A', 24)
 #define CANx_GCLK_ID CAN0_GCLK_ID
#elif CONFIG_ATSAMD_CANBUS_PB11_PB10
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PB11,PB10");
 #define GPIO_Rx GPIO('B', 11)
 #define GPIO_Tx GPIO('B', 10)
 #define CANx_GCLK_ID CAN1_GCLK_ID
#elif CONFIG_ATSAMD_CANBUS_PB13_PB12
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PB13,PB12");
 #define GPIO_Rx GPIO('B', 13)
 #define GPIO_Tx GPIO('B', 12)
 #define CANx_GCLK_ID CAN1_GCLK_ID
#elif CONFIG_ATSAMD_CANBUS_PB15_PB14
 DECL_CONSTANT_STR("RESERVE_PINS_CAN", "PB15,PB14");
 #define GPIO_Rx GPIO('B', 15)
 #define GPIO_Tx GPIO('B', 14)
 #define CANx_GCLK_ID CAN1_GCLK_ID
#endif

#if CANx_GCLK_ID == CAN0_GCLK_ID && CONFIG_MACH_SAMC21
 #define CAN_FUNCTION 'G'
 #define CANx CAN0
 #define CANx_IRQn CAN0_IRQn
 #define MCLK_AHBMASK_CANx MCLK_AHBMASK_CAN0
#elif CANx_GCLK_ID == CAN1_GCLK_ID && CONFIG_MACH_SAMC21
 #define CAN_FUNCTION 'G'
 #define CANx CAN1
 #define CANx_IRQn CAN1_IRQn
 #define MCLK_AHBMASK_CANx MCLK_AHBMASK_CAN1
#elif CANx_GCLK_ID == CAN0_GCLK_ID
 #define CAN_FUNCTION 'I'
 #define CANx CAN0
 #define CANx_IRQn CAN0_IRQn
#else
 #define CAN_FUNCTION 'H'
 #define CANx CAN1
 #define CANx_IRQn CAN1_IRQn
#endif


/****************************************************************
 * Message ram layout
 ****************************************************************/

struct fdcan_fifo {
    uint32_t id_section;
    uint32_t dlc_section;
    uint32_t data[8 / 4];
};

#define FDCAN_XTD (1<<30)
#define FDCAN_RTR (1<<29)

struct fdcan_msg_ram {
    uint32_t FLS[28]; // Filter list standard
    uint32_t FLE[16]; // Filter list extended
    struct fdcan_fifo RXF0[16];
    struct fdcan_fifo RXF1[3];
    uint32_t TEF[6]; // Tx event FIFO
    struct fdcan_fifo TXFIFO[16];
    uint32_t unused[22];
};

// Message ram is in regular memory
static struct fdcan_msg_ram MSG_RAM;


/****************************************************************
 * CANbus code
 ****************************************************************/

#define FDCAN_IE_TC        (CAN_IE_TCE | CAN_IE_TCFE | CAN_IE_TFEE)

// Transmit a packet
int
canhw_send(struct canbus_msg *msg)
{
    uint32_t txfqs = CANx->TXFQS.reg;
    if (txfqs & CAN_TXFQS_TFQF) {
        /*
         * A completion interrupt is useful only after software has found the
         * FIFO full and needs a future capacity notification.  Leaving it
         * enabled continuously interrupts and wakes the cooperative scheduler
         * after every frame, including the common case where no message is
         * waiting.  Arm it on demand while interrupts are masked, then re-read
         * TFQF so a completion between the first check and the IE write can not
         * become a lost wakeup.  Clear stale completion flags before arming;
         * otherwise an old successful send could cause an immediate wake while
         * the FIFO holding the current burst is still full.
         */
        irqstatus_t flag = irq_save();
        CANx->IR.reg = FDCAN_IE_TC;
        CANx->IE.reg |= FDCAN_IE_TC;
        barrier();
        txfqs = CANx->TXFQS.reg;
        if (txfqs & CAN_TXFQS_TFQF) {
            irq_restore(flag);
            return -1;
        }
        // Capacity already returned; this call can fill it without a wakeup.
        CANx->IE.reg &= ~FDCAN_IE_TC;
        irq_restore(flag);
    }

    uint32_t w_index = ((txfqs & CAN_TXFQS_TFQPI_Msk) >> CAN_TXFQS_TFQPI_Pos);
    struct fdcan_fifo *txfifo = &MSG_RAM.TXFIFO[w_index];
    uint32_t ids;
    if (msg->id & CANMSG_ID_EFF)
        ids = (msg->id & 0x1fffffff) | FDCAN_XTD;
    else
        ids = (msg->id & 0x7ff) << 18;
    ids |= msg->id & CANMSG_ID_RTR ? FDCAN_RTR : 0;
    txfifo->id_section = ids;
    txfifo->dlc_section = (msg->dlc & 0x0f) << 16;
    txfifo->data[0] = msg->data32[0];
    txfifo->data[1] = msg->data32[1];
    __DMB();
    CANx->TXBAR.reg;
    CANx->TXBAR.reg = ((uint32_t)1 << w_index);
    return CANMSG_DATA_LEN(msg);
}

static void
can_filter(uint32_t index, uint32_t id)
{
    MSG_RAM.FLS[index] = ((0x2 << 30) // Classic filter
                          | (0x1 << 27) // Store in Rx FIFO 0 if filter matches
                          | (id << 16)
                          | 0x7FF); // mask all enabled
}

// Setup the receive packet filter
void
canhw_set_filter(uint32_t id)
{
    if (!CONFIG_CANBUS_FILTER)
        return;
    /* Request initialisation */
    CANx->CCCR.reg |= CAN_CCCR_INIT;
    /* Wait the acknowledge */
    while (!(CANx->CCCR.reg & CAN_CCCR_INIT))
        ;
    /* Enable configuration change */
    CANx->CCCR.reg |= CAN_CCCR_CCE;

    // Load filter
    can_filter(0, CANBUS_ID_ADMIN);
    can_filter(1, id);
    can_filter(2, id + 1);

    /*
     * The standard-id filter list does not reject extended frames; the reset
     * global-filter action routes non-matching extended identifiers to FIFO 0.
     * A remote frame can also match one of the standard identifiers above even
     * though canserial consumes data frames only.  Both cases spend an entry in
     * the three-element receive FIFO and invoke the interrupt handler before
     * software discards them, so unrelated traffic can crowd out a command.
     * Reject remote frames globally and select the explicit reject action for
     * both non-matching identifier formats.  This function returns above when
     * CONFIG_CANBUS_FILTER is disabled, preserving arbitrary frame forwarding
     * for USB-to-CAN bridge builds.
     */
    uint32_t flssa = (uint32_t)MSG_RAM.FLS - CAN0_MSG_RAM_ADDR;
    CANx->SIDFC.reg = flssa | ((id ? 3 : 1) << CAN_SIDFC_LSS_Pos);
    CANx->GFC.reg = (CAN_GFC_RRFE | CAN_GFC_RRFS
                     | 0x02 << CAN_GFC_ANFE_Pos
                     | 0x02 << CAN_GFC_ANFS_Pos);

    /* Leave the initialisation mode for the filter */
    barrier();
    CANx->CCCR.reg &= ~CAN_CCCR_CCE;
    CANx->CCCR.reg &= ~CAN_CCCR_INIT;
}

static struct {
    uint32_t rx_error, tx_error, rx_overflows, bus_off;
} CAN_Errors;

// Report interface status
void
canhw_get_status(struct canbus_status *status)
{
    irqstatus_t flag = irq_save();
    uint32_t psr = CANx->PSR.reg, lec = psr & CAN_PSR_LEC_Msk;
    if (lec && lec != 7) {
        // Reading PSR clears it - so update state here
        if (lec >= 3 && lec <= 5)
            CAN_Errors.tx_error += 1;
        else
            CAN_Errors.rx_error += 1;
    }
    uint32_t rx_error = CAN_Errors.rx_error, tx_error = CAN_Errors.tx_error;
    uint32_t rx_overflows = CAN_Errors.rx_overflows;
    uint32_t bus_off = CAN_Errors.bus_off;
    irq_restore(flag);

    status->rx_error = rx_error;
    status->tx_error = tx_error;
    status->rx_overflows = rx_overflows;
    status->bus_off = bus_off;
    if (psr & CAN_PSR_BO)
        status->bus_state = CANBUS_STATE_OFF;
    else if (psr & CAN_PSR_EP)
        status->bus_state = CANBUS_STATE_PASSIVE;
    else if (psr & CAN_PSR_EW)
        status->bus_state = CANBUS_STATE_WARN;
    else
        status->bus_state = 0;
}

// This function handles CAN global interrupts
void
CAN_IRQHandler(void)
{
    uint32_t ir = CANx->IR.reg;

    if (ir & CAN_IE_RF0NE) {
        CANx->IR.reg = CAN_IE_RF0NE;

        /*
         * RF0N reports that FIFO 0 transitioned to containing new data; it is
         * not one interrupt record per FIFO element.  Several frames can arrive
         * before this handler runs.  After RF0N is cleared, acknowledging only
         * the oldest element may leave a quiet trailing frame in the FIFO with
         * no new transition to raise another interrupt.  Re-read the hardware
         * fill level after every acknowledgement and drain the complete batch.
         * Acknowledging by the current get index preserves receive order while
         * promptly returning every one of the three scarce FIFO slots.
         */
        for (;;) {
            uint32_t rxf0s = CANx->RXF0S.reg;
            if (!(rxf0s & CAN_RXF0S_F0FL_Msk))
                break;
            // Read and ack data packet
            uint32_t idx = (rxf0s & CAN_RXF0S_F0GI_Msk) >> CAN_RXF0S_F0GI_Pos;
            struct fdcan_fifo *rxf0 = &MSG_RAM.RXF0[idx];
            uint32_t ids = rxf0->id_section;
            struct canbus_msg msg;
            if (ids & FDCAN_XTD)
                msg.id = (ids & 0x1fffffff) | CANMSG_ID_EFF;
            else
                msg.id = (ids >> 18) & 0x7ff;
            msg.id |= ids & FDCAN_RTR ? CANMSG_ID_RTR : 0;
            msg.dlc = (rxf0->dlc_section >> 16) & 0x0f;
            msg.data32[0] = rxf0->data[0];
            msg.data32[1] = rxf0->data[1];
            barrier();
            CANx->RXF0A.reg = idx;

            // Process packet
            canbus_process_data(&msg);
        }
    }
    if (ir & CAN_IR_RF0L) {
        /*
         * RF0L is the only conclusive evidence that the controller discarded
         * a complete, otherwise valid frame because software did not return a
         * FIFO slot in time.  LEC and the receive error counter describe CAN
         * protocol faults on the wire; they do not change for this local
         * resource loss.  Preserve a separate monotonically wrapping count so
         * a host can distinguish signal-integrity trouble from IRQ latency or
         * an undersized receive path.  One interrupt flag represents at least
         * one lost frame, so the count is deliberately a lower bound when a
         * sustained burst overwrites more than one element before service.
         */
        CANx->IR.reg = CAN_IR_RF0L;
        CAN_Errors.rx_overflows++;
    }
    if (ir & FDCAN_IE_TC) {
        /*
         * IR records completions even while their interrupt sources are
         * masked, so a receive interrupt can observe and clear an unrelated
         * transmit flag.  Notify only if canhw_send() actually armed the source
         * while waiting for capacity.  Mask it before waking the task so one
         * completion releases one waiter instead of restoring per-frame IRQs.
         */
        uint32_t tx_ie = CANx->IE.reg & FDCAN_IE_TC;
        CANx->IR.reg = FDCAN_IE_TC;
        if (tx_ie) {
            CANx->IE.reg &= ~FDCAN_IE_TC;
            canbus_notify_tx();
        }
    }
    if (ir & CAN_IR_BO) {
        /*
         * The M_CAN core enters software-initialization mode by setting INIT
         * when its transmit error counter reaches bus-off.  In that mode it is
         * completely disconnected from the bus and will remain so even after
         * the wiring fault disappears; there is no automatic bus-off-management
         * option.  Clear INIT after acknowledging the event to start the ISO
         * CAN recovery sequence.  The controller still waits for the required
         * 129 occurrences of 11 recessive bits, so this does not bypass or
         * shorten the protocol's recovery safety interval.
         */
        CANx->IR.reg = CAN_IR_BO;
        CAN_Errors.bus_off++;
        CANx->CCCR.reg &= ~CAN_CCCR_INIT;
    }
    if (ir & (CAN_IR_PED | CAN_IR_PEA)) {
        // Bus error
        uint32_t psr = CANx->PSR.reg;
        CANx->IR.reg = CAN_IR_PED | CAN_IR_PEA;
        uint32_t lec = psr & CAN_PSR_LEC_Msk;
        if (lec && lec != 7) {
            if (lec >= 3 && lec <= 5)
                CAN_Errors.tx_error += 1;
            else
                CAN_Errors.rx_error += 1;
        }
    }
}

static inline const uint32_t
make_btr(uint32_t sjw,       // Sync jump width, ... hmm
         uint32_t time_seg1, // time segment before sample point, 1 .. 16
         uint32_t time_seg2, // time segment after sample point, 1 .. 8
         uint32_t brp)       // Baud rate prescaler, 1 .. 1024
{
    return (((uint32_t)(sjw-1)) << CAN_NBTP_NSJW_Pos
            | ((uint32_t)(time_seg1-1)) << CAN_NBTP_NTSEG1_Pos
            | ((uint32_t)(time_seg2-1)) << CAN_NBTP_NTSEG2_Pos
            | ((uint32_t)(brp - 1)) << CAN_NBTP_NBRP_Pos);
}

static inline const uint32_t
compute_btr(uint32_t pclock, uint32_t bitrate)
{
    /*
        Some equations:
        Tpclock = 1 / pclock
        Tq      = brp * Tpclock
        Tbs1    = Tq * TS1
        Tbs2    = Tq * TS2
        NominalBitTime = Tq + Tbs1 + Tbs2
        BaudRate = 1/NominalBitTime
        Bit value sample point is after Tq+Tbs1. Ideal sample point
        is at 87.5% of NominalBitTime
        Use the lowest brp where ts1 and ts2 are in valid range
     */

    /*
     * Integer division here used to discard any pclock remainder.  The NBTP
     * value then described the nearest lower number of peripheral clocks while
     * CANBUS_FREQUENCY still advertised the requested rate to the host.  CAN
     * nodes need the same nominal bit period to arbitrate reliably, so silently
     * selecting a nearby rate is worse than refusing an invalid build-time
     * choice.  Require an exact peripheral-clock division and a legal 1..512
     * prescaler before programming the controller.
     */
    if (!bitrate || pclock % bitrate)
        return CAN_INVALID_BTR;
    uint32_t bit_clocks = pclock / bitrate; // clock ticks per bit

    uint32_t sjw = 2;
    uint32_t qs;
    // Find number of time quantas that gives us the exact wanted bit time
    for (qs = 18; qs > 9; qs--) {
        // check that bit_clocks / quantas is an integer
        if (bit_clocks % qs)
            continue;
        uint32_t brp = bit_clocks / qs;
        if (!brp || brp > 512)
            continue;
        uint32_t time_seg2 = qs / 8; // sample at ~87.5%
        uint32_t time_seg1 = qs - (1 + time_seg2);
        return make_btr(sjw, time_seg1, time_seg2, brp);
    }
    return CAN_INVALID_BTR;
}

void
can_init(void)
{
#if CONFIG_HAVE_SAMD_USB
    if (!CONFIG_USB) {
        // The FDCAN peripheral only seems to run if at least one
        // other peripheral is also enabled.
        enable_pclock(USB_GCLK_ID, ID_USB);
        USB->DEVICE.CTRLA.reg = USB_CTRLA_ENABLE;
    }
#endif

#if CONFIG_MACH_SAMC21
    MCLK->AHBMASK.reg |= MCLK_AHBMASK_CANx;
#endif
    enable_pclock(CANx_GCLK_ID, -1);

    gpio_peripheral(GPIO_Rx, CAN_FUNCTION, 1);
    gpio_peripheral(GPIO_Tx, CAN_FUNCTION, 0);

    uint32_t pclock = get_pclock_frequency(CANx_GCLK_ID);

    uint32_t btr = compute_btr(pclock, CONFIG_CANBUS_FREQUENCY);
    if (btr == CAN_INVALID_BTR) {
        canserial_set_invalid_bitrate();
        return;
    }

    /*##-1- Configure the CAN #######################################*/

    /* Exit from sleep mode */
    CANx->CCCR.reg &= ~CAN_CCCR_CSR;
    /* Wait the acknowledge */
    while (CANx->CCCR.reg & CAN_CCCR_CSA)
        ;
    /* Request initialisation */
    CANx->CCCR.reg |= CAN_CCCR_INIT;
    /* Wait the acknowledge */
    while (!(CANx->CCCR.reg & CAN_CCCR_INIT))
        ;
    /* Enable configuration change */
    CANx->CCCR.reg |= CAN_CCCR_CCE;

    /* Disable protocol exception handling */
    CANx->CCCR.reg |= CAN_CCCR_PXHD;

    CANx->NBTP.reg = btr;

    /* Setup message RAM addresses */
    uint32_t f0sa = (uint32_t)MSG_RAM.RXF0 - CAN0_MSG_RAM_ADDR;
    CANx->RXF0C.reg = f0sa | (ARRAY_SIZE(MSG_RAM.RXF0) << CAN_RXF0C_F0S_Pos);
    /*
     * Klipper and gs_usb use classical CAN exclusively, so every receive and
     * transmit element needs eight data bytes, not the reset-independent
     * 64-byte CAN-FD maximum.  The smaller element size expands each active
     * FIFO from three records to sixteen while retaining the existing 212-word
     * message-RAM allocation.
     *
     * Deeper receive buffering tolerates interrupt latency without overwriting
     * commands, while deeper transmit buffering lets the routing task submit a
     * burst before it needs the demand-driven capacity interrupt.  FIFO 1
     * retains its prior entry count, and explicit tail padding documents and
     * preserves the total allocation instead of silently changing this
     * hardware-visible memory contract.
     */
    CANx->RXESC.reg = 0;
    uint32_t tbsa = (uint32_t)MSG_RAM.TXFIFO - CAN0_MSG_RAM_ADDR;
    CANx->TXBC.reg = tbsa | (ARRAY_SIZE(MSG_RAM.TXFIFO) << CAN_TXBC_TFQS_Pos);
    CANx->TXESC.reg = 0;

    /* Leave the initialisation mode */
    CANx->CCCR.reg &= ~CAN_CCCR_CCE;
    CANx->CCCR.reg &= ~CAN_CCCR_INIT;

    /*##-2- Configure the CAN Filter #######################################*/
    canhw_set_filter(0);

    /*##-3- Configure Interrupts #################################*/
    armcm_enable_irq(CAN_IRQHandler, CANx_IRQn, 1);
    CANx->ILE.reg = CAN_ILE_EINT0;
    CANx->IE.reg = (CAN_IE_RF0NE | CAN_IE_RF0LE | CAN_IE_BOE
                    | CAN_IE_PEDE | CAN_IE_PEAE);
}
DECL_INIT(can_init);
