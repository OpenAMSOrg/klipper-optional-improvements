// Support for gathering acceleration data from ADXL345 chip
//
// Copyright (C) 2020-2023  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memcpy
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_TASK
#include "sensor_bulk.h" // sensor_bulk_report
#include "spicmds.h" // spidev_transfer
#include "trigger_analog.h" // trigger_analog_update
#include "trsync.h" // trsync_do_trigger

#define ADXL_TA_SLOTS 2

struct adxl345 {
    struct timer timer;
    uint32_t rest_ticks;
    struct spidev_s *spi;
    uint8_t flags;
    struct trigger_analog *ta[ADXL_TA_SLOTS];
    uint8_t ta_axis[ADXL_TA_SLOTS];
    // Carrier-loss ("tone") contact detector: tracks the amplitude of
    // the speed-locked stepper vibration tone at the sensor via a
    // block I/Q correlation (single-bin DFT).  When the nozzle lands
    // and toolhead motion stops while the drivetrain keeps moving, the
    // tone at the sensor collapses - detecting soft contacts that
    // produce no impact transient.
    struct trsync *tone_ts;
    uint32_t tone_clock, tone_min_amp, tone_ema;
    int32_t tone_i, tone_q;
    uint16_t tone_phase, tone_phase_inc, tone_count;
    uint8_t tone_flags, tone_axis, tone_reason, tone_block_shift;
    uint8_t tone_ratio8, tone_warm, tone_warm_target;
    uint8_t tone_consec, tone_consec_target;
    struct sensor_bulk sb;
};

enum {
    AX_PENDING = 1<<0,
};

enum {
    TONE_ACTIVE = 1<<0, TONE_AWAIT = 1<<1,
};

#if CONFIG_WANT_ADXL345_PROBE
static const int8_t adxl_sin_tab[256] = {
    0, 3, 6, 9, 12, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46,
    49, 51, 54, 57, 60, 63, 65, 68, 71, 73, 76, 78, 81, 83, 85, 88,
    90, 92, 94, 96, 98, 100, 102, 104, 106, 107, 109, 111, 112, 113,
    115, 116, 117, 118, 120, 121, 122, 122, 123, 124, 125, 125, 126,
    126, 126, 127, 127, 127, 127, 127, 127, 127, 126, 126, 126, 125,
    125, 124, 123, 122, 122, 121, 120, 118, 117, 116, 115, 113, 112,
    111, 109, 107, 106, 104, 102, 100, 98, 96, 94, 92, 90, 88, 85,
    83, 81, 78, 76, 73, 71, 68, 65, 63, 60, 57, 54, 51, 49, 46, 43,
    40, 37, 34, 31, 28, 25, 22, 19, 16, 12, 9, 6, 3, 0, -3, -6, -9,
    -12, -16, -19, -22, -25, -28, -31, -34, -37, -40, -43, -46, -49,
    -51, -54, -57, -60, -63, -65, -68, -71, -73, -76, -78, -81, -83,
    -85, -88, -90, -92, -94, -96, -98, -100, -102, -104, -106, -107,
    -109, -111, -112, -113, -115, -116, -117, -118, -120, -121, -122,
    -122, -123, -124, -125, -125, -126, -126, -126, -127, -127, -127,
    -127, -127, -127, -127, -126, -126, -126, -125, -125, -124, -123,
    -122, -122, -121, -120, -118, -117, -116, -115, -113, -112, -111,
    -109, -107, -106, -104, -102, -100, -98, -96, -94, -92, -90, -88,
    -85, -83, -81, -78, -76, -73, -71, -68, -65, -63, -60, -57, -54,
    -51, -49, -46, -43, -40, -37, -34, -31, -28, -25, -22, -19, -16,
    -12, -9, -6, -3
};

static void
adxl_tone_update(struct adxl345 *ax, int32_t val)
{
    uint8_t flags = ax->tone_flags;
    if (!(flags & TONE_ACTIVE))
        return;
    if (flags & TONE_AWAIT) {
        if (timer_is_before(timer_read_time(), ax->tone_clock))
            return;
        ax->tone_flags = flags = TONE_ACTIVE;
    }
    // Quadrature correlation with the reference tone
    uint8_t ph = ax->tone_phase >> 8;
    ax->tone_phase += ax->tone_phase_inc;
    ax->tone_i += (val * adxl_sin_tab[(uint8_t)(ph + 64)]) >> 7;
    ax->tone_q += (val * adxl_sin_tab[ph]) >> 7;
    if (++ax->tone_count < (1 << ax->tone_block_shift))
        return;
    int32_t bi = ax->tone_i >> 4, bq = ax->tone_q >> 4;
    uint32_t amp2 = (uint32_t)(bi * bi) + (uint32_t)(bq * bq);
    ax->tone_i = ax->tone_q = 0;
    ax->tone_count = 0;
    if (ax->tone_warm < ax->tone_warm_target) {
        // Build the baseline before arming
        ax->tone_warm++;
        ax->tone_ema = ax->tone_warm == 1
            ? amp2 : ax->tone_ema - (ax->tone_ema >> 2) + (amp2 >> 2);
        return;
    }
    uint32_t ema = ax->tone_ema;
    // Slow baseline tracking
    ax->tone_ema = ema - (ema >> 3) + (amp2 >> 3);
    if (ema < ax->tone_min_amp) {
        ax->tone_consec = 0;
        return;
    }
    if ((uint64_t)amp2 * 256 < (uint64_t)ema * ax->tone_ratio8) {
        if (++ax->tone_consec >= ax->tone_consec_target) {
            trsync_do_trigger(ax->tone_ts, ax->tone_reason);
            ax->tone_flags = 0;
        }
    } else {
        ax->tone_consec = 0;
    }
}
#else
static void
adxl_tone_update(struct adxl345 *ax, int32_t val)
{
}
#endif

static struct task_wake adxl345_wake;

// Event handler that wakes adxl345_task() periodically
static uint_fast8_t
adxl345_event(struct timer *timer)
{
    struct adxl345 *ax = container_of(timer, struct adxl345, timer);
    ax->flags |= AX_PENDING;
    sched_wake_task(&adxl345_wake);
    return SF_DONE;
}

void
command_config_adxl345(uint32_t *args)
{
    struct adxl345 *ax = oid_alloc(args[0], command_config_adxl345
                                   , sizeof(*ax));
    ax->timer.func = adxl345_event;
    ax->spi = spidev_oid_lookup(args[1]);
}
DECL_COMMAND(command_config_adxl345, "config_adxl345 oid=%c spi_oid=%c");

// Helper code to reschedule the adxl345_event() timer
static void
adxl_reschedule_timer(struct adxl345 *ax)
{
    irq_disable();
    ax->timer.waketime = timer_read_time() + ax->rest_ticks;
    sched_add_timer(&ax->timer);
    irq_enable();
}

// Chip registers
#define AR_DATAX0      0x32
#define AR_FIFO_STATUS 0x39
#define AM_READ  0x80
#define AM_MULTI 0x40

#define SET_FIFO_CTL 0x90

#define BYTES_PER_SAMPLE 5

// Query accelerometer data
static void
adxl_query(struct adxl345 *ax, uint8_t oid)
{
    // Read data
    uint8_t msg[9] = { AR_DATAX0 | AM_READ | AM_MULTI, 0, 0, 0, 0, 0, 0, 0, 0 };
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    // Extract x, y, z measurements
    uint_fast8_t fifo_status = msg[8] & ~0x80; // Ignore trigger bit
    uint8_t *d = &ax->sb.data[ax->sb.data_count];
    if (((msg[2] & 0xf0) && (msg[2] & 0xf0) != 0xf0)
        || ((msg[4] & 0xf0) && (msg[4] & 0xf0) != 0xf0)
        || ((msg[6] & 0xf0) && (msg[6] & 0xf0) != 0xf0)
        || (msg[7] != SET_FIFO_CTL) || (fifo_status > 32)) {
        // Data error - may be a CS, MISO, MOSI, or SCLK glitch
        d[0] = d[1] = d[2] = d[3] = d[4] = 0xff;
        fifo_status = 0;
    } else {
        // Copy data
        d[0] = msg[1]; // x low bits
        d[1] = msg[3]; // y low bits
        d[2] = msg[5]; // z low bits
        d[3] = (msg[2] & 0x1f) | (msg[6] << 5); // x high bits and z high bits
        d[4] = (msg[4] & 0x1f) | ((msg[6] << 2) & 0x60); // y high and z high
        // Report selected axes for homing/probing trigger checks
#if CONFIG_WANT_ADXL345_PROBE
        for (uint_fast8_t slot = 0; slot < ADXL_TA_SLOTS; slot++) {
            if (!ax->ta[slot])
                continue;
            uint_fast8_t an = ax->ta_axis[slot] * 2;
            int32_t aval = (int16_t)((msg[an + 2] << 8) | msg[an + 1]);
            trigger_analog_update(ax->ta[slot], aval);
        }
        if (ax->tone_flags) {
            uint_fast8_t tn = ax->tone_axis * 2;
            int32_t tval = (int16_t)((msg[tn + 2] << 8) | msg[tn + 1]);
            adxl_tone_update(ax, tval);
        }
#endif
    }
    ax->sb.data_count += BYTES_PER_SAMPLE;
    if (ax->sb.data_count + BYTES_PER_SAMPLE > ARRAY_SIZE(ax->sb.data))
        sensor_bulk_report(&ax->sb, oid);
    // Check fifo status
    if (fifo_status >= 31)
        ax->sb.possible_overflows++;
    if (fifo_status > 1) {
        // More data in fifo - wake this task again
        sched_wake_task(&adxl345_wake);
    } else {
        // Sleep until next check time
        ax->flags &= ~AX_PENDING;
        adxl_reschedule_timer(ax);
    }
}

void
command_query_adxl345(uint32_t *args)
{
    struct adxl345 *ax = oid_lookup(args[0], command_config_adxl345);

    sched_del_timer(&ax->timer);
    ax->flags = 0;
    if (!args[1])
        // End measurements
        return;

    // Start new measurements query
    ax->rest_ticks = args[1];
    sensor_bulk_reset(&ax->sb);
    adxl_reschedule_timer(ax);
}
DECL_COMMAND(command_query_adxl345, "query_adxl345 oid=%c rest_ticks=%u");

void
adxl345_attach_trigger_analog(uint32_t *args)
{
    struct adxl345 *ax = oid_lookup(args[0], command_config_adxl345);
    uint8_t axis = args[2], slot = args[3];
    if (axis > 2)
        shutdown("adxl345 trigger axis out of range 0-2");
    if (slot >= ADXL_TA_SLOTS)
        shutdown("adxl345 trigger slot out of range");
    ax->ta[slot] = trigger_analog_oid_lookup(args[1]);
    ax->ta_axis[slot] = axis;
}
#if CONFIG_WANT_ADXL345_PROBE
DECL_COMMAND(adxl345_attach_trigger_analog, "adxl345_attach_trigger_analog"
             " oid=%c trigger_analog_oid=%c axis=%c slot=%c");
#endif

void
adxl345_tone_home(uint32_t *args)
{
    struct adxl345 *ax = oid_lookup(args[0], command_config_adxl345);
    ax->tone_flags = 0;
    ax->tone_ts = trsync_oid_lookup(args[1]);
    ax->tone_reason = args[2];
    uint8_t axis = args[3];
    if (axis > 2)
        shutdown("adxl345 tone axis out of range 0-2");
    ax->tone_axis = axis;
    ax->tone_clock = args[4];
    ax->tone_phase_inc = args[5];
    uint8_t bs = args[6];
    if (bs < 4 || bs > 8)
        shutdown("adxl345 tone block_shift out of range 4-8");
    ax->tone_block_shift = bs;
    ax->tone_ratio8 = args[7];
    ax->tone_warm_target = args[8];
    ax->tone_consec_target = args[9];
    ax->tone_min_amp = args[10];
    ax->tone_phase = 0;
    ax->tone_i = ax->tone_q = 0;
    ax->tone_count = 0;
    ax->tone_warm = 0;
    ax->tone_consec = 0;
    ax->tone_ema = 0;
    ax->tone_flags = TONE_ACTIVE | TONE_AWAIT;
}
#if CONFIG_WANT_ADXL345_PROBE
DECL_COMMAND(adxl345_tone_home, "adxl345_tone_home oid=%c trsync_oid=%c"
             " trigger_reason=%c axis=%c clock=%u phase_inc=%hu"
             " block_shift=%c ratio8=%c warmup=%c consec=%c min_amp=%u");
#endif

void
adxl345_tone_stop(uint32_t *args)
{
    struct adxl345 *ax = oid_lookup(args[0], command_config_adxl345);
    ax->tone_flags = 0;
}
#if CONFIG_WANT_ADXL345_PROBE
DECL_COMMAND(adxl345_tone_stop, "adxl345_tone_stop oid=%c");
#endif

void
command_query_adxl345_status(uint32_t *args)
{
    struct adxl345 *ax = oid_lookup(args[0], command_config_adxl345);
    uint8_t msg[2] = { AR_FIFO_STATUS | AM_READ, 0x00 };

    uint32_t time1 = timer_read_time();
    spidev_transfer(ax->spi, 1, sizeof(msg), msg);
    uint32_t time2 = timer_read_time();

    uint_fast8_t fifo_status = msg[1] & ~0x80; // Ignore trigger bit
    if (fifo_status > 32)
        // Query error - don't send response - host will retry
        return;
    sensor_bulk_status(&ax->sb, args[0], time1, time2-time1
                       , fifo_status * BYTES_PER_SAMPLE);
}
DECL_COMMAND(command_query_adxl345_status, "query_adxl345_status oid=%c");

void
adxl345_task(void)
{
    if (!sched_check_wake(&adxl345_wake))
        return;
    uint8_t oid;
    struct adxl345 *ax;
    foreach_oid(oid, ax, command_config_adxl345) {
        uint_fast8_t flags = ax->flags;
        if (flags & AX_PENDING)
            adxl_query(ax, oid);
    }
}
DECL_TASK(adxl345_task);
