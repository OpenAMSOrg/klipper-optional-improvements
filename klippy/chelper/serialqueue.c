// Serial port command queuing
//
// Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

// This goal of this code is to handle low-level serial port
// communications with a microcontroller (mcu).  This code is written
// in C (instead of python) to reduce communication latencies and to
// reduce scheduling jitter.  The code queues messages to be
// transmitted, schedules transmission of commands at specified mcu
// clock times, prioritizes commands, and handles retransmissions.  A
// background thread is launched to do this work and minimize latency.

#define _GNU_SOURCE // recvmmsg
#include <linux/can.h> // // struct can_frame
#include <math.h> // fabs
#include <pthread.h> // pthread_mutex_lock
#include <stddef.h> // offsetof
#include <stdint.h> // uint64_t
#include <stdio.h> // snprintf
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <sys/socket.h> // recvmmsg
#include <termios.h> // tcflush
#include <time.h> // clock_gettime
#include <unistd.h> // pipe
#include "compiler.h" // __visible
#include "list.h" // list_add_tail
#include "msgblock.h" // message_alloc
#include "pollreactor.h" // pollreactor_alloc
#include "pyhelper.h" // get_monotonic
#include "serialqueue.h" // struct queue_message

struct message_sub_queue {
    struct list_head msg_queue;
    struct list_node node;
};

struct command_queue {
    struct message_sub_queue ready, upcoming;
};

struct receiver {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int waiting;
    struct list_head queue;
    struct list_head old_receive;
};

struct transmit_requests {
    int pipe_fds[2];
    pthread_mutex_t lock; // protects variables below
    struct list_head upcoming_queues;
    int upcoming_bytes;
    uint64_t need_kick_clock, min_release_clock;
};

struct serialqueue {
    // Input reading
    struct pollreactor *pr;
    int serial_fd, serial_fd_type, client_id;
    uint8_t input_buf[4096];
    uint8_t need_sync;
    int input_pos;
    // Multi-threaded support for pushing and pulling messages
    struct receiver receiver;
    struct transmit_requests transmit_requests;
    // Threading (track lifetime so a naturally exited thread is still joined)
    char name[16];
    pthread_t tid;
    int thread_started, thread_joined;
    pthread_mutex_t lock; // protects variables below
    // Baud / clock tracking
    int receive_window;
    double bittime_adjust, idle_time;
    struct clock_estimate ce;
    double last_receive_sent_time;
    // Retransmit support
    uint64_t send_seq, receive_seq;
    uint64_t ignore_nak_seq, last_ack_seq, retransmit_seq, rtt_sample_seq;
    struct list_head sent_queue;
    double srtt, rttvar, rto;
    // Pending transmission message queues
    struct list_head ready_queues;
    int ready_bytes, need_ack_bytes, last_ack_bytes;
    struct list_head notify_queue;
    double last_write_fail_time;
    // Fastreader support
    pthread_mutex_t fast_reader_dispatch_lock;
    struct list_head fast_readers;
    // Debugging
    struct list_head old_sent;
    // Stats
    uint32_t bytes_write, bytes_read, bytes_retransmit, bytes_invalid;
};

#define SQPF_SERIAL 0
#define SQPF_PIPE   1
#define SQPF_NUM    2

#define SQPT_RETRANSMIT 0
#define SQPT_COMMAND    1
#define SQPT_NUM        2

#define SQT_UART 'u'
#define SQT_CAN 'c'
#define SQT_DEBUGFILE 'f'

#define MIN_RTO 0.025
#define MAX_RTO 5.000
#define MAX_PENDING_BLOCKS 12
#define MIN_REQTIME_DELTA 0.100
#define MIN_BACKGROUND_DELTA 0.005
#define IDLE_QUERY_TIME 1.0

#define DEBUG_QUEUE_SENT 100
#define DEBUG_QUEUE_RECEIVE 100

#define CAN_RX_BATCH 16

// Create a series of empty messages and add them to a list
static void
debug_queue_alloc(struct list_head *root, int count)
{
    int i;
    for (i=0; i<count; i++) {
        struct queue_message *qm = message_alloc();
        list_add_head(&qm->node, root);
    }
}

// Copy a message to a debug queue and free old debug messages
static struct queue_message *
_debug_queue_add(struct list_head *root, struct queue_message *qm)
{
    list_add_tail(&qm->node, root);
    struct queue_message *old = list_first_entry(
        root, struct queue_message, node);
    list_del(&old->node);
    return old;
}

static void
debug_queue_add(struct list_head *root, struct queue_message *qm)
{
    struct queue_message *old = _debug_queue_add(root, qm);
    message_free(old);
}

// Add messages and wake up the receiver thread if it is waiting
static void
receive_append_wake(struct receiver *receiver, struct list_head *msgs)
{
    int dokick = 0;
    pthread_mutex_lock(&receiver->lock);
    list_join_tail(msgs, &receiver->queue);
    if (receiver->waiting) {
        receiver->waiting = 0;
        dokick = 1;
    }
    pthread_mutex_unlock(&receiver->lock);
    if (dokick)
        pthread_cond_signal(&receiver->cond);
}

// Write to the internal pipe to wake the background thread if in poll
static void
kick_bg_thread(struct serialqueue *sq)
{
    int ret = write(sq->transmit_requests.pipe_fds[1], ".", 1);
    if (ret < 0)
        report_errno("pipe write", ret);
}

// Minimum number of bits in a canbus message
#define CANBUS_PACKET_BITS ((1 + 11 + 3 + 4) + (16 + 2 + 7 + 3))
#define CANBUS_IFS_BITS 4

// Determine minimum time needed to transmit a given number of bytes
static double
calculate_bittime(struct serialqueue *sq, uint32_t bytes)
{
    if (sq->serial_fd_type == SQT_CAN) {
        uint32_t pkts = DIV_ROUND_UP(bytes, 8);
        uint32_t bits = bytes * 8 + pkts * CANBUS_PACKET_BITS - CANBUS_IFS_BITS;
        return sq->bittime_adjust * bits;
    } else {
        return sq->bittime_adjust * bytes;
    }
}

// Update internal state when the receive sequence increases
static void
update_receive_seq(struct serialqueue *sq, double eventtime, uint64_t rseq)
{
    // Remove from sent queue
    uint64_t sent_seq = sq->receive_seq;
    for (;;) {
        struct queue_message *sent = list_first_entry(
            &sq->sent_queue, struct queue_message, node);
        if (list_empty(&sq->sent_queue)) {
            // Got an ack for a message not sent; must be connection init
            sq->send_seq = rseq;
            sq->last_receive_sent_time = 0.;
            break;
        }
        sq->need_ack_bytes -= sent->len;
        list_del(&sent->node);
        debug_queue_add(&sq->old_sent, sent);
        sent_seq++;
        if (rseq == sent_seq) {
            // Found sent message corresponding with the received sequence
            sq->last_receive_sent_time = sent->receive_time;
            sq->last_ack_bytes = sent->len;
            break;
        }
    }
    sq->receive_seq = rseq;
    pollreactor_update_timer(sq->pr, SQPT_COMMAND, PR_NOW);

    // Update retransmit info
    if (sq->rtt_sample_seq && rseq > sq->rtt_sample_seq
        && sq->last_receive_sent_time) {
        // RFC6298 rtt calculations
        double delta = eventtime - sq->last_receive_sent_time;
        if (!sq->srtt) {
            sq->rttvar = delta / 2.0;
            sq->srtt = delta * 10.0; // use a higher start default
        } else {
            sq->rttvar = (3.0 * sq->rttvar + fabs(sq->srtt - delta)) / 4.0;
            sq->srtt = (7.0 * sq->srtt + delta) / 8.0;
        }
        double rttvar4 = sq->rttvar * 4.0;
        if (rttvar4 < 0.001)
            rttvar4 = 0.001;
        sq->rto = sq->srtt + rttvar4;
        if (sq->rto < MIN_RTO)
            sq->rto = MIN_RTO;
        else if (sq->rto > MAX_RTO)
            sq->rto = MAX_RTO;
        sq->rtt_sample_seq = 0;
    }
    if (list_empty(&sq->sent_queue)) {
        pollreactor_update_timer(sq->pr, SQPT_RETRANSMIT, PR_NEVER);
    } else {
        struct queue_message *sent = list_first_entry(
            &sq->sent_queue, struct queue_message, node);
        double nr = eventtime + sq->rto + calculate_bittime(sq, sent->len);
        pollreactor_update_timer(sq->pr, SQPT_RETRANSMIT, nr);
    }
}

// Process a well formed input message
static void
handle_message(struct serialqueue *sq, double eventtime, double receive_time
               , const uint8_t *msg, int len)
{
    pthread_mutex_lock(&sq->lock);

    // Calculate receive sequence number
    uint32_t rseq_delta = ((msg[MESSAGE_POS_SEQ] - sq->receive_seq)
                           & MESSAGE_SEQ_MASK);
    uint64_t rseq = sq->receive_seq + rseq_delta;
    if (rseq != sq->receive_seq) {
        // New sequence number
        if (rseq > sq->send_seq && sq->receive_seq != 1) {
            // An ack for a message not sent?  Out of order message?
            sq->bytes_invalid += len;
            pthread_mutex_unlock(&sq->lock);
            return;
        }
        update_receive_seq(sq, eventtime, rseq);
    }
    sq->bytes_read += len;

    // Check for pending messages on notify_queue
    struct list_head received;
    list_init(&received);
    while (!list_empty(&sq->notify_queue)) {
        struct queue_message *qm = list_first_entry(
            &sq->notify_queue, struct queue_message, node);
        uint64_t wake_seq = rseq - 1 - (len > MESSAGE_MIN ? 1 : 0);
        uint64_t notify_msg_sent_seq = qm->req_clock;
        if (notify_msg_sent_seq > wake_seq)
            break;
        list_del(&qm->node);
        qm->len = 0;
        qm->sent_time = sq->last_receive_sent_time;
        qm->receive_time = eventtime;
        list_add_tail(&qm->node, &received);
    }

    // Process message
    if (len == MESSAGE_MIN) {
        // Ack/nak message
        if (sq->last_ack_seq < rseq)
            sq->last_ack_seq = rseq;
        else if (rseq > sq->ignore_nak_seq && !list_empty(&sq->sent_queue))
            // Duplicate Ack is a Nak - do fast retransmit
            pollreactor_update_timer(sq->pr, SQPT_RETRANSMIT, PR_NOW);
    } else {
        // Data message - add to receive queue
        struct queue_message *qm = message_fill(msg, len);
        qm->sent_time = (rseq > sq->retransmit_seq
                         ? sq->last_receive_sent_time : 0.);
        qm->receive_time = receive_time - calculate_bittime(sq, len);
        list_add_tail(&qm->node, &received);
    }

    if (!list_empty(&received))
        receive_append_wake(&sq->receiver, &received);

    // Check fast readers
    struct fastreader *fr;
    list_for_each_entry(fr, &sq->fast_readers, node) {
        if (len < fr->prefix_len + MESSAGE_MIN
            || memcmp(&msg[MESSAGE_HEADER_SIZE]
                      , fr->prefix, fr->prefix_len) != 0)
            continue;
        // Release main lock and invoke callback
        pthread_mutex_lock(&sq->fast_reader_dispatch_lock);
        pthread_mutex_unlock(&sq->lock);
        fr->func(fr, eventtime, msg, len);
        pthread_mutex_unlock(&sq->fast_reader_dispatch_lock);
        return;
    }
    pthread_mutex_unlock(&sq->lock);
}

// Process complete frames and retain any incomplete trailing data
static void
process_input(struct serialqueue *sq, double eventtime, double receive_time)
{
    // Walk each complete frame in place and compact the buffer only once
    int consumed = 0, invalid = 0;
    for (;;) {
        const uint8_t *msg = &sq->input_buf[consumed];
        int available = sq->input_pos - consumed;
        int msg_len = msgblock_check(&sq->need_sync, msg, available);
        if (!msg_len)
            break;
        if (msg_len > 0) {
            // Timestamp after read() when the transport supplied no timestamp
            if (msg_len > MESSAGE_MIN && !receive_time)
                receive_time = get_monotonic();
            handle_message(sq, eventtime, receive_time, msg, msg_len);
        } else {
            // Skip malformed data at the beginning of the input
            msg_len = -msg_len;
            invalid += msg_len;
        }
        consumed += msg_len;
    }
    if (invalid) {
        pthread_mutex_lock(&sq->lock);
        sq->bytes_invalid += invalid;
        pthread_mutex_unlock(&sq->lock);
    }
    sq->input_pos -= consumed;
    if (consumed && sq->input_pos)
        memmove(sq->input_buf, &sq->input_buf[consumed], sq->input_pos);
}

// Find the kernel receive time in a SocketCAN message
static double
can_get_receive_time(struct msghdr *msg, double realtime_to_monotonic
                     , double fallback_time, int have_clock_offset)
{
    if (!have_clock_offset || (msg->msg_flags & MSG_CTRUNC))
        return fallback_time;
    struct cmsghdr *cmsg;
    for (cmsg = CMSG_FIRSTHDR(msg); cmsg; cmsg = CMSG_NXTHDR(msg, cmsg)) {
        if (cmsg->cmsg_level != SOL_SOCKET || cmsg->cmsg_type != SCM_TIMESTAMPNS
            || cmsg->cmsg_len < CMSG_LEN(sizeof(struct timespec)))
            continue;
        struct timespec ts;
        memcpy(&ts, CMSG_DATA(cmsg), sizeof(ts));
        if (ts.tv_nsec < 0 || ts.tv_nsec >= 1000000000)
            return fallback_time;
        double realtime = (double)ts.tv_sec + (double)ts.tv_nsec * .000000001;
        return realtime + realtime_to_monotonic;
    }
    return fallback_time;
}

// Read a bounded batch of complete SocketCAN records
static void
can_input_event(struct serialqueue *sq, double eventtime)
{
    struct can_frame frames[CAN_RX_BATCH];
    struct iovec iov[CAN_RX_BATCH];
    struct mmsghdr msgs[CAN_RX_BATCH];
    union {
        struct cmsghdr align;
        char data[CMSG_SPACE(sizeof(struct timespec))];
    } control[CAN_RX_BATCH];
    memset(msgs, 0, sizeof(msgs));
    int i;
    for (i=0; i<CAN_RX_BATCH; i++) {
        iov[i].iov_base = &frames[i];
        iov[i].iov_len = sizeof(frames[i]);
        msgs[i].msg_hdr.msg_iov = &iov[i];
        msgs[i].msg_hdr.msg_iovlen = 1;
        msgs[i].msg_hdr.msg_control = control[i].data;
        msgs[i].msg_hdr.msg_controllen = sizeof(control[i].data);
    }

    /*
     * A raw CAN socket is record-oriented, but the old input path consumed only
     * one record for each poll callback.  A burst therefore required another
     * poll-reactor pass, timer scan, and read syscall per CAN frame even though
     * the socket already held the complete batch.  recvmmsg() preserves record
     * boundaries while amortizing those transitions.  Bound the batch so a
     * busy CAN interface can not starve command scheduling in this thread.
     */
    int count = recvmmsg(sq->serial_fd, msgs, ARRAY_SIZE(msgs), MSG_DONTWAIT,
                         NULL);
    if (count < 0) {
        report_errno("can read", count);
        pollreactor_do_exit(sq->pr);
        return;
    }

    /*
     * python-can enables nanosecond software timestamps before handing this
     * socket to serialqueue.  Those timestamps use CLOCK_REALTIME, whereas all
     * Klipper scheduling uses CLOCK_MONOTONIC_RAW.  Sample both clocks once for
     * this batch and translate each kernel timestamp into the scheduler's time
     * domain.  Falling back to the post-read monotonic time retains the prior
     * behavior if an older SocketCAN implementation supplies no control data.
     */
    struct timespec realtime_now;
    int have_clock_offset = !clock_gettime(CLOCK_REALTIME, &realtime_now);
    double fallback_time = get_monotonic(), realtime_to_monotonic = 0.;
    if (have_clock_offset) {
        double realtime = ((double)realtime_now.tv_sec
                           + (double)realtime_now.tv_nsec * .000000001);
        realtime_to_monotonic = fallback_time - realtime;
    }

    for (i=0; i<count; i++) {
        if (msgs[i].msg_len != sizeof(frames[i])
            || (msgs[i].msg_hdr.msg_flags & MSG_TRUNC)) {
            errorf("Got incomplete CAN frame (size=%u)", msgs[i].msg_len);
            pollreactor_do_exit(sq->pr);
            return;
        }
        struct can_frame *cf = &frames[i];
        if (cf->can_dlc > CAN_MAX_DLEN
            || cf->can_dlc > sizeof(sq->input_buf) - sq->input_pos) {
            errorf("Got invalid CAN frame length: %d", cf->can_dlc);
            pollreactor_do_exit(sq->pr);
            return;
        }
        if (cf->can_id != sq->client_id + 1)
            continue;
        memcpy(&sq->input_buf[sq->input_pos], cf->data, cf->can_dlc);
        sq->input_pos += cf->can_dlc;
        double receive_time = can_get_receive_time(
            &msgs[i].msg_hdr, realtime_to_monotonic, fallback_time,
            have_clock_offset);
        // Process now so a completed block retains its final frame's timestamp
        process_input(sq, eventtime, receive_time);
    }
}

// Callback for input activity on the serial fd
static void
input_event(struct serialqueue *sq, double eventtime)
{
    if (sq->serial_fd_type == SQT_CAN) {
        can_input_event(sq, eventtime);
        return;
    } else {
        int ret = read(sq->serial_fd, &sq->input_buf[sq->input_pos]
                       , sizeof(sq->input_buf) - sq->input_pos);
        if (ret <= 0) {
            if(ret < 0)
                report_errno("read", ret);
            else
                errorf("Got EOF when reading from device");
            pollreactor_do_exit(sq->pr);
            return;
        }
        sq->input_pos += ret;
    }
    process_input(sq, eventtime, 0.);
}

// Callback for input activity on the pipe fd (wakes command_event)
static void
kick_event(struct serialqueue *sq, double eventtime)
{
    char dummy[4096];
    int ret = read(sq->transmit_requests.pipe_fds[0], dummy, sizeof(dummy));
    if (ret < 0)
        report_errno("pipe read", ret);
    pollreactor_update_timer(sq->pr, SQPT_COMMAND, PR_NOW);
}

// OS write of data to be sent to the mcu
static void
do_write(struct serialqueue *sq, void *buf, int buflen)
{
    if (sq->serial_fd_type != SQT_CAN) {
        int ret = write(sq->serial_fd, buf, buflen);
        if (ret < 0)
            report_errno("write", ret);
        return;
    }
    // Write to CAN fd
    struct can_frame cf;
    // Clear kernel-visible padding fields
    memset(&cf, 0, sizeof(cf));
    cf.can_id = sq->client_id;
    uint8_t *data = buf;
    while (buflen) {
        int size = buflen > 8 ? 8 : buflen;
        cf.can_dlc = size;
        memcpy(cf.data, data, size);
        int ret = write(sq->serial_fd, &cf, sizeof(cf));
        /*
         * A SocketCAN raw socket is record-oriented: success is exactly one
         * complete struct can_frame, not a byte count that may be resumed with
         * another write.  Advancing after a short positive result would drop
         * the unsent suffix of this Klipper stream fragment, while writing that
         * suffix separately would create a second and semantically different
         * CAN frame.  The protocol retransmit machinery can recover after the
         * connection is stopped, but it can not infer this silent boundary
         * loss, so treat any short result as a fatal transport invariant
         * violation.  Negative results retain the existing timed retry path.
         */
        if (ret >= 0 && ret != sizeof(cf)) {
            errorf("Incomplete CAN frame write (size=%d)", ret);
            pollreactor_do_exit(sq->pr);
            return;
        }
        if (ret < 0) {
            report_errno("can write", ret);
            double curtime = get_monotonic();
            if (!sq->last_write_fail_time) {
                sq->last_write_fail_time = curtime;
            } else if (curtime > sq->last_write_fail_time + 10.0) {
                errorf("Halting reads due to CAN write errors.");
                pollreactor_do_exit(sq->pr);
            }
            return;
        }
        sq->last_write_fail_time = 0.0;
        data += size;
        buflen -= size;
    }
}

// Callback timer for when a retransmit should be done
static double
retransmit_event(struct serialqueue *sq, double eventtime)
{
    if (sq->serial_fd_type == SQT_UART) {
        int ret = tcflush(sq->serial_fd, TCOFLUSH);
        if (ret < 0)
            report_errno("tcflush", ret);
    }

    pthread_mutex_lock(&sq->lock);

    // Retransmit all pending messages
    uint8_t buf[MESSAGE_MAX * MAX_PENDING_BLOCKS + 1];
    int buflen = 0, first_buflen = 0;
    buf[buflen++] = MESSAGE_SYNC;
    struct queue_message *qm;
    list_for_each_entry(qm, &sq->sent_queue, node) {
        memcpy(&buf[buflen], qm->msg, qm->len);
        buflen += qm->len;
        if (!first_buflen)
            first_buflen = qm->len + 1;
    }
    do_write(sq, buf, buflen);
    sq->bytes_retransmit += buflen;

    // Update rto
    if (pollreactor_get_timer(sq->pr, SQPT_RETRANSMIT) == PR_NOW) {
        // Retransmit due to nak
        sq->ignore_nak_seq = sq->receive_seq;
        if (sq->receive_seq < sq->retransmit_seq)
            // Second nak for this retransmit - don't allow third
            sq->ignore_nak_seq = sq->retransmit_seq;
    } else {
        // Retransmit due to timeout
        sq->rto *= 2.0;
        if (sq->rto > MAX_RTO)
            sq->rto = MAX_RTO;
        sq->ignore_nak_seq = sq->send_seq;
    }
    sq->retransmit_seq = sq->send_seq;
    sq->rtt_sample_seq = 0;
    sq->idle_time = eventtime + calculate_bittime(sq, buflen);
    double waketime = eventtime + sq->rto + calculate_bittime(sq, first_buflen);

    pthread_mutex_unlock(&sq->lock);
    return waketime;
}

// Construct a block of data to be sent to the serial port
static int
build_and_send_command(struct serialqueue *sq, uint8_t *buf, int pending
                       , double eventtime)
{
    int len = MESSAGE_HEADER_SIZE;
    while (sq->ready_bytes) {
        // Find highest priority message (message with lowest req_clock)
        uint64_t min_clock = MAX_CLOCK;
        struct command_queue *q, *cq = NULL;
        struct queue_message *qm = NULL;
        list_for_each_entry(q, &sq->ready_queues, ready.node) {
            struct queue_message *m = list_first_entry(
                &q->ready.msg_queue, struct queue_message, node);
            if (m->req_clock < min_clock) {
                min_clock = m->req_clock;
                cq = q;
                qm = m;
            }
        }
        // Append message to outgoing command
        if (len + qm->len > MESSAGE_MAX - MESSAGE_TRAILER_SIZE)
            break;
        list_del(&qm->node);
        if (list_empty(&cq->ready.msg_queue))
            list_del(&cq->ready.node);
        memcpy(&buf[len], qm->msg, qm->len);
        len += qm->len;
        sq->ready_bytes -= qm->len;
        if (qm->notify_id) {
            // Message requires notification - add to notify list
            qm->req_clock = sq->send_seq;
            list_add_tail(&qm->node, &sq->notify_queue);
        } else {
            message_free(qm);
        }
    }

    // Fill header / trailer
    len += MESSAGE_TRAILER_SIZE;
    msgblock_finalize(buf, len,
                      MESSAGE_DEST | (sq->send_seq & MESSAGE_SEQ_MASK));

    // Store message block
    double idletime = eventtime > sq->idle_time ? eventtime : sq->idle_time;
    idletime += calculate_bittime(sq, pending + len);
    struct queue_message *out = message_alloc();
    memcpy(out->msg, buf, len);
    out->len = len;
    out->sent_time = eventtime;
    out->receive_time = idletime;
    if (list_empty(&sq->sent_queue))
        pollreactor_update_timer(sq->pr, SQPT_RETRANSMIT, idletime + sq->rto);
    if (!sq->rtt_sample_seq)
        sq->rtt_sample_seq = sq->send_seq;
    sq->send_seq++;
    sq->need_ack_bytes += len;
    list_add_tail(&out->node, &sq->sent_queue);
    return len;
}

// Move messages from upcoming queues to ready queues
static uint64_t
check_upcoming_queues(struct serialqueue *sq, uint64_t ack_clock)
{
    pthread_mutex_lock(&sq->transmit_requests.lock);
    sq->transmit_requests.need_kick_clock = 0;
    uint64_t min_release_clock = sq->transmit_requests.min_release_clock;
    if (ack_clock < min_release_clock) {
        pthread_mutex_unlock(&sq->transmit_requests.lock);
        return min_release_clock;
    }

    uint64_t min_stalled_clock = MAX_CLOCK;
    struct command_queue *cq, *_ncq;
    list_for_each_entry_safe(cq, _ncq, &sq->transmit_requests.upcoming_queues,
                             upcoming.node) {
        int not_in_ready_queues = list_empty(&cq->ready.msg_queue);
        // Move messages from the upcoming.msg_queue to the ready.msg_queue
        struct queue_message *qm, *_nqm;
        list_for_each_entry_safe(qm, _nqm, &cq->upcoming.msg_queue, node) {
            if (ack_clock < qm->min_clock) {
                if (qm->min_clock < min_stalled_clock)
                    min_stalled_clock = qm->min_clock;
                break;
            }
            list_del(&qm->node);
            list_add_tail(&qm->node, &cq->ready.msg_queue);
            sq->transmit_requests.upcoming_bytes -= qm->len;
            sq->ready_bytes += qm->len;
        }
        // Remove cq from the list if it is now empty
        if (list_empty(&cq->upcoming.msg_queue))
            list_del(&cq->upcoming.node);
        // Add to ready queues
        if (not_in_ready_queues && !list_empty(&cq->ready.msg_queue))
            list_add_tail(&cq->ready.node, &sq->ready_queues);
    }
    sq->transmit_requests.min_release_clock = min_stalled_clock;
    pthread_mutex_unlock(&sq->transmit_requests.lock);
    return min_stalled_clock;
}

// Set the next transmit queue need_kick_clock
static int
update_need_kick_clock(struct serialqueue *sq, uint64_t wantclock)
{
    pthread_mutex_lock(&sq->transmit_requests.lock);
    if (wantclock > sq->transmit_requests.min_release_clock) {
        pthread_mutex_unlock(&sq->transmit_requests.lock);
        return -1;
    }
    sq->transmit_requests.need_kick_clock = wantclock;
    pthread_mutex_unlock(&sq->transmit_requests.lock);
    return 0;
}

// Determine if ready to send commands (or the amount of time to sleep if not)
static double
check_send_command(struct serialqueue *sq, int pending, double eventtime)
{
    // Check for upcoming messages now ready
    double idletime = eventtime > sq->idle_time ? eventtime : sq->idle_time;
    idletime += calculate_bittime(sq, pending + MESSAGE_MIN);
    uint64_t ack_clock = clock_from_time(&sq->ce, idletime);
    uint64_t min_stalled_clock = check_upcoming_queues(sq, ack_clock);

    // Check if valid to send messages
    if (sq->send_seq - sq->receive_seq >= MAX_PENDING_BLOCKS
        && sq->receive_seq != (uint64_t)-1)
        // Need an ack before more messages can be sent
        return eventtime + 0.250;
    if (sq->send_seq > sq->receive_seq && sq->receive_window) {
        int need_ack_bytes = sq->need_ack_bytes + MESSAGE_MAX;
        if (sq->last_ack_seq < sq->receive_seq)
            need_ack_bytes += sq->last_ack_bytes;
        if (need_ack_bytes > sq->receive_window)
            // Wait for ack from past messages before sending next message
            return eventtime + 0.250;
    }

    // Check if a block is fully ready to send
    if (sq->ready_bytes >= MESSAGE_PAYLOAD_MAX)
        return PR_NOW;
    if (! sq->ce.est_freq) {
        // Clock unknown during initial startup - recheck on each add
        if (sq->ready_bytes)
            return PR_NOW;
        int mustwake = update_need_kick_clock(sq, 1);
        if (mustwake)
            return eventtime;
        return PR_NEVER;
    }

    // Check if it is still needed to send messages from the ready_queues
    uint64_t min_ready_clock = MAX_CLOCK;
    struct command_queue *cq;
    list_for_each_entry(cq, &sq->ready_queues, ready.node) {
        // Update min_ready_clock
        struct queue_message *qm = list_first_entry(
            &cq->ready.msg_queue, struct queue_message, node);
        uint64_t req_clock = qm->req_clock;
        double bgtime = pending ? idletime : sq->idle_time;
        double bgoffset = MIN_REQTIME_DELTA + MIN_BACKGROUND_DELTA;
        if (req_clock == BACKGROUND_PRIORITY_CLOCK)
            req_clock = clock_from_time(&sq->ce, bgtime + bgoffset);
        if (req_clock < min_ready_clock)
            min_ready_clock = req_clock;
    }
    uint64_t reqclock_delta = MIN_REQTIME_DELTA * sq->ce.est_freq;
    if (min_ready_clock <= ack_clock + reqclock_delta)
        return PR_NOW;

    // Determine next wakeup time
    if (pending)
        // Caller wont sleep anyway - just return
        return eventtime;
    uint64_t wantclock = min_ready_clock - reqclock_delta;
    if (min_stalled_clock < wantclock)
        wantclock = min_stalled_clock;
    int mustwake = update_need_kick_clock(sq, wantclock);
    if (mustwake)
        // Raced with add of new command - avoid sleeping
        return eventtime;
    return idletime + (wantclock - ack_clock) / sq->ce.est_freq;
}

// Callback timer to send data to the serial port
static double
command_event(struct serialqueue *sq, double eventtime)
{
    pthread_mutex_lock(&sq->lock);
    uint8_t buf[MESSAGE_MAX * MAX_PENDING_BLOCKS];
    int buflen = 0;
    double waketime;
    for (;;) {
        waketime = check_send_command(sq, buflen, eventtime);
        if (waketime != PR_NOW)
            break;
        buflen += build_and_send_command(sq, &buf[buflen], buflen, eventtime);
        if (buflen + MESSAGE_MAX > sizeof(buf))
            break;
    }
    if (buflen) {
        // Write message blocks
        do_write(sq, buf, buflen);
        sq->bytes_write += buflen;
        double idletime = eventtime > sq->idle_time ? eventtime : sq->idle_time;
        sq->idle_time = idletime + calculate_bittime(sq, buflen);
        waketime = PR_NOW;
    }
    pthread_mutex_unlock(&sq->lock);
    return waketime;
}

// Main background thread for reading/writing to serial port
static void *
background_thread(void *data)
{
    struct serialqueue *sq = data;
    set_thread_name(sq->name);
    pollreactor_run(sq->pr);

    // Wake any waiting receivers
    struct list_head dummy;
    list_init(&dummy);
    receive_append_wake(&sq->receiver, &dummy);

    return NULL;
}

// Create a new 'struct serialqueue' object
struct serialqueue * __visible
serialqueue_alloc(int serial_fd, char serial_fd_type, int client_id
                  , char name[16])
{
    struct serialqueue *sq = malloc(sizeof(*sq));
    memset(sq, 0, sizeof(*sq));
    sq->serial_fd = serial_fd;
    sq->serial_fd_type = serial_fd_type;
    sq->client_id = client_id;
    strncpy(sq->name, name, sizeof(sq->name));
    sq->name[sizeof(sq->name)-1] = '\0';

    int ret = pipe(sq->transmit_requests.pipe_fds);
    if (ret)
        goto fail;

    // Reactor setup
    sq->pr = pollreactor_alloc(SQPF_NUM, SQPT_NUM, sq);
    pollreactor_add_fd(sq->pr, SQPF_SERIAL, serial_fd, input_event
                       , serial_fd_type==SQT_DEBUGFILE);
    pollreactor_add_fd(sq->pr, SQPF_PIPE, sq->transmit_requests.pipe_fds[0]
                       , kick_event, 0);
    pollreactor_add_timer(sq->pr, SQPT_RETRANSMIT, retransmit_event);
    pollreactor_add_timer(sq->pr, SQPT_COMMAND, command_event);
    fd_set_non_blocking(serial_fd);
    fd_set_non_blocking(sq->transmit_requests.pipe_fds[0]);
    fd_set_non_blocking(sq->transmit_requests.pipe_fds[1]);

    // Retransmit setup
    sq->send_seq = 1;
    if (serial_fd_type == SQT_DEBUGFILE) {
        // Debug file output
        sq->receive_seq = -1;
        sq->rto = PR_NEVER;
    } else {
        sq->receive_seq = 1;
        sq->rto = MIN_RTO;
    }

    // Queues
    sq->transmit_requests.need_kick_clock = MAX_CLOCK;
    sq->transmit_requests.min_release_clock = MAX_CLOCK;
    list_init(&sq->transmit_requests.upcoming_queues);
    pthread_mutex_init(&sq->transmit_requests.lock, NULL);
    list_init(&sq->ready_queues);
    list_init(&sq->sent_queue);
    list_init(&sq->receiver.queue);
    list_init(&sq->notify_queue);
    list_init(&sq->fast_readers);

    // Debugging
    list_init(&sq->old_sent);
    list_init(&sq->receiver.old_receive);
    debug_queue_alloc(&sq->old_sent, DEBUG_QUEUE_SENT);
    debug_queue_alloc(&sq->receiver.old_receive, DEBUG_QUEUE_RECEIVE);

    // Thread setup
    ret = pthread_mutex_init(&sq->lock, NULL);
    if (ret)
        goto fail;
    ret = pthread_mutex_init(&sq->receiver.lock, NULL);
    if (ret)
        goto fail;
    ret = pthread_cond_init(&sq->receiver.cond, NULL);
    if (ret)
        goto fail;
    ret = pthread_mutex_init(&sq->fast_reader_dispatch_lock, NULL);
    if (ret)
        goto fail;
    ret = pthread_create(&sq->tid, NULL, background_thread, sq);
    if (ret)
        goto fail;
    sq->thread_started = 1;

    return sq;

fail:
    report_errno("init", ret);
    return NULL;
}

// Request that the background thread exit
void __visible
serialqueue_exit(struct serialqueue *sq)
{
    if (!sq || !sq->thread_started || sq->thread_joined)
        return;
    pollreactor_do_exit(sq->pr);
    kick_bg_thread(sq);
    int ret = pthread_join(sq->tid, NULL);
    if (ret)
        report_errno("pthread_join", ret);
    else
        sq->thread_joined = 1;
}

// Free all resources associated with a serialqueue
void __visible
serialqueue_free(struct serialqueue *sq)
{
    if (!sq)
        return;
    if (!sq->thread_joined)
        serialqueue_exit(sq);
    pthread_mutex_lock(&sq->lock);
    message_queue_free(&sq->sent_queue);
    pthread_mutex_lock(&sq->receiver.lock);
    message_queue_free(&sq->receiver.queue);
    message_queue_free(&sq->receiver.old_receive);
    pthread_mutex_unlock(&sq->receiver.lock);
    message_queue_free(&sq->notify_queue);
    message_queue_free(&sq->old_sent);
    while (!list_empty(&sq->ready_queues)) {
        struct command_queue* cq = list_first_entry(
            &sq->ready_queues, struct command_queue, ready.node);
        list_del(&cq->ready.node);
        message_queue_free(&cq->ready.msg_queue);
    }
    pthread_mutex_lock(&sq->transmit_requests.lock);
    while (!list_empty(&sq->transmit_requests.upcoming_queues)) {
        struct command_queue *cq = list_first_entry(
            &sq->transmit_requests.upcoming_queues,
            struct command_queue, upcoming.node);
        list_del(&cq->upcoming.node);
        message_queue_free(&cq->upcoming.msg_queue);
    }
    pthread_mutex_unlock(&sq->transmit_requests.lock);
    pthread_mutex_unlock(&sq->lock);
    pollreactor_free(sq->pr);
    close(sq->transmit_requests.pipe_fds[0]);
    close(sq->transmit_requests.pipe_fds[1]);
    free(sq);
}

// Allocate a 'struct command_queue'
struct command_queue * __visible
serialqueue_alloc_commandqueue(void)
{
    struct command_queue *cq = malloc(sizeof(*cq));
    memset(cq, 0, sizeof(*cq));
    list_init(&cq->ready.msg_queue);
    list_init(&cq->upcoming.msg_queue);
    return cq;
}

// Free a 'struct command_queue'
void __visible
serialqueue_free_commandqueue(struct command_queue *cq)
{
    if (!cq)
        return;
    if (!list_empty(&cq->ready.msg_queue) ||
        !list_empty(&cq->upcoming.msg_queue)) {
        errorf("Memory leak! Can't free non-empty commandqueue");
        return;
    }
    free(cq);
}

// Add a low-latency message handler
void
serialqueue_add_fastreader(struct serialqueue *sq, struct fastreader *fr)
{
    pthread_mutex_lock(&sq->lock);
    list_add_tail(&fr->node, &sq->fast_readers);
    pthread_mutex_unlock(&sq->lock);
}

// Remove a previously registered low-latency message handler
void
serialqueue_rm_fastreader(struct serialqueue *sq, struct fastreader *fr)
{
    pthread_mutex_lock(&sq->lock);
    list_del(&fr->node);
    pthread_mutex_unlock(&sq->lock);

    pthread_mutex_lock(&sq->fast_reader_dispatch_lock); // XXX - goofy locking
    pthread_mutex_unlock(&sq->fast_reader_dispatch_lock);
}

// Add a batch of messages to the given command_queue
void
serialqueue_send_batch(struct serialqueue *sq, struct command_queue *cq
                       , struct list_head *msgs)
{
    // Make sure min_clock is set in list and calculate total bytes
    int len = 0;
    struct queue_message *qm;
    list_for_each_entry(qm, msgs, node) {
        if (qm->min_clock + (3LL<<29) < qm->req_clock
            && qm->req_clock != BACKGROUND_PRIORITY_CLOCK)
            // Avoid mcu clock comparison 31-bit overflow issues
            qm->min_clock = qm->req_clock - (3LL<<29);
        len += qm->len;
    }
    if (! len)
        return;
    qm = list_first_entry(msgs, struct queue_message, node);
    uint64_t min_clock = qm->min_clock;

    // Add list to cq->upcoming_queue
    int mustwake = 0;
    pthread_mutex_lock(&sq->transmit_requests.lock);
    if (list_empty(&cq->upcoming.msg_queue)) {
        list_add_tail(&cq->upcoming.node,
            &sq->transmit_requests.upcoming_queues);
        if (min_clock < sq->transmit_requests.min_release_clock)
            sq->transmit_requests.min_release_clock = min_clock;
        if (min_clock < sq->transmit_requests.need_kick_clock) {
            sq->transmit_requests.need_kick_clock = 0;
            mustwake = 1;
        }
    }
    list_join_tail(msgs, &cq->upcoming.msg_queue);
    sq->transmit_requests.upcoming_bytes += len;
    pthread_mutex_unlock(&sq->transmit_requests.lock);

    // Wake the background thread if necessary
    if (mustwake)
        kick_bg_thread(sq);
}

// Helper to send a single message
void
serialqueue_send_one(struct serialqueue *sq, struct command_queue *cq
                     , struct queue_message *qm)
{
    struct list_head msgs;
    list_init(&msgs);
    list_add_tail(&qm->node, &msgs);
    serialqueue_send_batch(sq, cq, &msgs);
}

// Schedule the transmission of a message on the serial port at a
// given time and priority.
void __visible
serialqueue_send(struct serialqueue *sq, struct command_queue *cq, uint8_t *msg
                 , int len, uint64_t min_clock, uint64_t req_clock
                 , uint64_t notify_id)
{
    if (len < 0 || len > MESSAGE_PAYLOAD_MAX) {
        errorf("Invalid serial message length: %d", len);
        return;
    }
    struct queue_message *qm = message_fill(msg, len);
    if (!qm)
        return;
    qm->min_clock = min_clock;
    qm->req_clock = req_clock;
    qm->notify_id = notify_id;
    serialqueue_send_one(sq, cq, qm);
}

// Return a message read from the serial port (or wait for one if none
// available)
void __visible
serialqueue_pull(struct serialqueue *sq, struct pull_queue_message *pqm)
{
    struct receiver *receiver = &sq->receiver;
    pthread_mutex_lock(&receiver->lock);
    // Wait for message to be available
    while (list_empty(&receiver->queue)) {
        if (pollreactor_is_exit(sq->pr))
            goto exit;
        receiver->waiting = 1;
        int ret = pthread_cond_wait(&receiver->cond, &receiver->lock);
        if (ret)
            report_errno("pthread_cond_wait", ret);
    }

    // Remove message from queue
    struct queue_message *qm = list_first_entry(
        &receiver->queue, struct queue_message, node);
    list_del(&qm->node);

    // Copy message
    memcpy(pqm->msg, qm->msg, qm->len);
    pqm->len = qm->len;
    pqm->sent_time = qm->sent_time;
    pqm->receive_time = qm->receive_time;
    pqm->notify_id = qm->notify_id;
    if (qm->len)
        qm = _debug_queue_add(&receiver->old_receive, qm);
    pthread_mutex_unlock(&receiver->lock);
    message_free(qm);
    return;

exit:
    pqm->len = -1;
    pthread_mutex_unlock(&receiver->lock);
}

void __visible
serialqueue_set_wire_frequency(struct serialqueue *sq, double frequency)
{
    pthread_mutex_lock(&sq->lock);
    if (sq->serial_fd_type == SQT_CAN) {
        sq->bittime_adjust = 1. / frequency;
    } else {
        // An 8N1 serial line is 10 bits per byte (1 start, 8 data, 1 stop)
        sq->bittime_adjust = 10. / frequency;
    }
    pthread_mutex_unlock(&sq->lock);
}

void __visible
serialqueue_set_receive_window(struct serialqueue *sq, int receive_window)
{
    pthread_mutex_lock(&sq->lock);
    sq->receive_window = receive_window;
    pthread_mutex_unlock(&sq->lock);
}

// Set the estimated clock rate of the mcu on the other end of the
// serial port
void __visible
serialqueue_set_clock_est(struct serialqueue *sq, double est_freq
                          , double conv_time, uint64_t conv_clock)
{
    pthread_mutex_lock(&sq->lock);
    clock_fill(&sq->ce, est_freq, conv_time, conv_clock);
    pthread_mutex_unlock(&sq->lock);
}

// Return the latest clock estimate
void
serialqueue_get_clock_est(struct serialqueue *sq, struct clock_estimate *ce)
{
    pthread_mutex_lock(&sq->lock);
    memcpy(ce, &sq->ce, sizeof(sq->ce));
    pthread_mutex_unlock(&sq->lock);
}

// Return a string buffer containing statistics for the serial port
void __visible
serialqueue_get_stats(struct serialqueue *sq, char *buf, int len)
{
    if (!buf || len <= 0)
        return;
    struct serialqueue stats;
    pthread_mutex_lock(&sq->lock);
    pthread_mutex_lock(&sq->transmit_requests.lock);
    memcpy(&stats, sq, sizeof(stats));
    pthread_mutex_unlock(&sq->transmit_requests.lock);
    pthread_mutex_unlock(&sq->lock);

    snprintf(buf, len, "bytes_write=%u bytes_read=%u"
             " bytes_retransmit=%u bytes_invalid=%u"
             " send_seq=%u receive_seq=%u retransmit_seq=%u"
             " srtt=%.3f rttvar=%.3f rto=%.3f"
             " ready_bytes=%u upcoming_bytes=%u"
             , stats.bytes_write, stats.bytes_read
             , stats.bytes_retransmit, stats.bytes_invalid
             , (int)stats.send_seq, (int)stats.receive_seq
             , (int)stats.retransmit_seq
             , stats.srtt, stats.rttvar, stats.rto
             , stats.ready_bytes, stats.transmit_requests.upcoming_bytes);
}

// Extract old messages stored in the debug queues
int __visible
serialqueue_extract_old(struct serialqueue *sq, int sentq
                        , struct pull_queue_message *q, int max)
{
    int count = sentq ? DEBUG_QUEUE_SENT : DEBUG_QUEUE_RECEIVE;
    struct list_head replacement, current;
    list_init(&replacement);
    debug_queue_alloc(&replacement, count);
    list_init(&current);

    // Atomically replace existing debug list with new zero'd list
    if (sentq) {
        pthread_mutex_lock(&sq->lock);
        list_join_tail(&sq->old_sent, &current);
        list_init(&sq->old_sent);
        list_join_tail(&replacement, &sq->old_sent);
        pthread_mutex_unlock(&sq->lock);
    } else {
        pthread_mutex_lock(&sq->receiver.lock);
        list_join_tail(&sq->receiver.old_receive, &current);
        list_init(&sq->receiver.old_receive);
        list_join_tail(&replacement, &sq->receiver.old_receive);
        pthread_mutex_unlock(&sq->receiver.lock);
    }

    // Walk the debug list
    int pos = 0;
    while (!list_empty(&current)) {
        struct queue_message *qm = list_first_entry(
            &current, struct queue_message, node);
        if (qm->len && pos < max) {
            struct pull_queue_message *pqm = q++;
            pos++;
            memcpy(pqm->msg, qm->msg, qm->len);
            pqm->len = qm->len;
            pqm->sent_time = qm->sent_time;
            pqm->receive_time = qm->receive_time;
        }
        list_del(&qm->node);
        message_free(qm);
    }
    return pos;
}
