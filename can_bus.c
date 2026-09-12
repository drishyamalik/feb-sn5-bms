#include "can_bus.h"
#include <stdio.h>
#include <string.h>

/* Two buffers on purpose:
 *   queue - what a real receiver would drain with can_recv() (bounded,
 *           wraps, can overflow just like real hardware FIFOs).
 *   trace - every frame ever sent this run, for can_bus_dump() /
 *           debugging / "prove the BMS actually broadcast X" in tests.
 * A real ECU wouldn't keep the trace; it's here purely so main.c and
 * the tests can show their work without a logic analyzer. */
#define TRACE_DEPTH 256

static can_frame_t queue[CAN_BUS_DEPTH];
static size_t q_head = 0, q_count = 0;

static can_frame_t trace[TRACE_DEPTH];
static size_t trace_count = 0;

void can_bus_reset(void) {
    q_head = 0;
    q_count = 0;
    trace_count = 0;
    memset(queue, 0, sizeof(queue));
    memset(trace, 0, sizeof(trace));
}

int can_send(can_frame_t frame) {
    if (trace_count < TRACE_DEPTH) {
        trace[trace_count++] = frame;
    }
    if (q_count >= CAN_BUS_DEPTH) {
        return -1; /* bus full - a real driver would count this as a drop */
    }
    size_t idx = (q_head + q_count) % CAN_BUS_DEPTH;
    queue[idx] = frame;
    q_count++;
    return 0;
}

int can_recv(can_frame_t *out) {
    if (q_count == 0) {
        return -1;
    }
    *out = queue[q_head];
    q_head = (q_head + 1) % CAN_BUS_DEPTH;
    q_count--;
    return 0;
}

size_t can_bus_pending(void) {
    return q_count;
}

void can_bus_dump(void) {
    for (size_t i = 0; i < trace_count; i++) {
        printf("[CAN] t=%6ums id=0x%03X dlc=%u data=", trace[i].timestamp_ms,
               trace[i].id, trace[i].dlc);
        for (int b = 0; b < trace[i].dlc; b++) {
            printf("%02X ", trace[i].data[b]);
        }
        printf("\n");
    }
}
