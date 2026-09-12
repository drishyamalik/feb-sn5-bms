#ifndef BMS_FSM_H
#define BMS_FSM_H

#include "bms_types.h"

typedef enum {
    BMS_STATE_INIT = 0,
    BMS_STATE_STANDBY,
    BMS_STATE_PRECHARGE,
    BMS_STATE_DRIVE,
    BMS_STATE_CHARGE,
    BMS_STATE_BALANCE,
    BMS_STATE_FAULT,
    BMS_NUM_STATES
} bms_state_t;

/* Drive has an internal power-mode sub-state. Modeling it as its own
 * orthogonal region (instead of duplicating every Drive transition
 * into "Drive-Normal" / "Drive-Derated" copies in the main table)
 * keeps the top-level table small and makes derate logic a one-place
 * change - see needs_derate() in bms_fsm.c. */
typedef enum {
    DRIVE_MODE_NORMAL = 0,
    DRIVE_MODE_DERATED,
} drive_mode_t;

typedef enum {
    EV_SELFTEST_PASS,
    EV_SELFTEST_FAIL,
    EV_HV_REQUEST,
    EV_HV_RELEASE,
    EV_PRECHARGE_OK,
    EV_PRECHARGE_TIMEOUT,
    EV_CHARGER_CONNECTED,
    EV_CHARGER_DONE,
    EV_BALANCE_NEEDED,
    EV_BALANCE_DONE,
    EV_FAULT_RAISED,
    EV_FAULT_CLEARED_AND_ACKED,
    EV_TICK,           /* nothing state-changing happened this tick */
    BMS_NUM_EVENTS
} bms_event_t;

typedef struct {
    bms_state_t  state;
    drive_mode_t drive_mode;
    uint32_t     fault_flags;     /* bms_fault_t bitmask, frozen at trip time */
    uint32_t     state_entry_ms;  /* when we entered the current state       */
} bms_fsm_t;

void bms_fsm_init(bms_fsm_t *fsm, uint32_t now_ms);

/* Call once per control-loop tick. Classifies the sensor/driver
 * snapshot into a single event, drives the transition table, fires
 * any entry action (including a CAN broadcast), and returns the event
 * that was processed - callers/tests use the return value to assert
 * on WHY a transition did or didn't happen. */
bms_event_t bms_fsm_step(bms_fsm_t *fsm, const bms_inputs_t *in);

const char *bms_state_name(bms_state_t s);
const char *bms_event_name(bms_event_t e);

#endif /* BMS_FSM_H */
