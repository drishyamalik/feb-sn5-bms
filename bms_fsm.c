#include "bms_fsm.h"
#include "can_bus.h"
#include <string.h>
#include <stddef.h>

/* =======================================================================
 * Sensor evaluation - the ONLY place that reads bms_inputs_t arrays.
 * Adding a new fault condition means touching this function and
 * bms_types.h; it never means touching the transition table below.
 * ===================================================================== */

static uint32_t evaluate_faults(const bms_inputs_t *in) {
    uint32_t flags = FAULT_NONE;

    for (int i = 0; i < NUM_MODULES; i++) {
        if (in->cell_voltage[i] > CELL_V_OVERVOLT)  flags |= FAULT_CELL_OVERVOLT;
        if (in->cell_voltage[i] < CELL_V_UNDERVOLT) flags |= FAULT_CELL_UNDERVOLT;
        if (in->cell_temp_c[i]  > TEMP_FAULT_C)     flags |= FAULT_OVERTEMP;
    }
    if (in->pack_current_a > CURRENT_FAULT_A)               flags |= FAULT_OVERCURRENT;
    if (in->pack_current_a < REGEN_CURRENT_FAULT_A)         flags |= FAULT_OVERCURRENT;
    if (!in->imd_ok)                                         flags |= FAULT_ISOLATION;
    if (!in->bspd_ok)                                        flags |= FAULT_BSPD;
    if (in->vcu_heartbeat_age_ms > VCU_HEARTBEAT_TIMEOUT_MS) flags |= FAULT_COMMS_LOSS;

    return flags;
}

static int needs_derate(const bms_inputs_t *in) {
    for (int i = 0; i < NUM_MODULES; i++) {
        if (in->cell_temp_c[i] > TEMP_WARN_C)          return 1;
        if (in->cell_voltage[i] < CELL_V_UNDERVOLT_WARN) return 1;
    }
    if (in->pack_current_a > CURRENT_WARN_A) return 1;
    return 0;
}

static int needs_balance(const bms_inputs_t *in) {
    float vmin = in->cell_voltage[0];
    float vmax = in->cell_voltage[0];
    for (int i = 1; i < NUM_MODULES; i++) {
        if (in->cell_voltage[i] < vmin) vmin = in->cell_voltage[i];
        if (in->cell_voltage[i] > vmax) vmax = in->cell_voltage[i];
    }
    return (vmax - vmin) > BALANCE_DELTA_V && vmax > BALANCE_NEAR_FULL_V;
}

static float pack_voltage_estimate(const bms_inputs_t *in) {
    float sum = 0.0f;
    for (int i = 0; i < NUM_MODULES; i++) sum += in->cell_voltage[i];
    return sum;
}

/* =======================================================================
 * CAN adapter - the FSM's only side effect. Broadcasting is an entry
 * action, never something the classifier or a test has to poke at
 * directly.
 * ===================================================================== */

static void can_broadcast_status(const bms_fsm_t *fsm, const bms_inputs_t *in) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = CAN_ID_BMS_STATUS;
    f.dlc = 3;
    f.data[0] = (uint8_t)fsm->state;
    f.data[1] = (uint8_t)fsm->drive_mode;
    f.data[2] = (uint8_t)(fsm->fault_flags & 0xFFu);
    f.timestamp_ms = in->now_ms;
    can_send(f);
}

static void can_broadcast_fault(const bms_fsm_t *fsm, const bms_inputs_t *in) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = CAN_ID_BMS_FAULT;
    f.dlc = 4;
    f.data[0] = (uint8_t)((fsm->fault_flags >> 0)  & 0xFF);
    f.data[1] = (uint8_t)((fsm->fault_flags >> 8)  & 0xFF);
    f.data[2] = (uint8_t)((fsm->fault_flags >> 16) & 0xFF);
    f.data[3] = (uint8_t)((fsm->fault_flags >> 24) & 0xFF);
    f.timestamp_ms = in->now_ms;
    can_send(f);
}

/* =======================================================================
 * Entry actions
 * ===================================================================== */

static void action_enter_standby(bms_fsm_t *fsm, const bms_inputs_t *in) {
    fsm->fault_flags = FAULT_NONE;
    can_broadcast_status(fsm, in);
}
static void action_enter_precharge(bms_fsm_t *fsm, const bms_inputs_t *in) {
    can_broadcast_status(fsm, in);
}
static void action_enter_drive(bms_fsm_t *fsm, const bms_inputs_t *in) {
    fsm->drive_mode = DRIVE_MODE_NORMAL;
    can_broadcast_status(fsm, in);
}
static void action_enter_charge(bms_fsm_t *fsm, const bms_inputs_t *in) {
    can_broadcast_status(fsm, in);
}
static void action_enter_balance(bms_fsm_t *fsm, const bms_inputs_t *in) {
    can_broadcast_status(fsm, in);
}
static void action_enter_fault(bms_fsm_t *fsm, const bms_inputs_t *in) {
    /* Real hardware: open the AIRs right here. In this simulation the
     * "contactor" is just whatever main.c does in response to seeing
     * state == BMS_STATE_FAULT on the bus - the FSM's job stops at
     * telling the rest of the car the truth as loudly as possible. */
    can_broadcast_status(fsm, in);
    can_broadcast_fault(fsm, in);
}

/* Precharge timeout doesn't come from evaluate_faults() (it's a
 * state-specific timing condition, not a sensor reading out of
 * range), so classify_event() never populates fault_flags for it.
 * Set the bit explicitly here so the CAN fault frame still tells the
 * truth about why the AIRs opened. */
static void action_enter_fault_precharge_timeout(bms_fsm_t *fsm, const bms_inputs_t *in) {
    fsm->fault_flags |= FAULT_PRECHARGE_TIMEOUT;
    action_enter_fault(fsm, in);
}

/* =======================================================================
 * Transition table
 *
 * Each row is (state we're in, event that fires, action to run,
 * state we land in). bms_fsm_step() below is a ~15-line loop over
 * this table - adding a state or transition means adding a row, not
 * touching the dispatcher.
 * ===================================================================== */

typedef void (*bms_action_fn)(bms_fsm_t *fsm, const bms_inputs_t *in);

typedef struct {
    bms_state_t   from;
    bms_event_t   event;
    bms_action_fn action; /* may be NULL for a state change with no side effect */
    bms_state_t   to;
} bms_transition_t;

static const bms_transition_t TRANSITIONS[] = {
    { BMS_STATE_INIT,      EV_SELFTEST_PASS,          action_enter_standby,   BMS_STATE_STANDBY },
    { BMS_STATE_INIT,      EV_SELFTEST_FAIL,           action_enter_fault,     BMS_STATE_FAULT   },

    { BMS_STATE_STANDBY,   EV_HV_REQUEST,               action_enter_precharge, BMS_STATE_PRECHARGE },
    { BMS_STATE_STANDBY,   EV_CHARGER_CONNECTED,        action_enter_charge,    BMS_STATE_CHARGE  },

    { BMS_STATE_PRECHARGE, EV_PRECHARGE_OK,             action_enter_drive,     BMS_STATE_DRIVE   },
    { BMS_STATE_PRECHARGE, EV_PRECHARGE_TIMEOUT,        action_enter_fault_precharge_timeout, BMS_STATE_FAULT },

    { BMS_STATE_DRIVE,     EV_HV_RELEASE,               action_enter_standby,   BMS_STATE_STANDBY },

    { BMS_STATE_CHARGE,    EV_CHARGER_DONE,             action_enter_standby,   BMS_STATE_STANDBY },
    { BMS_STATE_CHARGE,    EV_BALANCE_NEEDED,           action_enter_balance,   BMS_STATE_BALANCE },

    { BMS_STATE_BALANCE,   EV_BALANCE_DONE,             action_enter_charge,    BMS_STATE_CHARGE  },

    { BMS_STATE_FAULT,     EV_FAULT_CLEARED_AND_ACKED,  action_enter_standby,   BMS_STATE_STANDBY },
};
#define NUM_TRANSITIONS (sizeof(TRANSITIONS) / sizeof(TRANSITIONS[0]))

/* =======================================================================
 * Event classifier
 *
 * Turns "the whole world right now" into a single event. Faults are
 * evaluated fresh every tick and always take priority over whatever
 * state-specific logic would otherwise apply - a defense-in-depth
 * choice: even if a state's own branch below has a bug, the fault
 * check upstream of the switch still catches it.
 * ===================================================================== */

static bms_event_t classify_event(bms_fsm_t *fsm, const bms_inputs_t *in) {
    uint32_t faults_now = evaluate_faults(in);

    if (fsm->state != BMS_STATE_FAULT && faults_now != FAULT_NONE) {
        fsm->fault_flags = faults_now; /* freeze the reason at trip time */
        return EV_FAULT_RAISED;
    }

    if (fsm->state == BMS_STATE_FAULT) {
        /* Latched: only leaves once the world is clean AND a human has
         * physically acknowledged it. Never auto-clears. */
        if (faults_now == FAULT_NONE && in->fault_ack_request) {
            return EV_FAULT_CLEARED_AND_ACKED;
        }
        return EV_TICK;
    }

    switch (fsm->state) {
    case BMS_STATE_INIT:
        return in->selftest_ok ? EV_SELFTEST_PASS : EV_SELFTEST_FAIL;

    case BMS_STATE_STANDBY:
        if (in->driver_hv_request) return EV_HV_REQUEST;
        if (in->charger_present)   return EV_CHARGER_CONNECTED;
        return EV_TICK;

    case BMS_STATE_PRECHARGE: {
        float delta = in->bus_voltage - pack_voltage_estimate(in);
        if (delta < 0) delta = -delta;
        if (delta <= PRECHARGE_V_DELTA_OK) return EV_PRECHARGE_OK;
        if (in->now_ms - fsm->state_entry_ms >= PRECHARGE_TIMEOUT_MS) return EV_PRECHARGE_TIMEOUT;
        return EV_TICK;
    }

    case BMS_STATE_DRIVE:
        if (!in->driver_hv_request) return EV_HV_RELEASE;
        return EV_TICK; /* derate is handled as a sub-mode, not a top event */

    case BMS_STATE_CHARGE:
        if (!in->charger_present) return EV_CHARGER_DONE;
        if (needs_balance(in))    return EV_BALANCE_NEEDED;
        return EV_TICK;

    case BMS_STATE_BALANCE:
        if (!needs_balance(in)) return EV_BALANCE_DONE;
        return EV_TICK;

    default:
        return EV_TICK;
    }
}

/* =======================================================================
 * Public API
 * ===================================================================== */

void bms_fsm_init(bms_fsm_t *fsm, uint32_t now_ms) {
    memset(fsm, 0, sizeof(*fsm));
    fsm->state = BMS_STATE_INIT;
    fsm->drive_mode = DRIVE_MODE_NORMAL;
    fsm->fault_flags = FAULT_NONE;
    fsm->state_entry_ms = now_ms;
}

bms_event_t bms_fsm_step(bms_fsm_t *fsm, const bms_inputs_t *in) {
    bms_event_t ev = classify_event(fsm, in);

    if (ev == EV_FAULT_RAISED) {
        fsm->state = BMS_STATE_FAULT;
        fsm->state_entry_ms = in->now_ms;
        action_enter_fault(fsm, in);
        return ev;
    }

    /* Orthogonal sub-region: only meaningful while actually driving,
     * evaluated independently of whatever the top-level table does
     * this tick. */
    if (fsm->state == BMS_STATE_DRIVE) {
        drive_mode_t want = needs_derate(in) ? DRIVE_MODE_DERATED : DRIVE_MODE_NORMAL;
        if (want != fsm->drive_mode) {
            fsm->drive_mode = want;
            can_broadcast_status(fsm, in);
        }
    }

    for (size_t i = 0; i < NUM_TRANSITIONS; i++) {
        if (TRANSITIONS[i].from == fsm->state && TRANSITIONS[i].event == ev) {
            fsm->state = TRANSITIONS[i].to;
            fsm->state_entry_ms = in->now_ms;
            if (TRANSITIONS[i].action) {
                TRANSITIONS[i].action(fsm, in);
            }
            break;
        }
    }
    return ev;
}

const char *bms_state_name(bms_state_t s) {
    switch (s) {
    case BMS_STATE_INIT:      return "INIT";
    case BMS_STATE_STANDBY:   return "STANDBY";
    case BMS_STATE_PRECHARGE: return "PRECHARGE";
    case BMS_STATE_DRIVE:     return "DRIVE";
    case BMS_STATE_CHARGE:    return "CHARGE";
    case BMS_STATE_BALANCE:   return "BALANCE";
    case BMS_STATE_FAULT:     return "FAULT";
    default:                  return "UNKNOWN";
    }
}

const char *bms_event_name(bms_event_t e) {
    switch (e) {
    case EV_SELFTEST_PASS:          return "SELFTEST_PASS";
    case EV_SELFTEST_FAIL:          return "SELFTEST_FAIL";
    case EV_HV_REQUEST:             return "HV_REQUEST";
    case EV_HV_RELEASE:             return "HV_RELEASE";
    case EV_PRECHARGE_OK:           return "PRECHARGE_OK";
    case EV_PRECHARGE_TIMEOUT:      return "PRECHARGE_TIMEOUT";
    case EV_CHARGER_CONNECTED:      return "CHARGER_CONNECTED";
    case EV_CHARGER_DONE:           return "CHARGER_DONE";
    case EV_BALANCE_NEEDED:         return "BALANCE_NEEDED";
    case EV_BALANCE_DONE:           return "BALANCE_DONE";
    case EV_FAULT_RAISED:           return "FAULT_RAISED";
    case EV_FAULT_CLEARED_AND_ACKED:return "FAULT_CLEARED_AND_ACKED";
    case EV_TICK:                   return "TICK";
    default:                        return "UNKNOWN";
    }
}
