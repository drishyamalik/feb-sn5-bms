#include <stdio.h>
#include <string.h>
#include "bms_fsm.h"
#include "can_bus.h"

static void make_nominal(bms_inputs_t *in, uint32_t now_ms) {
    memset(in, 0, sizeof(*in));
    for (int i = 0; i < NUM_MODULES; i++) {
        in->cell_voltage[i] = 3.7f;
        in->cell_temp_c[i]  = 25.0f;
    }
    in->pack_current_a       = 0.0f;
    in->bus_voltage          = 0.0f;
    in->imd_ok                = 1;
    in->bspd_ok                = 1;
    in->selftest_ok            = 1;
    in->vcu_heartbeat_age_ms  = 0;
    in->now_ms                = now_ms;
}

static void log_step(const bms_fsm_t *fsm, bms_event_t ev, bms_state_t prev) {
    if (fsm->state != prev) {
        printf("t=%6ums  %-9s --[%s]--> %-9s  (drive_mode=%s)\n",
               fsm->state_entry_ms, bms_state_name(prev), bms_event_name(ev),
               bms_state_name(fsm->state),
               fsm->drive_mode == DRIVE_MODE_DERATED ? "DERATED" : "normal");
    }
}

int main(void) {
    can_bus_reset();

    bms_fsm_t fsm;
    bms_fsm_init(&fsm, 0);
    bms_inputs_t in;

    printf("=== SN5 BMS state machine demo ===\n\n");
    printf("--- Power-up + drive session ---\n");

    uint32_t t = 0;
    bms_state_t prev = fsm.state;

    /* Init -> Standby */
    make_nominal(&in, t);
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    /* Standby -> Precharge (driver hits start button) */
    t += 10;
    make_nominal(&in, t);
    in.driver_hv_request = 1;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    /* Precharge ticks until the bus catches up to pack voltage */
    float pack_v = 3.7f * NUM_MODULES;
    for (int i = 0; i < 6; i++) {
        t += 200;
        make_nominal(&in, t);
        in.driver_hv_request = 1;
        in.bus_voltage = pack_v * (0.5f + 0.1f * i); /* ramping bus voltage */
        prev = fsm.state;
        log_step(&fsm, bms_fsm_step(&fsm, &in), prev);
        if (fsm.state != BMS_STATE_PRECHARGE) break;
    }

    /* Drive normally for a bit, then push a hotspot into the warn band
     * to demonstrate the derate sub-mode, then let it cool back off. */
    for (int i = 0; i < 4; i++) {
        t += 100;
        make_nominal(&in, t);
        in.driver_hv_request = 1;
        in.bus_voltage = pack_v;
        if (i == 2) in.cell_temp_c[7] = 48.0f; /* nudges into DERATED */
        prev = fsm.state;
        log_step(&fsm, bms_fsm_step(&fsm, &in), prev);
        if (fsm.drive_mode == DRIVE_MODE_DERATED) {
            printf("t=%6ums  (still DRIVE, but drive_mode=DERATED - module 7 at %.1fC)\n",
                   t, in.cell_temp_c[7]);
        }
    }

    /* Driver releases HV -> back to Standby */
    t += 100;
    make_nominal(&in, t);
    in.driver_hv_request = 0;
    in.bus_voltage = pack_v;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    printf("\n--- Charging session ---\n");

    /* Standby -> Charge */
    t += 10;
    make_nominal(&in, t);
    in.charger_present = 1;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    /* Charge -> Balance: one module races ahead of the pack near full */
    t += 100;
    make_nominal(&in, t);
    in.charger_present = 1;
    for (int i = 0; i < NUM_MODULES; i++) in.cell_voltage[i] = 4.12f;
    in.cell_voltage[3] = 4.18f; /* outlier, 60 mV above the rest */
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    /* Balance -> Charge: bleeding resistors have pulled it back in line */
    t += 500;
    make_nominal(&in, t);
    in.charger_present = 1;
    for (int i = 0; i < NUM_MODULES; i++) in.cell_voltage[i] = 4.12f;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    /* Charger unplugged -> Standby */
    t += 10;
    make_nominal(&in, t);
    in.charger_present = 0;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    printf("\n--- Fault + manual reset ---\n");

    /* Driver drives again, then a module hard-trips overtemp */
    t += 10;
    make_nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev); /* -> Precharge */

    t += 200;
    make_nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev); /* -> Drive */

    t += 100;
    make_nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v;
    in.cell_temp_c[19] = 63.0f; /* over TEMP_FAULT_C */
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev); /* -> Fault */
    printf("t=%6ums  fault_flags=0x%02X (bit2 = FAULT_OVERTEMP)\n", t, fsm.fault_flags);

    /* Fault does NOT clear on its own even once temps normalize */
    t += 100;
    make_nominal(&in, t);
    prev = fsm.state;
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    printf("t=%6ums  still %s even though sensors are clean (ev=%s, no ack yet)\n",
           t, bms_state_name(fsm.state), bms_event_name(ev));

    /* Operator presses the physical reset -> Standby */
    t += 100;
    make_nominal(&in, t);
    in.fault_ack_request = 1;
    prev = fsm.state;
    log_step(&fsm, bms_fsm_step(&fsm, &in), prev);

    printf("\n--- CAN bus trace (everything the BMS broadcast this run) ---\n");
    can_bus_dump();

    return 0;
}
