#include <stdio.h>
#include <string.h>
#include "bms_fsm.h"
#include "can_bus.h"

/* ---- tiny hand-rolled test harness, zero dependencies ---- */
static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(cond) do { \
    tests_run++; \
    if (!(cond)) { \
        tests_failed++; \
        printf("  FAIL line %d: %s\n", __LINE__, #cond); \
    } \
} while (0)

#define TEST(name) static void name(void)
#define RUN(name) do { printf("%s\n", #name); name(); } while (0)

/* ---- shared input builders ---- */

static void nominal(bms_inputs_t *in, uint32_t now_ms) {
    memset(in, 0, sizeof(*in));
    for (int i = 0; i < NUM_MODULES; i++) {
        in->cell_voltage[i] = 3.7f;
        in->cell_temp_c[i]  = 25.0f;
    }
    in->imd_ok               = 1;
    in->bspd_ok               = 1;
    in->selftest_ok           = 1;
    in->vcu_heartbeat_age_ms = 0;
    in->now_ms                = now_ms;
}

static float pack_v(void) { return 3.7f * NUM_MODULES; }

/* Drives fsm: Init -> Standby. */
static void to_standby(bms_fsm_t *fsm) {
    bms_inputs_t in;
    bms_fsm_init(fsm, 0);
    nominal(&in, 0);
    bms_fsm_step(fsm, &in);
}

/* Drives fsm: Init -> Standby -> Precharge -> Drive. Returns the next
 * free timestamp so callers can keep advancing the clock. */
static uint32_t to_drive(bms_fsm_t *fsm) {
    bms_inputs_t in;
    uint32_t t = 0;
    to_standby(fsm);

    t += 10;
    nominal(&in, t);
    in.driver_hv_request = 1;
    bms_fsm_step(fsm, &in); /* -> Precharge */

    t += 100;
    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    bms_fsm_step(fsm, &in); /* -> Drive */

    CHECK(fsm->state == BMS_STATE_DRIVE);
    return t + 10;
}

/* =======================================================================
 * Tests
 * ===================================================================== */

TEST(selftest_pass_leads_to_standby) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    bms_fsm_init(&fsm, 0);
    nominal(&in, 0);
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_SELFTEST_PASS);
    CHECK(fsm.state == BMS_STATE_STANDBY);
}

TEST(selftest_fail_leads_to_fault) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    bms_fsm_init(&fsm, 0);
    nominal(&in, 0);
    in.selftest_ok = 0;
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_SELFTEST_FAIL);
    CHECK(fsm.state == BMS_STATE_FAULT);
}

TEST(precharge_success_leads_to_drive) {
    bms_fsm_t fsm;
    uint32_t t = to_drive(&fsm);
    (void)t;
    CHECK(fsm.state == BMS_STATE_DRIVE);
    CHECK(fsm.drive_mode == DRIVE_MODE_NORMAL);
}

TEST(precharge_timeout_leads_to_fault) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    to_standby(&fsm);

    nominal(&in, 10);
    in.driver_hv_request = 1;
    bms_fsm_step(&fsm, &in); /* -> Precharge, entry at t=10 */
    CHECK(fsm.state == BMS_STATE_PRECHARGE);

    /* Bus voltage never converges (stuck contactor / blown resistor). */
    nominal(&in, 10 + PRECHARGE_TIMEOUT_MS + 1);
    in.driver_hv_request = 1;
    in.bus_voltage = 0.0f;
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_PRECHARGE_TIMEOUT);
    CHECK(fsm.state == BMS_STATE_FAULT);
    CHECK((fsm.fault_flags & FAULT_PRECHARGE_TIMEOUT) != 0);
}

TEST(overvoltage_faults_during_drive) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.cell_voltage[12] = 4.30f; /* over CELL_V_OVERVOLT */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK(fsm.state == BMS_STATE_FAULT);
    CHECK((fsm.fault_flags & FAULT_CELL_OVERVOLT) != 0);
}

TEST(undervoltage_faults_during_drive) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.cell_voltage[5] = 2.5f; /* under CELL_V_UNDERVOLT */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK(fsm.state == BMS_STATE_FAULT);
    CHECK((fsm.fault_flags & FAULT_CELL_UNDERVOLT) != 0);
}

TEST(warm_cell_derates_without_faulting) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.cell_temp_c[7] = 48.0f; /* over TEMP_WARN_C, under TEMP_FAULT_C */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_TICK);
    CHECK(fsm.state == BMS_STATE_DRIVE); /* still driving... */
    CHECK(fsm.drive_mode == DRIVE_MODE_DERATED); /* ...just throttled back */

    /* and it un-derates once the hotspot cools back down */
    nominal(&in, t + 100);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    bms_fsm_step(&fsm, &in);
    CHECK(fsm.drive_mode == DRIVE_MODE_NORMAL);
}

TEST(hot_cell_hard_trips_fault) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.cell_temp_c[19] = 61.0f; /* over TEMP_FAULT_C */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK(fsm.state == BMS_STATE_FAULT);
    CHECK((fsm.fault_flags & FAULT_OVERTEMP) != 0);
}

TEST(discharge_overcurrent_faults) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.pack_current_a = 190.0f; /* over CURRENT_FAULT_A */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK((fsm.fault_flags & FAULT_OVERCURRENT) != 0);
}

TEST(regen_overcurrent_faults) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.pack_current_a = -130.0f; /* past REGEN_CURRENT_FAULT_A */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK((fsm.fault_flags & FAULT_OVERCURRENT) != 0);
}

TEST(mild_regen_is_fine) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.pack_current_a = -40.0f; /* normal regen braking, not a fault */
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_TICK);
    CHECK(fsm.state == BMS_STATE_DRIVE);
}

TEST(vcu_comms_loss_faults_during_drive) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.vcu_heartbeat_age_ms = VCU_HEARTBEAT_TIMEOUT_MS + 1;
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK((fsm.fault_flags & FAULT_COMMS_LOSS) != 0);
}

TEST(isolation_fault_from_imd) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.imd_ok = 0;
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK((fsm.fault_flags & FAULT_ISOLATION) != 0);
}

TEST(bspd_fault) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.bspd_ok = 0;
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_FAULT_RAISED);
    CHECK((fsm.fault_flags & FAULT_BSPD) != 0);
}

TEST(fault_does_not_clear_without_ack) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.imd_ok = 0;
    bms_fsm_step(&fsm, &in); /* -> Fault */
    CHECK(fsm.state == BMS_STATE_FAULT);

    /* sensors go clean again, but nobody pressed reset */
    nominal(&in, t + 100);
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_TICK);
    CHECK(fsm.state == BMS_STATE_FAULT); /* still latched */
}

TEST(fault_clears_with_ack_after_conditions_resolve) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    uint32_t t = to_drive(&fsm);

    nominal(&in, t);
    in.driver_hv_request = 1;
    in.bus_voltage = pack_v();
    in.imd_ok = 0;
    bms_fsm_step(&fsm, &in); /* -> Fault */

    /* ack pressed too early, while still faulted - must NOT clear */
    nominal(&in, t + 50);
    in.imd_ok = 0;
    in.fault_ack_request = 1;
    bms_fsm_step(&fsm, &in);
    CHECK(fsm.state == BMS_STATE_FAULT);

    /* conditions clean AND ack pressed - now it clears */
    nominal(&in, t + 100);
    in.fault_ack_request = 1;
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_FAULT_CLEARED_AND_ACKED);
    CHECK(fsm.state == BMS_STATE_STANDBY);
    CHECK(fsm.fault_flags == FAULT_NONE);
}

TEST(charger_connect_from_standby_leads_to_charge) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    to_standby(&fsm);

    nominal(&in, 10);
    in.charger_present = 1;
    bms_event_t ev = bms_fsm_step(&fsm, &in);

    CHECK(ev == EV_CHARGER_CONNECTED);
    CHECK(fsm.state == BMS_STATE_CHARGE);
}

TEST(balance_triggered_by_cell_delta_then_returns_to_charge) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    to_standby(&fsm);

    nominal(&in, 10);
    in.charger_present = 1;
    bms_fsm_step(&fsm, &in); /* -> Charge */
    CHECK(fsm.state == BMS_STATE_CHARGE);

    nominal(&in, 20);
    in.charger_present = 1;
    for (int i = 0; i < NUM_MODULES; i++) in.cell_voltage[i] = 4.12f;
    in.cell_voltage[3] = 4.18f; /* 60 mV outlier, near full */
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_BALANCE_NEEDED);
    CHECK(fsm.state == BMS_STATE_BALANCE);

    nominal(&in, 30);
    in.charger_present = 1;
    for (int i = 0; i < NUM_MODULES; i++) in.cell_voltage[i] = 4.12f; /* now level */
    ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_BALANCE_DONE);
    CHECK(fsm.state == BMS_STATE_CHARGE);
}

TEST(charger_unplug_from_charge_leads_to_standby) {
    bms_fsm_t fsm;
    bms_inputs_t in;
    to_standby(&fsm);

    nominal(&in, 10);
    in.charger_present = 1;
    bms_fsm_step(&fsm, &in); /* -> Charge */

    nominal(&in, 20);
    in.charger_present = 0;
    bms_event_t ev = bms_fsm_step(&fsm, &in);
    CHECK(ev == EV_CHARGER_DONE);
    CHECK(fsm.state == BMS_STATE_STANDBY);
}

TEST(state_entry_broadcasts_on_can_bus) {
    can_bus_reset();
    bms_fsm_t fsm;
    to_standby(&fsm); /* Init -> Standby, one CAN_ID_BMS_STATUS frame */

    can_frame_t f;
    int got = can_recv(&f);
    CHECK(got == 0);
    CHECK(f.id == CAN_ID_BMS_STATUS);
    CHECK(f.data[0] == (uint8_t)BMS_STATE_STANDBY);
}

TEST(fault_entry_broadcasts_fault_frame) {
    can_bus_reset();
    bms_fsm_t fsm;
    bms_inputs_t in;
    bms_fsm_init(&fsm, 0);
    nominal(&in, 0);
    in.selftest_ok = 0;
    bms_fsm_step(&fsm, &in); /* -> Fault */

    /* drain the queue looking for the dedicated fault frame */
    can_frame_t f;
    int saw_fault_frame = 0;
    while (can_recv(&f) == 0) {
        if (f.id == CAN_ID_BMS_FAULT) saw_fault_frame = 1;
    }
    CHECK(saw_fault_frame);
}

int main(void) {
    RUN(selftest_pass_leads_to_standby);
    RUN(selftest_fail_leads_to_fault);
    RUN(precharge_success_leads_to_drive);
    RUN(precharge_timeout_leads_to_fault);
    RUN(overvoltage_faults_during_drive);
    RUN(undervoltage_faults_during_drive);
    RUN(warm_cell_derates_without_faulting);
    RUN(hot_cell_hard_trips_fault);
    RUN(discharge_overcurrent_faults);
    RUN(regen_overcurrent_faults);
    RUN(mild_regen_is_fine);
    RUN(vcu_comms_loss_faults_during_drive);
    RUN(isolation_fault_from_imd);
    RUN(bspd_fault);
    RUN(fault_does_not_clear_without_ack);
    RUN(fault_clears_with_ack_after_conditions_resolve);
    RUN(charger_connect_from_standby_leads_to_charge);
    RUN(balance_triggered_by_cell_delta_then_returns_to_charge);
    RUN(charger_unplug_from_charge_leads_to_standby);
    RUN(state_entry_broadcasts_on_can_bus);
    RUN(fault_entry_broadcasts_fault_frame);

    printf("\n%d/%d checks passed\n", tests_run - tests_failed, tests_run);
    return tests_failed ? 1 : 0;
}
