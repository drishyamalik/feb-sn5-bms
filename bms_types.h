#ifndef BMS_TYPES_H
#define BMS_TYPES_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 * Pack layout constants
 *
 * SN4's segments were built from Energus 1s4p 18650 modules (4 cells
 * in parallel form one "module", modules stacked in series to build
 * segment/pack voltage). NUM_SEGMENTS / MODULES_PER_SEGMENT below are
 * placeholders sized to look like a real FSAE pack (~220 V nominal) -
 * swap in SN5's real CAD numbers once the pack is finalized.
 *
 * Voltage/temp/current thresholds are representative of the 18650
 * Li-ion cells (Samsung/Sony) commonly used in Energus-style FSAE
 * modules - NOT pulled from a specific verified datasheet. Treat them
 * as "plausible defaults to argue with", not gospel.
 * ------------------------------------------------------------------- */
#define NUM_SEGMENTS         5      /* series segments in the accumulator  */
#define MODULES_PER_SEGMENT  12     /* 1s4p modules in series, per segment */
#define NUM_MODULES          (NUM_SEGMENTS * MODULES_PER_SEGMENT)

/* Per-module (per series position) voltage thresholds, volts. */
#define CELL_V_NOMINAL         3.6f
#define CELL_V_MAX_CHARGE      4.20f
#define CELL_V_OVERVOLT        4.25f   /* fault: above charge cutoff       */
#define CELL_V_UNDERVOLT_WARN  3.20f   /* warning band -> derate           */
#define CELL_V_UNDERVOLT       2.80f   /* fault: below safe discharge floor*/

/* Module hot-spot temperature thresholds, deg C. */
#define TEMP_WARN_C   45.0f
#define TEMP_FAULT_C  60.0f

/* Pack current thresholds, amps, bus-level (+discharge / -charge-regen). */
#define CURRENT_FAULT_A        180.0f
#define CURRENT_WARN_A         150.0f
#define REGEN_CURRENT_FAULT_A -120.0f

/* Cell-balance thresholds, volts. */
#define BALANCE_DELTA_V        0.05f
#define BALANCE_NEAR_FULL_V    4.10f

/* Timing, milliseconds. */
#define PRECHARGE_TIMEOUT_MS      2500u
#define PRECHARGE_V_DELTA_OK      2.0f
#define VCU_HEARTBEAT_TIMEOUT_MS  500u

/* One sensor/driver snapshot fed into the FSM each control-loop tick.
 * This is the ONLY interface between "the world" and the state
 * machine - the FSM core never touches CAN, GPIO, or ADCs directly,
 * which is what makes it possible to unit test without any hardware
 * or bus simulation at all. */
typedef struct {
    float    cell_voltage[NUM_MODULES];  /* one reading per series module */
    float    cell_temp_c[NUM_MODULES];   /* hot-spot sensor per module    */
    float    pack_current_a;             /* +discharge, -charge/regen     */
    float    bus_voltage;                /* inverter-side DC bus voltage  */
    uint8_t  imd_ok;                     /* insulation monitoring device  */
    uint8_t  bspd_ok;                    /* brake system plausibility dev */
    uint8_t  selftest_ok;                /* internal diagnostics pass     */
    uint8_t  driver_hv_request;          /* start button / ignition       */
    uint8_t  charger_present;            /* charger seen present          */
    uint8_t  fault_ack_request;          /* physical reset button pressed */
    uint32_t vcu_heartbeat_age_ms;       /* time since last VCU CAN frame */
    uint32_t now_ms;                     /* simulated clock               */
} bms_inputs_t;

/* Bitmask fault reasons, broadcast over CAN so downstream boards and
 * the driver display know WHY the AIRs opened, not just that they did. */
typedef enum {
    FAULT_NONE              = 0,
    FAULT_CELL_OVERVOLT     = 1u << 0,
    FAULT_CELL_UNDERVOLT    = 1u << 1,
    FAULT_OVERTEMP          = 1u << 2,
    FAULT_OVERCURRENT       = 1u << 3,
    FAULT_ISOLATION         = 1u << 4,
    FAULT_PRECHARGE_TIMEOUT = 1u << 5,
    FAULT_COMMS_LOSS        = 1u << 6,
    FAULT_BSPD              = 1u << 7,
} bms_fault_t;

#endif /* BMS_TYPES_H */
