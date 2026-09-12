# SN5 BMS state machine

A from-scratch (no libraries) battery management state machine for the
accumulator, plus a fake CAN bus and a hand-rolled unit test harness.
Plain C11, compiles with just `gcc` - no build system, no package
manager, no dependencies to install.

## Files

| File              | What's in it                                                        |
|-------------------|-----------------------------------------------------------------------|
| `bms_types.h`     | Pack layout constants, voltage/temp/current thresholds, the `bms_inputs_t` sensor snapshot struct, and the fault bitmask |
| `bms_fsm.h/.c`    | The state machine itself: fault/derate/balance evaluation, the event classifier, and a **table-driven** transition dispatcher |
| `can_bus.h/.c`    | A fake CAN bus - fixed-size ring buffer + a full trace log, no real hardware needed |
| `main.c`          | A scripted demo: power-up, drive with a mid-session derate, a full charge+balance cycle, and a forced fault with manual reset - prints every transition and the full CAN trace |
| `test_bms_fsm.c`  | 21 hand-rolled tests (68 individual checks), zero test framework dependency |

## Build & run

```
make          # builds bms_demo and bms_test
make run      # runs the scripted demo
make test     # runs the unit tests
```

Both binaries compile clean under `-Wall -Wextra -Wpedantic`.

## Architecture, in short

**The FSM core never touches CAN, GPIO, or ADCs.** Every tick it's handed
one `bms_inputs_t` struct - a plain snapshot of every sensor and driver
input - and it returns the event it processed. That's the whole
interface. It means the 21 tests in `test_bms_fsm.c` don't simulate any
hardware or bus traffic at all; they just build input structs and check
the resulting state, which is exactly why zero test framework was
needed - it's all just data in, data out.

**States are transitioned through a table, not a switch statement.**
`bms_fsm.c` defines an array of `{from_state, event, action_fn,
to_state}` rows. Adding a state or a transition means adding a row, not
touching a dispatcher. Adding a *new fault condition* means touching
exactly one function, `evaluate_faults()` - the table itself never
changes for that.

**Sensor evaluation is a separate phase from state dispatch.** Each
tick, `classify_event()` looks at the current state and the sensor
snapshot and boils it down to a single event (`EV_HV_REQUEST`,
`EV_PRECHARGE_TIMEOUT`, `EV_FAULT_RAISED`, ...). Only after that does the
transition table get consulted. This two-phase split - classify, then
dispatch - is what keeps "did a cell overheat" logic completely
separate from "what state do we go to when that happens" logic.

**Faults always win, from any state, via one shared check** - rather
than writing an `EV_FAULT_RAISED` row for every single state (six
near-duplicate rows), `classify_event()` checks `evaluate_faults()`
before it even looks at what state it's in. One state (`FAULT`) is
latched: even once sensors read clean again, the machine stays in
`FAULT` until a `fault_ack_request` (a physical reset button, in
reality) shows up on top of clean sensors. That mirrors how a real AMS
has to behave - a fault should never clear itself just because the
number went back in range for one tick.

**Drive's thermal/voltage derating is an orthogonal sub-state, not a
top-level state.** `drive_mode` (`NORMAL` / `DERATED`) is evaluated
independently of the main transition table, once per tick, only while
`state == BMS_STATE_DRIVE`. This avoids doubling every Drive-related
row in the table into Normal/Derated copies, and it's a direct answer
to "how would this scale" - a deeper pack (say, per-segment states)
would get its own small orthogonal region the same way, instead of a
combinatorial explosion of top-level states.

**CAN is a thin adapter, used only as an entry action.** Every state
change broadcasts a 3-byte status frame (state, drive_mode, fault
bitmask); entering `FAULT` additionally broadcasts a dedicated 4-byte
fault frame with the full bitmask, so a dash display or a data logger
downstream knows not just *that* the AIRs opened but *why*.

## A real bug this caught

Early on, entry actions were firing *before* `fsm->state` was updated
to the new state - so the CAN status frame for every transition was
one state behind (a `Standby` entry broadcast still said `Init`). The
`state_entry_broadcasts_on_can_bus` test caught it immediately. Fixed
by reordering `bms_fsm_step()` to set `fsm->state` before calling the
transition's action. Left it in the test names/history on purpose -
it's a good example of exactly the kind of ordering bug a table-driven
FSM makes easy to introduce and easy to catch.

## Placeholders

Pack size (`NUM_SEGMENTS` / `MODULES_PER_SEGMENT`) is still a
placeholder.

Voltage, temperature, and current thresholds in `bms_types.h` are 
sourced from the **Energus Li4P25RT datasheet** (Energus
Power Solutions Ltd., revision A, 2017-01-16) - the actual 1s4p module
SN4 used, built from Samsung INR18650-25R cells. Every hard limit
(`CELL_V_OVERVOLT`, `CELL_V_UNDERVOLT`, `TEMP_FAULT_DISCHARGE_C`,
`TEMP_FAULT_CHARGE_C`, `CURRENT_FAULT_A`, `REGEN_CURRENT_FAULT_A`) is
either taken directly from the datasheet's Table 1 or derived from it
with the reasoning documented right next to the `#define`.

The one category that's still a judgment call, and always will be:
the `WARN_*` thresholds (the derate band before a hard fault). The
datasheet states hard min/max limits, not a "back off before you get
there" margin - that margin is inherently a design decision, not
something you look up.

Getting the real datasheet also caught a modeling mistake: the
original placeholder used one flat temperature limit and roughly
symmetric current limits for both charging and discharging. The real
datasheet gives charge and discharge *different* limits for both
(45C vs 60C; ~15-20A vs 120A) - Li-ion cells tolerate far less current
going in than coming out. `bms_fsm.c` now reads the direction off
`pack_current_a`'s sign (`is_charging()`) and picks the right limit
regardless of which top-level state triggered it - including regen
braking during Drive, which is electrochemically a charge event even
though the car is still in the `Drive` state.
