#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>
#include <stddef.h>

#define CAN_MAX_DLC   8
#define CAN_BUS_DEPTH 32   /* pending-message ring buffer depth */

/* Made-up but realistic-looking arbitration IDs for a small FSAE bus.
 * Real teams would pull these from the car's DBC file. */
#define CAN_ID_BMS_STATUS      0x100u  /* state, drive_mode, fault flags */
#define CAN_ID_BMS_FAULT       0x102u  /* full 32-bit fault mask, sent on trip */
#define CAN_ID_VCU_HEARTBEAT   0x200u  /* "I'm alive" from the vehicle ctrl unit */
#define CAN_ID_CHARGER_PRESENT 0x300u

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[CAN_MAX_DLC];
    uint32_t timestamp_ms;
} can_frame_t;

void   can_bus_reset(void);
int    can_send(can_frame_t frame);   /* 0 on success, -1 if the queue is full */
int    can_recv(can_frame_t *out);    /* pops oldest queued frame, -1 if empty */
size_t can_bus_pending(void);
void   can_bus_dump(void);            /* prints every frame ever sent this run */

#endif /* CAN_BUS_H */
