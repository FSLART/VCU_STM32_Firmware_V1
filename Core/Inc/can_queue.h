#ifndef CAN_QUEUE_H
#define CAN_QUEUE_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Configuration */
#define CAN_QUEUE_SIZE 32  /* Must be power of 2 */

/* CAN bus identifiers — used for TX routing */
typedef enum {
    CAN_BUS_1 = 1,   /* Data bus */
    CAN_BUS_2 = 2,   /* Powertrain */
    CAN_BUS_3 = 3,   /* Autonomous */
} can_bus_t;

/* CAN message entry stored in queue */
typedef struct {
    uint32_t id;            /* CAN Standard ID */
    uint8_t  dlc;           /* Data length code (0-8) */
    uint8_t  bus;           /* Target bus for TX (ignored for RX). Holds a
                                can_bus_t value; stored as uint8_t instead of
                                the enum type so struct layout is fixed
                                regardless of -fshort-enums or compiler ABI
                                choices for enum underlying type. */
    uint8_t  data[8];       /* Raw CAN data bytes */
    uint32_t timestamp;     /* HAL_GetTick() at reception */
} can_msg_t;

/* Ring buffer for one CAN bus (used for both RX and TX) */
typedef struct {
    can_msg_t buffer[CAN_QUEUE_SIZE];
    volatile uint32_t head;  /* Write index (ISR context) */
    volatile uint32_t tail;  /* Read index (main context) */
} can_queue_t;

/* Initialize a queue (reset head/tail to 0) */
void can_queue_init(can_queue_t *q);

/* Push message into queue. Call from ISR. Returns true on success, false if full. */
bool can_queue_push(can_queue_t *q, const can_msg_t *msg);

/* Pop message from queue. Call from main context. Returns true if message available, false if empty. */
bool can_queue_pop(can_queue_t *q, can_msg_t *msg);

/* Peek at next message without removing. Returns true if available. */
bool can_queue_peek(const can_queue_t *q, can_msg_t *msg);

/* Return number of messages in queue */
uint32_t can_queue_count(const can_queue_t *q);

/* Return true if queue is empty */
bool can_queue_empty(const can_queue_t *q);

/* Return true if queue is full */
bool can_queue_full(const can_queue_t *q);

/* Three global RX queues, one per bus */
extern can_queue_t can1_rx_queue;
extern can_queue_t can2_rx_queue;
extern can_queue_t can3_rx_queue;

/* Three global TX queues, one per bus.
 *
 * Each STM32F767 bxCAN peripheral instance (CAN1/CAN2/CAN3) has its own
 * independent set of 3 hardware TX mailboxes — mailboxes are NOT shared
 * or pooled across instances. A single shared TX queue therefore creates
 * head-of-line blocking: a message stuck at the front waiting for bus 1's
 * mailboxes can block bus 2/3 messages even when bus 2/3 mailboxes are
 * completely free. Splitting into one queue per bus maps directly onto
 * the hardware and guarantees no bus can starve another. */
extern can_queue_t can1_tx_queue;
extern can_queue_t can2_tx_queue;
extern can_queue_t can3_tx_queue;

/* Push a TX message into the queue matching msg->bus.
 * Returns false if msg->bus is not a recognized bus, or if the target
 * queue is full (queue-full semantics match can_queue_push). */
bool can_tx_enqueue(const can_msg_t *msg);

/* Look up the TX queue for a given bus. Returns NULL for an unrecognized
 * bus value. Useful when the caller already knows the bus and wants to
 * pop directly (e.g. from each bus's TX-mailbox-empty handling). */
can_queue_t *can_tx_queue_for_bus(can_bus_t bus);

#endif /* CAN_QUEUE_H */
