#include "can_queue.h"
#include <string.h>

can_queue_t can1_rx_queue;
can_queue_t can2_rx_queue;
can_queue_t can3_rx_queue;

can_queue_t can1_tx_queue;
can_queue_t can2_tx_queue;
can_queue_t can3_tx_queue;

void can_queue_init(can_queue_t *q)
{
    q->head = 0;
    q->tail = 0;
}

bool can_queue_push(can_queue_t *q, const can_msg_t *msg)
{
    uint32_t next_head = (q->head + 1) & (CAN_QUEUE_SIZE - 1);

    if (next_head == q->tail) {
        return false;  /* Full — drop message */
    }

    q->buffer[q->head] = *msg;

    __DMB();  /* Ensure the data write above is visible before publishing
                 the new head index to the consumer (main-loop) context. */

    q->head = next_head;
    return true;
}

bool can_queue_pop(can_queue_t *q, can_msg_t *msg)
{
    if (q->head == q->tail) {
        return false;  /* Empty */
    }

    __DMB();  /* Ensure we don't read buffer[] speculatively before
                 observing the producer's updated head index. */

    *msg = q->buffer[q->tail];

    __DMB();  /* Ensure the copy above completes before publishing the
                 new tail index, freeing that slot for the producer. */

    q->tail = (q->tail + 1) & (CAN_QUEUE_SIZE - 1);
    return true;
}

bool can_queue_peek(const can_queue_t *q, can_msg_t *msg)
{
    if (q->head == q->tail) {
        return false;  /* Empty */
    }

    __DMB();

    *msg = q->buffer[q->tail];
    return true;
}

uint32_t can_queue_count(const can_queue_t *q)
{
    return (q->head - q->tail) & (CAN_QUEUE_SIZE - 1);
}

bool can_queue_empty(const can_queue_t *q)
{
    return (q->head == q->tail);
}

bool can_queue_full(const can_queue_t *q)
{
    return (((q->head + 1) & (CAN_QUEUE_SIZE - 1)) == q->tail);
}

can_queue_t *can_tx_queue_for_bus(can_bus_t bus)
{
    switch (bus) {
        case CAN_BUS_1: return &can1_tx_queue;
        case CAN_BUS_2: return &can2_tx_queue;
        case CAN_BUS_3: return &can3_tx_queue;
        default:        return NULL;
    }
}

bool can_tx_enqueue(const can_msg_t *msg)
{
    can_queue_t *q = can_tx_queue_for_bus((can_bus_t)msg->bus);

    if (q == NULL) {
        return false;  /* Unrecognized bus — nowhere to route this message */
    }

    return can_queue_push(q, msg);
}
