#include "can_queue.h"
#include <string.h>

can_queue_t can1_rx_queue;
can_queue_t can2_rx_queue;
can_queue_t can3_rx_queue;

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
    q->head = next_head;
    return true;
}

bool can_queue_pop(can_queue_t *q, can_msg_t *msg)
{
    if (q->head == q->tail) {
        return false;  /* Empty */
    }

    *msg = q->buffer[q->tail];
    q->tail = (q->tail + 1) & (CAN_QUEUE_SIZE - 1);
    return true;
}

bool can_queue_peek(const can_queue_t *q, can_msg_t *msg)
{
    if (q->head == q->tail) {
        return false;  /* Empty */
    }

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
