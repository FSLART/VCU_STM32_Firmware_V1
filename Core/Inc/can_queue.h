#ifndef CAN_QUEUE_H
#define CAN_QUEUE_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Configuration */
#define CAN_QUEUE_SIZE 32  /* Must be power of 2 */

/* CAN message entry stored in queue */
typedef struct {
    uint32_t id;            /* CAN Standard ID */
    uint8_t  dlc;           /* Data length code (0-8) */
    uint8_t  data[8];       /* Raw CAN data bytes */
    uint32_t timestamp;     /* HAL_GetTick() at reception */
} can_msg_t;

/* Ring buffer for one CAN bus */
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

/* Three global queues, one per bus */
extern can_queue_t can1_rx_queue;
extern can_queue_t can2_rx_queue;
extern can_queue_t can3_rx_queue;

#endif /* CAN_QUEUE_H */
