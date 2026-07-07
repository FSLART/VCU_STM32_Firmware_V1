#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "stm32f7xx_hal.h"
#include "can_queue.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t can1_tx_count;
    uint32_t can2_tx_count;
    uint32_t can3_tx_count;
    uint32_t can1_rx_count;
    uint32_t can2_rx_count;
    uint32_t can3_rx_count;
    uint32_t can1_tx_dropped;
    uint32_t can2_tx_dropped;
    uint32_t can3_tx_dropped;
} can_stats_t;

/* Debug counters — incremented on each successful hardware TX/RX.
 * Defined in can_driver.c; declare extern here so any module that
 * includes this header can reference them (e.g. for debugger watch). */
extern volatile can_stats_t can_stats;

/* Initialize filters, activate RX + TX interrupts, start all three CAN buses */
void can_driver_init(void);

/* Send a standard CAN frame (blocking, direct — for one-off use) */
bool can_driver_tx(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t *data, uint8_t len);

/* Polling read: try to read one frame from RX FIFO0 (non-blocking). */
bool can_driver_rx_poll(CAN_HandleTypeDef *hcan, can_msg_t *msg);

/* Called from HAL_CAN_RxFifo0MsgPendingCallback — routes to correct RX queue */
void can_driver_rx_isr(CAN_HandleTypeDef *hcan);

/* Poll TX queues — try one message per bus, call from main loop */
void can_driver_tx_poll(void);

#endif /* CAN_DRIVER_H */
