#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "stm32f7xx_hal.h"
#include "can_queue.h"
#include <stdbool.h>
#include <stdint.h>

/* Initialize filters, activate RX interrupts, start all three CAN buses */
void can_driver_init(void);

/* Send a standard CAN frame (blocking until mailbox free or timeout) */
bool can_driver_tx(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t *data, uint8_t len);

/* Polling read: try to read one frame from RX FIFO0 (non-blocking).
   Returns true if frame read, false if FIFO empty. */
bool can_driver_rx_poll(CAN_HandleTypeDef *hcan, can_msg_t *msg);

/* Called from HAL_CAN_RxFifo0MsgPendingCallback — routes to correct queue */
void can_driver_rx_isr(CAN_HandleTypeDef *hcan);

#endif /* CAN_DRIVER_H */
