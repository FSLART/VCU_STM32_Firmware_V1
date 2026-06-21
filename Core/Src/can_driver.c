#include "can_driver.h"

#include <string.h>

#include "can.h"

/* Debug counters — watch in debugger; incremented on each successful hardware TX/RX */
volatile uint32_t can1_tx_count = 0;
volatile uint32_t can2_tx_count = 0;
volatile uint32_t can3_tx_count = 0;
volatile uint32_t can1_rx_count = 0;
volatile uint32_t can2_rx_count = 0;
volatile uint32_t can3_rx_count = 0;

/* --------------- Filter configuration (accept-all, static) --------------- */

static void CAN1_Filter_Config(void) {
    CAN_FilterTypeDef f = {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .SlaveStartFilterBank = 14, /* CAN1 gets banks 0–13, CAN2 gets 14–27 */
    };
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
        Error_Handler();
}

static void CAN2_Filter_Config(void) {
    CAN_FilterTypeDef f = {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank = 14,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .SlaveStartFilterBank = 14, /* Same split boundary — CAN2 must know where its banks start */
    };
    if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK)
        Error_Handler();
}

static void CAN3_Filter_Config(void) {
    CAN_FilterTypeDef f = {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh = 0x0000,
        .FilterIdLow = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow = 0x0000,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterScale = CAN_FILTERSCALE_32BIT,
    };
    if (HAL_CAN_ConfigFilter(&hcan3, &f) != HAL_OK)
        Error_Handler();
}

/* --------------- Public API --------------- */

void can_driver_init(void) {
    CAN1_Filter_Config();
    CAN2_Filter_Config();
    CAN3_Filter_Config();

    /* Enable NVIC vectors for all three CAN buses — RX */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);

    /* Start CAN buses BEFORE activating notifications (HAL recommended order) */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();
    if (HAL_CAN_Start(&hcan3) != HAL_OK) Error_Handler();

    /* Activate RX FIFO0 message pending notifications */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
    if (HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}

bool can_driver_tx(CAN_HandleTypeDef* hcan, uint32_t std_id, const uint8_t* data, uint8_t len) {
    CAN_TxHeaderTypeDef header = {
        .StdId = std_id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = len,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t*)data, &mailbox) == HAL_OK;
}

bool can_driver_rx_poll(CAN_HandleTypeDef* hcan, can_msg_t* msg) {
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0)
        return false;

    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg->data) != HAL_OK)
        return false;

    msg->id = rx_header.StdId;
    msg->dlc = rx_header.DLC;
    msg->timestamp = HAL_GetTick();
    return true;
}

void can_driver_rx_isr(CAN_HandleTypeDef* hcan) {
    can_queue_t*       q;
    volatile uint32_t *rx_count;
    uint8_t            bus;

    if (hcan->Instance == CAN1) {
        q        = &can1_rx_queue;
        rx_count = &can1_rx_count;
        bus      = CAN_BUS_1;
    } else if (hcan->Instance == CAN2) {
        q        = &can2_rx_queue;
        rx_count = &can2_rx_count;
        bus      = CAN_BUS_2;
    } else if (hcan->Instance == CAN3) {
        q        = &can3_rx_queue;
        rx_count = &can3_rx_count;
        bus      = CAN_BUS_3;
    } else {
        return;
    }

    CAN_RxHeaderTypeDef rx_header;
    can_msg_t msg;

    /* Drain the entire FIFO — bxCAN holds up to 3 messages and the pending
       interrupt may not re-fire if more than one frame arrived between ISR
       calls. Breaking on HAL_OK failure avoids a spin if the FIFO is
       momentarily inconsistent (e.g. overrun). */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK)
            break;

        msg.id        = rx_header.StdId;
        msg.dlc       = rx_header.DLC;
        msg.bus       = bus;
        msg.timestamp = HAL_GetTick();

        can_queue_push(q, &msg);
        (*rx_count)++;
    }
}

void can_driver_tx_poll(void)
{
    CAN_TxHeaderTypeDef header = {
        .RTR               = CAN_RTR_DATA,
        .IDE               = CAN_ID_STD,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;

    /* Try one message from each bus. If mailbox full, leave in queue for next call. */
    can_msg_t msg;

    if (can_queue_pop(&can1_tx_queue, &msg)) {
        header.StdId = msg.id;
        header.DLC   = msg.dlc;
        if (HAL_CAN_AddTxMessage(&hcan1, &header, msg.data, &mailbox) == HAL_OK)
            can1_tx_count++;
        /* else: drop — no re-enqueue */
    }

    if (can_queue_pop(&can2_tx_queue, &msg)) {
        header.StdId = msg.id;
        header.DLC   = msg.dlc;
        if (HAL_CAN_AddTxMessage(&hcan2, &header, msg.data, &mailbox) == HAL_OK)
            can2_tx_count++;
        /* else: drop — no re-enqueue */
    }

    if (can_queue_pop(&can3_tx_queue, &msg)) {
        header.StdId = msg.id;
        header.DLC   = msg.dlc;
        if (HAL_CAN_AddTxMessage(&hcan3, &header, msg.data, &mailbox) == HAL_OK)
            can3_tx_count++;
        /* else: drop — no re-enqueue */
    }
}
