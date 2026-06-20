#include "can_driver.h"

#include <string.h>

#include "can.h"

/* Debug — watch in debugger */
volatile HAL_StatusTypeDef can_tx_result = HAL_OK;
volatile can_bus_t can_tx_last_bus = CAN_BUS_1;

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

    /* Enable NVIC vectors for all three CAN buses */
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

    /* Now activate RX FIFO0 message pending notifications */
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
    can_queue_t* q;
    if (hcan->Instance == CAN1)
        q = &can1_rx_queue;
    else if (hcan->Instance == CAN2)
        q = &can2_rx_queue;
    else if (hcan->Instance == CAN3)
        q = &can3_rx_queue;
    else
        return;

    CAN_RxHeaderTypeDef rx_header;
    can_msg_t msg;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK)
        return;

    msg.id = rx_header.StdId;
    msg.dlc = rx_header.DLC;
    msg.timestamp = HAL_GetTick();

    can_queue_push(q, &msg);
}

void can_driver_tx_drain(void) {
    can_msg_t msg;
    CAN_TxHeaderTypeDef header = {
        .RTR = CAN_RTR_DATA,
        .IDE = CAN_ID_STD,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;

    /* Single-pass drain from one shared TX queue.
       Try one message per call. If mailbox full, leave in queue for next iteration. */
    if (can_queue_pop(&can_tx_queue, &msg)) {
        CAN_HandleTypeDef* hcan;
        switch (msg.bus) {
            case CAN_BUS_1:
                hcan = &hcan1;
                break;
            case CAN_BUS_2:
                hcan = &hcan2;
                break;
            case CAN_BUS_3:
                hcan = &hcan3;
                break;
            default:
                return; /* Invalid bus — drop */
        }
        header.StdId = msg.id;
        header.DLC = msg.dlc;
        can_tx_result = HAL_CAN_AddTxMessage(hcan, &header, msg.data, &mailbox);
        can_tx_last_bus = msg.bus;
        if (can_tx_result != HAL_OK)
            can_queue_push(&can_tx_queue, &msg); /* Re-queue for next iteration */
    }
}
