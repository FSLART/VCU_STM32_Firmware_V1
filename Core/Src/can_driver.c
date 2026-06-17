#include "can_driver.h"
#include "can.h"

/* --------------- Filter configuration (accept-all, static) --------------- */

static void CAN1_Filter_Config(void)
{
    CAN_FilterTypeDef f = {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank       = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh     = 0x0000,
        .FilterIdLow      = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow  = 0x0000,
        .FilterMode       = CAN_FILTERMODE_IDMASK,
        .FilterScale      = CAN_FILTERSCALE_32BIT,
    };
    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK)
        Error_Handler();
}

static void CAN2_Filter_Config(void)
{
    CAN_FilterTypeDef f = {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank       = 14,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh     = 0x0000,
        .FilterIdLow      = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow  = 0x0000,
        .FilterMode       = CAN_FILTERMODE_IDMASK,
        .FilterScale      = CAN_FILTERSCALE_32BIT,
    };
    if (HAL_CAN_ConfigFilter(&hcan2, &f) != HAL_OK)
        Error_Handler();
}

static void CAN3_Filter_Config(void)
{
    CAN_FilterTypeDef f = {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank       = 0,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterIdHigh     = 0x0000,
        .FilterIdLow      = 0x0000,
        .FilterMaskIdHigh = 0x0000,
        .FilterMaskIdLow  = 0x0000,
        .FilterMode       = CAN_FILTERMODE_IDMASK,
        .FilterScale      = CAN_FILTERSCALE_32BIT,
    };
    if (HAL_CAN_ConfigFilter(&hcan3, &f) != HAL_OK)
        Error_Handler();
}

/* --------------- Public API --------------- */

void can_driver_init(void)
{
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

bool can_driver_tx(CAN_HandleTypeDef *hcan, uint32_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {
        .StdId             = std_id,
        .IDE               = CAN_ID_STD,
        .RTR               = CAN_RTR_DATA,
        .DLC               = len,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(hcan, &header, (uint8_t *)data, &mailbox) == HAL_OK;
}

bool can_driver_rx_poll(CAN_HandleTypeDef *hcan, can_msg_t *msg)
{
    if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0)
        return false;

    CAN_RxHeaderTypeDef rx_header;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg->data) != HAL_OK)
        return false;

    msg->id        = rx_header.StdId;
    msg->dlc       = rx_header.DLC;
    msg->timestamp = HAL_GetTick();
    return true;
}

void can_driver_rx_isr(CAN_HandleTypeDef *hcan)
{
    can_queue_t *q;
    if      (hcan->Instance == CAN1) q = &can1_rx_queue;
    else if (hcan->Instance == CAN2) q = &can2_rx_queue;
    else if (hcan->Instance == CAN3) q = &can3_rx_queue;
    else return;

    CAN_RxHeaderTypeDef rx_header;
    can_msg_t msg;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, msg.data) != HAL_OK)
        return;

    msg.id        = rx_header.StdId;
    msg.dlc       = rx_header.DLC;
    msg.timestamp = HAL_GetTick();

    can_queue_push(q, &msg);
}
