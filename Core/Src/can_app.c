/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_app.c
  * @brief   CAN application layer for STM32 FDCAN + FreeRTOS.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "can_app.h"
#include "fdcan.h"

#define CAN_PERIODIC_TX_ENABLE 1
#define CAN_RX_POLL_ENABLE 0
#define CAN_RX_INTERRUPT_ENABLE 1
#define CAN_STM32_TX_ID 0x120U
#define CAN_ESP32_TX_ID 0x321U
#define CAN_ESP32_HEARTBEAT_ID 0x701U
#define CAN_TX_PERIOD_MS 1000U
#define CAN_HEARTBEAT_PERIOD_MS 500U
#define CAN_HEARTBEAT_TIMEOUT_MS (3U * CAN_HEARTBEAT_PERIOD_MS)
#define CAN_RX_QUEUE_WAIT_MS 100U

static osMessageQueueId_t can_rx_queue_handle;
static uint32_t heartbeat_timed_out_latched;
static volatile CanStatistics_t can_statistics = {
    .last_tx_ret = HAL_OK,
    .rx_queue_put_last_status = osOK,
    .node_state = CAN_NODE_INIT,
    .fdcan_start_ret = HAL_OK,
    .fdcan_filter_ret = HAL_OK,
    .fdcan_global_filter_ret = HAL_OK,
    .fdcan_interrupt_line_ret = HAL_OK,
    .fdcan_notification_ret = HAL_OK,
    .firmware_marker = 0x20260818U,
    .can_periodic_tx_runtime = CAN_PERIODIC_TX_ENABLE,
    .can_rx_interrupt_runtime = CAN_RX_INTERRUPT_ENABLE,
};

/* Kept for the existing stm32h7xx_it.c IRQ debug counters in this cleanup pass. */
volatile uint32_t fdcan_it0_irq_count = 0;
volatile uint32_t fdcan_it1_irq_count = 0;

static void CAN_App_Start(void);
static void CAN_App_SnapshotStatus(void);
static void CAN_App_NoteReceivedFrame(const CanFrame_t *frame);
static void CAN_App_HandleHeartbeatReceived(void);
static void CAN_App_UpdateHeartbeatStatus(void);
static void CAN_App_SetNodeState(CanNodeState_t state);
static uint8_t CAN_App_DlcToBytes(uint32_t dlc);

void CAN_App_Init(osMessageQueueId_t rxQueue)
{
    can_rx_queue_handle = rxQueue;
    CAN_App_Start();
}

const CanStatistics_t *CAN_App_GetStatistics(void)
{
    return (const CanStatistics_t *)&can_statistics;
}

void can_tx_task(void *argument)
{
    uint8_t counter = 0;
    (void)argument;

    for (;;) {
        if (CAN_PERIODIC_TX_ENABLE == 0) {
            CAN_App_SnapshotStatus();
            osDelay(CAN_TX_PERIOD_MS);
            continue;
        }

        FDCAN_TxHeaderTypeDef txHeader = {0};
        uint8_t txData[8] = {0};

        txHeader.Identifier = CAN_STM32_TX_ID;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        txData[0] = counter++;
        txData[1] = 0xC1;
        txData[2] = 0xC2;
        txData[3] = 0xC3;
        txData[4] = 0xC4;
        txData[5] = 0xC5;
        txData[6] = 0xC6;
        txData[7] = 0xC7;

        can_statistics.tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

        if (can_statistics.tx_fifo_free_level == 0U) {
            can_statistics.tx_full_skip_count++;
            CAN_App_SnapshotStatus();
            osDelay(CAN_TX_PERIOD_MS);
            continue;
        }

        can_statistics.last_tx_ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
        if (can_statistics.last_tx_ret == HAL_OK) {
            can_statistics.latest_tx_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            can_statistics.tx_ok_count++;
        } else {
            can_statistics.tx_err_count++;
        }

        CAN_App_SnapshotStatus();
        osDelay(CAN_TX_PERIOD_MS);
    }
}

void can_rx_task(void *argument)
{
    CanFrame_t frame;
    (void)argument;

    for (;;) {
#if CAN_RX_POLL_ENABLE
        can_statistics.rx_poll_count++;
        if ((can_statistics.rx_poll_count & 0x3FU) == 0U) {
            CAN_App_SnapshotStatus();
        }

        can_statistics.fdcan_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
        if (can_statistics.fdcan_rx_fifo0_fill > 0U) {
            FDCAN_RxHeaderTypeDef rxHeader = {0};
            frame.id = 0U;
            frame.dlc = 0U;
            for (uint32_t i = 0; i < sizeof(frame.data); i++) {
                frame.data[i] = 0U;
            }

            can_statistics.rx_poll_fifo_seen++;
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, frame.data) == HAL_OK) {
                frame.id = rxHeader.Identifier;
                frame.dlc = CAN_App_DlcToBytes(rxHeader.DataLength);
                can_statistics.rx_task_count++;
                CAN_App_NoteReceivedFrame(&frame);
                CAN_App_UpdateHeartbeatStatus();
                can_statistics.rx_poll_get_ok_count++;
                CAN_App_SnapshotStatus();
            } else {
                can_statistics.rx_poll_get_err_count++;
            }
        }
        osDelay(1);
#else
        if (osMessageQueueGet(can_rx_queue_handle, &frame, NULL, CAN_RX_QUEUE_WAIT_MS) == osOK) {
            can_statistics.rx_task_count++;
            CAN_App_NoteReceivedFrame(&frame);
            CAN_App_UpdateHeartbeatStatus();
            CAN_App_SnapshotStatus();
            __NOP();
        } else {
            CAN_App_UpdateHeartbeatStatus();
        }
#endif
    }
}

static void CAN_App_NoteReceivedFrame(const CanFrame_t *frame)
{
    can_statistics.last_rx_frame = *frame;

    if (frame->id == CAN_ESP32_TX_ID) {
        can_statistics.expected_rx_seen_count++;
        can_statistics.esp32_rx_count++;
        can_statistics.esp32_last_rx_id = frame->id;
        can_statistics.esp32_last_rx_dlc = frame->dlc;
        can_statistics.esp32_last_rx_counter = (frame->dlc > 0U) ? frame->data[0] : 0U;

        for (uint32_t i = 0; i < sizeof(can_statistics.esp32_last_rx_data); i++) {
            can_statistics.esp32_last_rx_data[i] = frame->data[i];
        }
    } else if (frame->id == CAN_ESP32_HEARTBEAT_ID) {
        CAN_App_HandleHeartbeatReceived();
    } else {
        can_statistics.unexpected_rx_seen_count++;
    }
}

static void CAN_App_HandleHeartbeatReceived(void)
{
    can_statistics.heartbeat_rx_count++;
    can_statistics.last_heartbeat_tick = osKernelGetTickCount();
    can_statistics.heartbeat_age_ms = 0U;
    heartbeat_timed_out_latched = 0U;

    switch (can_statistics.node_state) {
    case CAN_NODE_INIT:
        can_statistics.recovery_heartbeat_count = 0U;
        CAN_App_SetNodeState(CAN_NODE_ONLINE);
        break;

    case CAN_NODE_OFFLINE:
        can_statistics.recovery_heartbeat_count = 1U;
        CAN_App_SetNodeState(CAN_NODE_RECOVERING);
        break;

    case CAN_NODE_RECOVERING:
        can_statistics.recovery_heartbeat_count++;
        if (can_statistics.recovery_heartbeat_count >= 3U) {
            can_statistics.recovery_heartbeat_count = 0U;
            CAN_App_SetNodeState(CAN_NODE_ONLINE);
        }
        break;

    case CAN_NODE_ONLINE:
    default:
        can_statistics.recovery_heartbeat_count = 0U;
        CAN_App_SetNodeState(CAN_NODE_ONLINE);
        break;
    }
}

static void CAN_App_UpdateHeartbeatStatus(void)
{
    uint32_t now = osKernelGetTickCount();

    if (can_statistics.last_heartbeat_tick == 0U) {
        can_statistics.heartbeat_age_ms = 0U;
        CAN_App_SetNodeState(CAN_NODE_INIT);
        return;
    }

    can_statistics.heartbeat_age_ms = now - can_statistics.last_heartbeat_tick;
    if (can_statistics.heartbeat_age_ms > CAN_HEARTBEAT_TIMEOUT_MS) {
        if (heartbeat_timed_out_latched == 0U) {
            can_statistics.heartbeat_timeout_count++;
            heartbeat_timed_out_latched = 1U;
        }

        can_statistics.recovery_heartbeat_count = 0U;
        CAN_App_SetNodeState(CAN_NODE_OFFLINE);
    }
}

static void CAN_App_SetNodeState(CanNodeState_t state)
{
    if (can_statistics.node_state != state) {
        can_statistics.node_state = state;
        can_statistics.node_state_change_count++;
    }

    can_statistics.esp32_online = (state == CAN_NODE_ONLINE) ? 1U : 0U;
}

static void CAN_App_SnapshotStatus(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};
    uint32_t nominal_total_tq = 1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2;

    can_statistics.can_periodic_tx_runtime = CAN_PERIODIC_TX_ENABLE;
    can_statistics.can_rx_interrupt_runtime = CAN_RX_INTERRUPT_ENABLE;
    can_statistics.tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    can_statistics.fdcan_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    can_statistics.fdcan_it0_irq_count = fdcan_it0_irq_count;
    can_statistics.fdcan_it1_irq_count = fdcan_it1_irq_count;

    if (can_rx_queue_handle != NULL) {
        can_statistics.rx_queue_count = osMessageQueueGetCount(can_rx_queue_handle);
        can_statistics.rx_queue_space = osMessageQueueGetSpace(can_rx_queue_handle);
    }

    can_statistics.fdcan_error_code = HAL_FDCAN_GetError(&hfdcan1);
    can_statistics.fdcan_init_rx_fifo0_elements = hfdcan1.Init.RxFifo0ElmtsNbr;
    can_statistics.fdcan_kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    if ((can_statistics.fdcan_kernel_clock_hz != 0U) &&
        (hfdcan1.Init.NominalPrescaler != 0U) &&
        (nominal_total_tq != 0U)) {
        can_statistics.fdcan_calculated_bitrate = can_statistics.fdcan_kernel_clock_hz /
                                                  hfdcan1.Init.NominalPrescaler /
                                                  nominal_total_tq;
    } else {
        can_statistics.fdcan_calculated_bitrate = 0U;
    }

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
        can_statistics.fdcan_last_error_code = protocol_status.LastErrorCode;
        can_statistics.fdcan_data_last_error_code = protocol_status.DataLastErrorCode;
        can_statistics.fdcan_activity = protocol_status.Activity;
        can_statistics.fdcan_bus_off = protocol_status.BusOff;
        can_statistics.fdcan_error_passive = protocol_status.ErrorPassive;
        can_statistics.fdcan_warning = protocol_status.Warning;
        can_statistics.fdcan_rx_esi_flag = protocol_status.RxESIflag;
    }

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
        can_statistics.fdcan_tx_error_count = error_counters.TxErrorCnt;
        can_statistics.fdcan_rx_error_count = error_counters.RxErrorCnt;
    }
}

static void CAN_App_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};
    const uint32_t active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                FDCAN_IT_TX_COMPLETE |
                                FDCAN_IT_TX_ABORT_COMPLETE |
                                FDCAN_IT_TX_FIFO_EMPTY |
                                FDCAN_IT_ERROR_PASSIVE |
                                FDCAN_IT_ERROR_WARNING |
                                FDCAN_IT_BUS_OFF |
                                FDCAN_IT_ARB_PROTOCOL_ERROR |
                                FDCAN_IT_DATA_PROTOCOL_ERROR;

    if (can_statistics.fdcan_started != 0U) {
        CAN_App_SnapshotStatus();
        return;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_ESP32_TX_ID;
    filter.FilterID2 = CAN_ESP32_HEARTBEAT_ID;

    can_statistics.fdcan_filter_ret = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    can_statistics.fdcan_global_filter_ret = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                                                          FDCAN_REJECT,
                                                                          FDCAN_REJECT,
                                                                          FDCAN_REJECT_REMOTE,
                                                                          FDCAN_REJECT_REMOTE);

#if CAN_RX_INTERRUPT_ENABLE
    can_statistics.fdcan_interrupt_line_ret = HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                                                             active_its,
                                                                             FDCAN_INTERRUPT_LINE0);
    can_statistics.fdcan_notification_ret = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                                                           active_its,
                                                                           FDCAN_TX_BUFFER0 |
                                                                           FDCAN_TX_BUFFER1 |
                                                                           FDCAN_TX_BUFFER2);
#else
    (void)active_its;
    can_statistics.fdcan_interrupt_line_ret = HAL_OK;
    can_statistics.fdcan_notification_ret = HAL_OK;
#endif

    can_statistics.fdcan_start_ret = HAL_FDCAN_Start(&hfdcan1);
    if (can_statistics.fdcan_start_ret == HAL_OK) {
        can_statistics.fdcan_started = 1U;
    }

    CAN_App_SnapshotStatus();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    can_statistics.rx_fifo0_callback_count++;
    can_statistics.rx_fifo0_its_last = RxFifo0ITs;

    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        FDCAN_RxHeaderTypeDef rxHeader = {0};
        CanFrame_t frame = {0};

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, frame.data) != HAL_OK) {
            can_statistics.rx_get_err_count++;
            break;
        }

        frame.id = rxHeader.Identifier;
        frame.dlc = CAN_App_DlcToBytes(rxHeader.DataLength);
        can_statistics.rx_irq_count++;
        can_statistics.rx_get_ok_count++;
        can_statistics.last_rx_frame = frame;

        if (can_rx_queue_handle != NULL) {
            can_statistics.rx_queue_put_last_status = osMessageQueuePut(can_rx_queue_handle, &frame, 0U, 0U);
            if (can_statistics.rx_queue_put_last_status == osOK) {
                can_statistics.rx_queue_put_ok_count++;
            } else {
                can_statistics.rx_queue_put_err_count++;
            }
        }
    }
}

static uint8_t CAN_App_DlcToBytes(uint32_t dlc)
{
    switch (dlc) {
    case FDCAN_DLC_BYTES_0:  return 0U;
    case FDCAN_DLC_BYTES_1:  return 1U;
    case FDCAN_DLC_BYTES_2:  return 2U;
    case FDCAN_DLC_BYTES_3:  return 3U;
    case FDCAN_DLC_BYTES_4:  return 4U;
    case FDCAN_DLC_BYTES_5:  return 5U;
    case FDCAN_DLC_BYTES_6:  return 6U;
    case FDCAN_DLC_BYTES_7:  return 7U;
    case FDCAN_DLC_BYTES_8:  return 8U;
    default:                 return 8U;
    }
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    can_statistics.last_tx_complete_buffer = BufferIndexes;
    can_statistics.tx_complete_count++;
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    can_statistics.last_tx_abort_buffer = BufferIndexes;
    can_statistics.tx_abort_count++;
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    can_statistics.tx_fifo_empty_count++;
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    can_statistics.fdcan_error_callback_count++;
    can_statistics.fdcan_error_code = HAL_FDCAN_GetError(hfdcan);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    can_statistics.fdcan_error_status_callback_count++;
    CAN_App_SnapshotStatus();
    (void)ErrorStatusITs;
}
