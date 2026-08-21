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

#define BUS_OFF_TEST 0

#define CAN_PERIODIC_TX_ENABLE 1
#define CAN_RX_POLL_ENABLE 0
#define CAN_RX_INTERRUPT_ENABLE 1
#define CAN_STM32_TX_ID 0x120U
#define CAN_ESP32_TX_ID 0x321U
#define CAN_ESP32_HEARTBEAT_ID 0x701U
#if BUS_OFF_TEST
#define CAN_TX_PERIOD_MS 10U
#else
#define CAN_TX_PERIOD_MS 1000U
#endif
#define CAN_HEARTBEAT_PERIOD_MS 500U
#define CAN_HEARTBEAT_TIMEOUT_MS (3U * CAN_HEARTBEAT_PERIOD_MS)
#define CAN_RX_QUEUE_WAIT_MS 100U
#define CAN_BUS_OFF_RECOVERY_USE_LEVEL2 1U
#define CAN_BUS_OFF_RECOVERY_KEEP_LEVEL1_DIAGNOSTIC 0U
#define CAN_BUS_OFF_RECOVERY_BACKOFF_MS 1000U
#define CAN_BUS_OFF_RECOVERY_CONFIRM_MS 1000U
#define CAN_BUS_OFF_ABORT_TIMEOUT_MS 10U
#define CAN_BUS_OFF_LEVEL1_MAX_ATTEMPTS 3U
#define CAN_FDCAN_ACTIVE_ITS (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | \
                              FDCAN_IT_TX_COMPLETE | \
                              FDCAN_IT_TX_ABORT_COMPLETE | \
                              FDCAN_IT_TX_FIFO_EMPTY | \
                              FDCAN_IT_ERROR_PASSIVE | \
                              FDCAN_IT_ERROR_WARNING | \
                              FDCAN_IT_BUS_OFF | \
                              FDCAN_IT_ARB_PROTOCOL_ERROR | \
                              FDCAN_IT_DATA_PROTOCOL_ERROR)
#define CAN_FDCAN_TX_BUFFER_MASK (FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2)
#define CAN_RECOVERY_FAIL_NONE 0U
#define CAN_RECOVERY_FAIL_LEVEL1_EXHAUSTED (1U << 0)
#define CAN_RECOVERY_FAIL_ABORT_ERROR (1U << 1)
#define CAN_RECOVERY_FAIL_ABORT_TIMEOUT (1U << 2)
#define CAN_RECOVERY_FAIL_STOP_ERROR (1U << 3)
#define CAN_RECOVERY_FAIL_START_ERROR (1U << 4)
#define CAN_RECOVERY_FAIL_NOTIFICATION_ERROR (1U << 5)
#define CAN_RECOVERY_FAIL_CONTROLLER_CHECK (1U << 6)
#define CAN_RECOVERY_FAIL_LEVEL2_STOP_ERROR (1U << 7)
#define CAN_RECOVERY_FAIL_LEVEL2_DEINIT_ERROR (1U << 8)
#define CAN_RECOVERY_FAIL_LEVEL2_INIT_NOT_READY (1U << 9)
#define CAN_READY_FAIL_HAL_STATE (1U << 0)
#define CAN_READY_FAIL_CCCR_INIT (1U << 1)
#define CAN_READY_FAIL_PSR_BUS_OFF (1U << 2)
#define CAN_READY_FAIL_ILE_LINE0 (1U << 3)
#define CAN_READY_FAIL_IE_BITS (1U << 4)
#define CAN_READY_FAIL_ILS_MAPPING (1U << 5)
#define CAN_READY_FAIL_TXBRP_PENDING (1U << 6)
#define CAN_READY_FAIL_TX_FIFO_FULL (1U << 7)

static osMessageQueueId_t can_rx_queue_handle;
static uint32_t heartbeat_timed_out_latched;
static volatile CanStatistics_t can_statistics = {
    .last_tx_ret = HAL_OK,
    .rx_queue_put_last_status = osOK,
    .node_state = CAN_NODE_INIT,
    .bus_state = CAN_BUS_OK,
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
static void CAN_App_UpdateBusStatus(uint32_t allow_recovery);
static void CAN_App_SetBusState(CanBusState_t state);
static void CAN_App_SetBusStateFromProtocol(const FDCAN_ProtocolStatusTypeDef *protocol_status);
static void CAN_App_UpdateBusCounters(const FDCAN_ProtocolStatusTypeDef *protocol_status,
                                      const FDCAN_ErrorCountersTypeDef *error_counters);
static void CAN_App_HandleBusOff(uint32_t now, uint32_t allow_recovery);
static void CAN_App_HandleBusRecovering(const FDCAN_ProtocolStatusTypeDef *protocol_status, uint32_t now);
#if (CAN_BUS_OFF_RECOVERY_USE_LEVEL2 == 0U) || (CAN_BUS_OFF_RECOVERY_KEEP_LEVEL1_DIAGNOSTIC != 0U)
static uint32_t CAN_App_TryRecoverBusOffLevel1(uint32_t now);
#endif
static uint32_t CAN_App_TryRecoverBusOffLevel2(uint32_t now);
static uint32_t CAN_App_TryAcquireRecoveryOwnership(void);
static void CAN_App_ReleaseRecoveryOwnership(void);
static uint32_t CAN_App_IsFdcanControllerReady(void);
static void CAN_App_CaptureFdcanSnapshot(volatile CanFdcanSnapshot_t *snapshot);
static uint32_t CAN_App_IsBusinessTxAllowed(void);
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

        CAN_App_UpdateBusStatus(1U);
        if (CAN_App_IsBusinessTxAllowed() == 0U) {
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
                CAN_App_UpdateBusStatus(1U);
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
            CAN_App_UpdateBusStatus(1U);
            CAN_App_SnapshotStatus();
            __NOP();
        } else {
            CAN_App_UpdateHeartbeatStatus();
            CAN_App_UpdateBusStatus(1U);
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

static void CAN_App_UpdateBusStatus(uint32_t allow_recovery)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};
    uint32_t now = osKernelGetTickCount();

    if (can_statistics.recovery_in_progress != 0U) {
        return;
    }

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) != HAL_OK) {
        return;
    }

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) != HAL_OK) {
        return;
    }

    CAN_App_UpdateBusCounters(&protocol_status, &error_counters);

    if (can_statistics.bus_state == CAN_BUS_RECOVERING) {
        if (allow_recovery != 0U) {
            CAN_App_HandleBusRecovering(&protocol_status, now);
        }
        return;
    }

    if (protocol_status.BusOff != 0U) {
        CAN_App_HandleBusOff(now, allow_recovery);
        return;
    }

    CAN_App_SetBusStateFromProtocol(&protocol_status);
}

static void CAN_App_SetBusState(CanBusState_t state)
{
    if (can_statistics.bus_state != state) {
        can_statistics.bus_state = state;
        can_statistics.bus_state_change_count++;
    }
}

static void CAN_App_SetBusStateFromProtocol(const FDCAN_ProtocolStatusTypeDef *protocol_status)
{
    if (protocol_status->BusOff != 0U) {
        CAN_App_SetBusState(CAN_BUS_OFF);
    } else if (protocol_status->ErrorPassive != 0U) {
        CAN_App_SetBusState(CAN_BUS_ERROR_PASSIVE);
    } else if (protocol_status->Warning != 0U) {
        CAN_App_SetBusState(CAN_BUS_WARNING);
    } else {
        CAN_App_SetBusState(CAN_BUS_OK);
    }
}

static void CAN_App_UpdateBusCounters(const FDCAN_ProtocolStatusTypeDef *protocol_status,
                                      const FDCAN_ErrorCountersTypeDef *error_counters)
{
    can_statistics.fdcan_last_error_code = protocol_status->LastErrorCode;
    can_statistics.fdcan_data_last_error_code = protocol_status->DataLastErrorCode;
    can_statistics.fdcan_activity = protocol_status->Activity;
    can_statistics.fdcan_bus_off = protocol_status->BusOff;
    can_statistics.fdcan_error_passive = protocol_status->ErrorPassive;
    can_statistics.fdcan_warning = protocol_status->Warning;
    can_statistics.fdcan_rx_esi_flag = protocol_status->RxESIflag;

    can_statistics.fdcan_tx_error_count = error_counters->TxErrorCnt;
    can_statistics.fdcan_rx_error_count = error_counters->RxErrorCnt;
    can_statistics.tx_error_count = error_counters->TxErrorCnt;
    can_statistics.rx_error_count = error_counters->RxErrorCnt;
}

static void CAN_App_HandleBusOff(uint32_t now, uint32_t allow_recovery)
{
    if (can_statistics.bus_state != CAN_BUS_OFF) {
        can_statistics.bus_off_count++;
        can_statistics.last_bus_off_tick = now;
        can_statistics.recovery_level1_attempt_in_event = 0U;
        can_statistics.recovery_level1_exhausted = 0U;
        can_statistics.recovery_controller_ready = 0U;
        CAN_App_SetBusState(CAN_BUS_OFF);
        return;
    }

    if (allow_recovery == 0U) {
        return;
    }

    if ((now - can_statistics.last_bus_off_tick) < CAN_BUS_OFF_RECOVERY_BACKOFF_MS) {
        return;
    }

    if (CAN_App_TryAcquireRecoveryOwnership() == 0U) {
        return;
    }

    can_statistics.bus_off_recovery_attempt_count++;
    can_statistics.last_recovery_tick = now;
    CAN_App_SetBusState(CAN_BUS_RECOVERING);

#if CAN_BUS_OFF_RECOVERY_USE_LEVEL2
    if (CAN_App_TryRecoverBusOffLevel2(now) != 0U) {
        can_statistics.bus_off_recovery_success_count++;
        CAN_App_SetBusState(CAN_BUS_OK);
        CAN_App_ReleaseRecoveryOwnership();
        return;
    }

    can_statistics.bus_off_recovery_fail_count++;
    can_statistics.last_bus_off_tick = now;
    CAN_App_SetBusState(CAN_BUS_OFF);
    CAN_App_ReleaseRecoveryOwnership();
    return;
#else
    if (can_statistics.recovery_level1_attempt_in_event >= CAN_BUS_OFF_LEVEL1_MAX_ATTEMPTS) {
        can_statistics.recovery_level1_exhausted = 1U;
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_LEVEL1_EXHAUSTED;
        CAN_App_ReleaseRecoveryOwnership();
        return;
    }

    can_statistics.recovery_level1_attempt_in_event++;

    if (CAN_App_TryRecoverBusOffLevel1(now) != 0U) {
        FDCAN_ProtocolStatusTypeDef protocol_status = {0};
        FDCAN_ErrorCountersTypeDef error_counters = {0};

        can_statistics.bus_off_recovery_success_count++;
        can_statistics.recovery_level1_attempt_in_event = 0U;
        can_statistics.recovery_level1_exhausted = 0U;

        if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
            if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
                CAN_App_UpdateBusCounters(&protocol_status, &error_counters);
            }
            CAN_App_SetBusStateFromProtocol(&protocol_status);
        } else {
            CAN_App_SetBusState(CAN_BUS_OK);
        }
        CAN_App_ReleaseRecoveryOwnership();
        return;
    }

    can_statistics.bus_off_recovery_fail_count++;
    can_statistics.last_bus_off_tick = now;
    CAN_App_SetBusState(CAN_BUS_OFF);
    CAN_App_ReleaseRecoveryOwnership();
#endif
}

static void CAN_App_HandleBusRecovering(const FDCAN_ProtocolStatusTypeDef *protocol_status, uint32_t now)
{
    if (CAN_App_IsFdcanControllerReady() != 0U) {
        can_statistics.bus_off_recovery_success_count++;
        can_statistics.recovery_level1_attempt_in_event = 0U;
        can_statistics.recovery_level1_exhausted = 0U;
        CAN_App_SetBusStateFromProtocol(protocol_status);
        return;
    }

    if ((now - can_statistics.last_recovery_tick) >= CAN_BUS_OFF_RECOVERY_CONFIRM_MS) {
        can_statistics.bus_off_recovery_fail_count++;
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_CONTROLLER_CHECK;
        can_statistics.last_bus_off_tick = now;
        CAN_App_SetBusState(CAN_BUS_OFF);
    }
}

#if (CAN_BUS_OFF_RECOVERY_USE_LEVEL2 == 0U) || (CAN_BUS_OFF_RECOVERY_KEEP_LEVEL1_DIAGNOSTIC != 0U)
static uint32_t CAN_App_TryRecoverBusOffLevel1(uint32_t now)
{
    uint32_t pending_tx;
    uint32_t abort_mask;
    uint32_t wait_start;
    uint32_t ir_pending;
    (void)now;

    can_statistics.recovery_fail_reason = CAN_RECOVERY_FAIL_NONE;
    can_statistics.recovery_abort_ret = HAL_OK;
    can_statistics.recovery_stop_ret = HAL_OK;
    can_statistics.recovery_start_ret = HAL_OK;
    can_statistics.recovery_notification_ret = HAL_OK;
    can_statistics.recovery_ir_clear_mask = 0U;
    can_statistics.recovery_abort_wait_tick_count = 0U;
    can_statistics.recovery_controller_ready = 0U;
    can_statistics.recovery_ready_fail_mask = 0U;

    CAN_App_CaptureFdcanSnapshot(&can_statistics.recovery_before_snapshot);

    pending_tx = hfdcan1.Instance->TXBRP & CAN_FDCAN_TX_BUFFER_MASK;
    can_statistics.recovery_txbrp_before_abort = pending_tx;
    if (pending_tx != 0U) {
        for (abort_mask = FDCAN_TX_BUFFER0; abort_mask <= FDCAN_TX_BUFFER2; abort_mask <<= 1U) {
            if ((pending_tx & abort_mask) != 0U) {
                can_statistics.recovery_abort_ret = HAL_FDCAN_AbortTxRequest(&hfdcan1, abort_mask);
                if (can_statistics.recovery_abort_ret != HAL_OK) {
                    can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_ABORT_ERROR;
                }
            }
        }

        if ((can_statistics.recovery_fail_reason & CAN_RECOVERY_FAIL_ABORT_ERROR) == 0U) {
            wait_start = osKernelGetTickCount();
            while ((hfdcan1.Instance->TXBRP & CAN_FDCAN_TX_BUFFER_MASK) != 0U) {
                if ((osKernelGetTickCount() - wait_start) >= CAN_BUS_OFF_ABORT_TIMEOUT_MS) {
                    can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_ABORT_TIMEOUT;
                    break;
                }
                osDelay(1U);
            }
            can_statistics.recovery_abort_wait_tick_count = osKernelGetTickCount() - wait_start;
        }
    }
    can_statistics.recovery_txbrp_after_abort = hfdcan1.Instance->TXBRP & CAN_FDCAN_TX_BUFFER_MASK;

    can_statistics.recovery_stop_ret = HAL_FDCAN_Stop(&hfdcan1);
    if (can_statistics.recovery_stop_ret != HAL_OK) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_STOP_ERROR;
    }

    ir_pending = hfdcan1.Instance->IR & FDCAN_IR_MASK;
    can_statistics.recovery_ir_clear_mask = ir_pending;
    if (ir_pending != 0U) {
        hfdcan1.Instance->IR = ir_pending;
    }

    can_statistics.recovery_start_ret = HAL_FDCAN_Start(&hfdcan1);
    if (can_statistics.recovery_start_ret != HAL_OK) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_START_ERROR;
    }

#if CAN_RX_INTERRUPT_ENABLE
    can_statistics.recovery_notification_ret = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                                                              CAN_FDCAN_ACTIVE_ITS,
                                                                              CAN_FDCAN_TX_BUFFER_MASK);
    if (can_statistics.recovery_notification_ret != HAL_OK) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_NOTIFICATION_ERROR;
    }
#else
    can_statistics.recovery_notification_ret = HAL_OK;
#endif

    CAN_App_CaptureFdcanSnapshot(&can_statistics.recovery_after_snapshot);
    can_statistics.recovery_controller_ready = CAN_App_IsFdcanControllerReady();
    if (can_statistics.recovery_controller_ready == 0U) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_CONTROLLER_CHECK;
    }

    return (can_statistics.recovery_fail_reason == CAN_RECOVERY_FAIL_NONE) ? 1U : 0U;
}
#endif

static uint32_t CAN_App_TryRecoverBusOffLevel2(uint32_t now)
{
    (void)now;

    can_statistics.recovery_fail_reason = CAN_RECOVERY_FAIL_NONE;
    can_statistics.recovery_controller_ready = 0U;
    can_statistics.recovery_ready_fail_mask = 0U;
    can_statistics.recovery_level2_attempt_count++;
    can_statistics.level2_stop_ret = HAL_OK;
    can_statistics.level2_deinit_ret = HAL_OK;
    can_statistics.level2_init_ret = HAL_ERROR;
    can_statistics.level2_init_ready = 0U;
    can_statistics.level2_init_hal_state = 0U;
    can_statistics.level2_init_error_code = HAL_FDCAN_ERROR_NONE;
    can_statistics.level2_start_ret = HAL_ERROR;
    can_statistics.level2_reset_apb1hrstr_before = 0U;
    can_statistics.level2_reset_apb1hrstr_after_force = 0U;
    can_statistics.level2_reset_apb1hrstr_after_release = 0U;

    CAN_App_CaptureFdcanSnapshot(&can_statistics.recovery_before_snapshot);
    CAN_App_CaptureFdcanSnapshot(&can_statistics.level2_before_peripheral_reset_snapshot);

    if (can_rx_queue_handle != NULL) {
        can_statistics.recovery_rx_queue_reset_status = osMessageQueueReset(can_rx_queue_handle);
        if (can_statistics.recovery_rx_queue_reset_status == osOK) {
            can_statistics.recovery_rx_queue_reset_count++;
        }
    }

    can_statistics.level2_stop_ret = HAL_FDCAN_Stop(&hfdcan1);
    can_statistics.recovery_stop_ret = can_statistics.level2_stop_ret;
    if (can_statistics.level2_stop_ret != HAL_OK) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_LEVEL2_STOP_ERROR;
    }

    can_statistics.level2_deinit_ret = HAL_FDCAN_DeInit(&hfdcan1);
    if (can_statistics.level2_deinit_ret != HAL_OK) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_LEVEL2_DEINIT_ERROR;
    }

    can_statistics.level2_reset_apb1hrstr_before = RCC->APB1HRSTR;
    __HAL_RCC_FDCAN_FORCE_RESET();
    __DSB();
    can_statistics.level2_reset_apb1hrstr_after_force = RCC->APB1HRSTR;

    __HAL_RCC_FDCAN_RELEASE_RESET();
    __DSB();
    can_statistics.level2_reset_apb1hrstr_after_release = RCC->APB1HRSTR;
    can_statistics.level2_force_reset_count++;

    can_statistics.fdcan_started = 0U;
    MX_FDCAN1_Init();
    CAN_App_CaptureFdcanSnapshot(&can_statistics.level2_after_mx_init_snapshot);

    can_statistics.level2_init_hal_state = (uint32_t)HAL_FDCAN_GetState(&hfdcan1);
    can_statistics.level2_init_error_code = HAL_FDCAN_GetError(&hfdcan1);
    if (HAL_FDCAN_GetState(&hfdcan1) == HAL_FDCAN_STATE_READY) {
        can_statistics.level2_init_ready = 1U;
        can_statistics.level2_init_ret = HAL_OK;
    } else {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_LEVEL2_INIT_NOT_READY;
    }

    CAN_App_Start();
    CAN_App_CaptureFdcanSnapshot(&can_statistics.level2_after_can_app_start_snapshot);
    can_statistics.level2_start_ret = can_statistics.fdcan_start_ret;
    can_statistics.recovery_start_ret = can_statistics.fdcan_start_ret;
    can_statistics.recovery_notification_ret = can_statistics.fdcan_notification_ret;

    if ((can_statistics.fdcan_filter_ret != HAL_OK) ||
        (can_statistics.fdcan_global_filter_ret != HAL_OK) ||
        (can_statistics.fdcan_interrupt_line_ret != HAL_OK) ||
        (can_statistics.fdcan_start_ret != HAL_OK)) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_START_ERROR;
    }

    if (can_statistics.fdcan_notification_ret != HAL_OK) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_NOTIFICATION_ERROR;
    }

    CAN_App_CaptureFdcanSnapshot(&can_statistics.recovery_after_snapshot);
    can_statistics.recovery_controller_ready = CAN_App_IsFdcanControllerReady();
    if (can_statistics.recovery_controller_ready == 0U) {
        can_statistics.recovery_fail_reason |= CAN_RECOVERY_FAIL_CONTROLLER_CHECK;
    }

    if (can_statistics.recovery_fail_reason == CAN_RECOVERY_FAIL_NONE) {
        can_statistics.recovery_level2_success_count++;
        return 1U;
    }

    can_statistics.recovery_level2_fail_count++;
    return 0U;
}

static uint32_t CAN_App_TryAcquireRecoveryOwnership(void)
{
    uint32_t primask;
    uint32_t acquired = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (can_statistics.recovery_in_progress == 0U) {
        can_statistics.recovery_in_progress = 1U;
        can_statistics.recovery_owner_take_count++;
        acquired = 1U;
    } else {
        can_statistics.recovery_owner_busy_count++;
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return acquired;
}

static void CAN_App_ReleaseRecoveryOwnership(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    can_statistics.recovery_in_progress = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

static uint32_t CAN_App_IsFdcanControllerReady(void)
{
    uint32_t required_ile = FDCAN_INTERRUPT_LINE0;
    uint32_t tx_fifo_not_full = ((hfdcan1.Instance->TXFQS & FDCAN_TXFQS_TFQF) == 0U) ? 1U : 0U;
    uint32_t fail_mask = 0U;

    if (HAL_FDCAN_GetState(&hfdcan1) != HAL_FDCAN_STATE_BUSY) {
        fail_mask |= CAN_READY_FAIL_HAL_STATE;
    }

    if ((hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) != 0U) {
        fail_mask |= CAN_READY_FAIL_CCCR_INIT;
    }

    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U) {
        fail_mask |= CAN_READY_FAIL_PSR_BUS_OFF;
    }

    if ((hfdcan1.Instance->ILE & required_ile) != required_ile) {
        fail_mask |= CAN_READY_FAIL_ILE_LINE0;
    }

    if ((hfdcan1.Instance->IE & CAN_FDCAN_ACTIVE_ITS) != CAN_FDCAN_ACTIVE_ITS) {
        fail_mask |= CAN_READY_FAIL_IE_BITS;
    }

    if ((hfdcan1.Instance->ILS & CAN_FDCAN_ACTIVE_ITS) != 0U) {
        fail_mask |= CAN_READY_FAIL_ILS_MAPPING;
    }

    if ((hfdcan1.Instance->TXBRP & CAN_FDCAN_TX_BUFFER_MASK) != 0U) {
        fail_mask |= CAN_READY_FAIL_TXBRP_PENDING;
    }

    if (tx_fifo_not_full == 0U) {
        fail_mask |= CAN_READY_FAIL_TX_FIFO_FULL;
    }

    can_statistics.recovery_ready_fail_mask = fail_mask;
    return (fail_mask == 0U) ? 1U : 0U;
}

static void CAN_App_CaptureFdcanSnapshot(volatile CanFdcanSnapshot_t *snapshot)
{
    snapshot->hal_state = (uint32_t)HAL_FDCAN_GetState(&hfdcan1);
    snapshot->cccr = hfdcan1.Instance->CCCR;
    snapshot->psr = hfdcan1.Instance->PSR;
    snapshot->ecr = hfdcan1.Instance->ECR;
    snapshot->ie = hfdcan1.Instance->IE;
    snapshot->ile = hfdcan1.Instance->ILE;
    snapshot->ils = hfdcan1.Instance->ILS;
    snapshot->ir = hfdcan1.Instance->IR;
    snapshot->rxf0s = hfdcan1.Instance->RXF0S;
    snapshot->txfqs = hfdcan1.Instance->TXFQS;
    snapshot->txbrp = hfdcan1.Instance->TXBRP;
}

static uint32_t CAN_App_IsBusinessTxAllowed(void)
{
    return ((can_statistics.bus_state != CAN_BUS_OFF) &&
            (can_statistics.bus_state != CAN_BUS_RECOVERING)) ? 1U : 0U;
}

static void CAN_App_SnapshotStatus(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};
    uint32_t nominal_total_tq = 1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2;

    if (can_statistics.recovery_in_progress != 0U) {
        can_statistics.fdcan_it0_irq_count = fdcan_it0_irq_count;
        can_statistics.fdcan_it1_irq_count = fdcan_it1_irq_count;
        if (can_rx_queue_handle != NULL) {
            can_statistics.rx_queue_count = osMessageQueueGetCount(can_rx_queue_handle);
            can_statistics.rx_queue_space = osMessageQueueGetSpace(can_rx_queue_handle);
        }
        return;
    }

    can_statistics.can_periodic_tx_runtime = CAN_PERIODIC_TX_ENABLE;
    can_statistics.can_rx_interrupt_runtime = CAN_RX_INTERRUPT_ENABLE;
    can_statistics.tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    can_statistics.fdcan_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    can_statistics.fdcan_it0_irq_count = fdcan_it0_irq_count;
    can_statistics.fdcan_it1_irq_count = fdcan_it1_irq_count;
    CAN_App_CaptureFdcanSnapshot(&can_statistics.fdcan_snapshot);

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
        if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
            CAN_App_UpdateBusCounters(&protocol_status, &error_counters);
        }
    }
}

static void CAN_App_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};

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
                                                                             CAN_FDCAN_ACTIVE_ITS,
                                                                             FDCAN_INTERRUPT_LINE0);
    can_statistics.fdcan_notification_ret = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                                                           CAN_FDCAN_ACTIVE_ITS,
                                                                           CAN_FDCAN_TX_BUFFER_MASK);
#else
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
    CAN_App_UpdateBusStatus(0U);
    CAN_App_SnapshotStatus();
    (void)ErrorStatusITs;
}
