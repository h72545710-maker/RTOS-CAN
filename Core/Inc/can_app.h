/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can_app.h
  * @brief   CAN application layer for STM32 FDCAN + FreeRTOS.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CAN_APP_H__
#define __CAN_APP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame_t;

typedef enum {
    CAN_NODE_INIT = 0,
    CAN_NODE_ONLINE,
    CAN_NODE_OFFLINE,
    CAN_NODE_RECOVERING
} CanNodeState_t;

typedef enum {
    CAN_BUS_OK = 0,
    CAN_BUS_WARNING,
    CAN_BUS_ERROR_PASSIVE,
    CAN_BUS_OFF,
    CAN_BUS_RECOVERING
} CanBusState_t;

typedef struct {
    uint32_t hal_state;
    uint32_t cccr;
    uint32_t psr;
    uint32_t ecr;
    uint32_t ie;
    uint32_t ile;
    uint32_t ils;
    uint32_t ir;
    uint32_t rxf0s;
    uint32_t txfqs;
    uint32_t txbrp;
} CanFdcanSnapshot_t;

typedef struct {
    /* STM32 business TX path. */
    uint32_t tx_ok_count;
    uint32_t tx_err_count;
    uint32_t tx_full_skip_count;
    uint32_t tx_complete_count;
    uint32_t tx_fifo_empty_count;
    uint32_t tx_abort_count;
    uint32_t latest_tx_buffer;
    uint32_t last_tx_complete_buffer;
    uint32_t last_tx_abort_buffer;
    HAL_StatusTypeDef last_tx_ret;

    /* RX FIFO0 callback, CMSIS queue, and RX task path. */
    uint32_t rx_irq_count;
    uint32_t rx_task_count;
    uint32_t rx_fifo0_callback_count;
    uint32_t rx_fifo0_its_last;
    uint32_t rx_get_ok_count;
    uint32_t rx_get_err_count;
    uint32_t rx_queue_put_ok_count;
    uint32_t rx_queue_put_err_count;
    osStatus_t rx_queue_put_last_status;
    uint32_t rx_queue_count;
    uint32_t rx_queue_space;
    uint32_t rx_poll_count;
    uint32_t rx_poll_fifo_seen;
    uint32_t rx_poll_get_ok_count;
    uint32_t rx_poll_get_err_count;
    CanFrame_t last_rx_frame;

    /* ESP32 business frame and heartbeat/node supervision. */
    uint32_t expected_rx_seen_count;
    uint32_t unexpected_rx_seen_count;
    uint32_t esp32_rx_count;
    uint32_t esp32_last_rx_id;
    uint32_t esp32_last_rx_dlc;
    uint32_t esp32_last_rx_counter;
    uint8_t esp32_last_rx_data[8];
    uint32_t heartbeat_rx_count;
    uint32_t last_heartbeat_tick;
    uint32_t heartbeat_age_ms;
    uint32_t heartbeat_timeout_count;
    uint32_t esp32_online;
    CanNodeState_t node_state;
    uint32_t node_state_change_count;
    uint32_t recovery_heartbeat_count;

    /* FDCAN bus state, protocol error counters, and recovery summary. */
    CanBusState_t bus_state;
    uint32_t bus_state_change_count;
    uint32_t bus_off_count;
    uint32_t bus_off_recovery_attempt_count;
    uint32_t bus_off_recovery_success_count;
    uint32_t bus_off_recovery_fail_count;
    uint32_t last_bus_off_tick;
    uint32_t last_recovery_tick;
    uint32_t tx_error_count;
    uint32_t rx_error_count;

    /* FDCAN start/filter/interrupt status and live controller snapshot. */
    HAL_StatusTypeDef fdcan_start_ret;
    HAL_StatusTypeDef fdcan_filter_ret;
    HAL_StatusTypeDef fdcan_global_filter_ret;
    HAL_StatusTypeDef fdcan_interrupt_line_ret;
    HAL_StatusTypeDef fdcan_notification_ret;
    uint32_t fdcan_started;
    uint32_t fdcan_error_code;
    uint32_t fdcan_last_error_code;
    uint32_t fdcan_data_last_error_code;
    uint32_t fdcan_activity;
    uint32_t fdcan_bus_off;
    uint32_t fdcan_error_passive;
    uint32_t fdcan_warning;
    uint32_t fdcan_rx_esi_flag;
    uint32_t fdcan_tx_error_count;
    uint32_t fdcan_rx_error_count;
    uint32_t fdcan_error_callback_count;
    uint32_t fdcan_error_status_callback_count;
    uint32_t fdcan_it0_irq_count;
    uint32_t fdcan_it1_irq_count;
    uint32_t fdcan_rx_fifo0_fill;
    uint32_t fdcan_init_rx_fifo0_elements;
    CanFdcanSnapshot_t fdcan_snapshot;

    /* Bus-Off recovery diagnostics; kept for controlled fault-injection tests. */
    CanFdcanSnapshot_t recovery_before_snapshot;
    CanFdcanSnapshot_t recovery_after_snapshot;
    CanFdcanSnapshot_t level2_before_peripheral_reset_snapshot;
    CanFdcanSnapshot_t level2_after_mx_init_snapshot;
    CanFdcanSnapshot_t level2_after_can_app_start_snapshot;
    HAL_StatusTypeDef recovery_abort_ret;
    HAL_StatusTypeDef recovery_stop_ret;
    HAL_StatusTypeDef recovery_start_ret;
    HAL_StatusTypeDef recovery_notification_ret;
    uint32_t recovery_level1_attempt_in_event;
    uint32_t recovery_level1_exhausted;
    uint32_t recovery_fail_reason;
    uint32_t recovery_ir_clear_mask;
    uint32_t recovery_txbrp_before_abort;
    uint32_t recovery_txbrp_after_abort;
    uint32_t recovery_abort_wait_tick_count;
    uint32_t recovery_controller_ready;
    uint32_t recovery_ready_fail_mask;
    uint32_t recovery_in_progress;
    uint32_t recovery_owner_take_count;
    uint32_t recovery_owner_busy_count;
    osStatus_t recovery_rx_queue_reset_status;
    uint32_t recovery_rx_queue_reset_count;
    uint32_t recovery_level2_attempt_count;
    uint32_t recovery_level2_success_count;
    uint32_t recovery_level2_fail_count;
    HAL_StatusTypeDef level2_stop_ret;
    HAL_StatusTypeDef level2_deinit_ret;
    HAL_StatusTypeDef level2_init_ret;
    uint32_t level2_init_ready;
    uint32_t level2_init_hal_state;
    uint32_t level2_init_error_code;
    HAL_StatusTypeDef level2_start_ret;
    uint32_t level2_force_reset_count;
    uint32_t level2_reset_apb1hrstr_before;
    uint32_t level2_reset_apb1hrstr_after_force;
    uint32_t level2_reset_apb1hrstr_after_release;
    uint32_t tx_fifo_free_level;
    uint32_t fdcan_kernel_clock_hz;
    uint32_t fdcan_calculated_bitrate;
    uint32_t firmware_marker;
    uint32_t can_periodic_tx_runtime;
    uint32_t can_rx_interrupt_runtime;
} CanStatistics_t;

void CAN_App_Init(osMessageQueueId_t rxQueue);
const CanStatistics_t *CAN_App_GetStatistics(void);

void can_tx_task(void *argument);
void can_rx_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_APP_H__ */
