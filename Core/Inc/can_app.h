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

typedef struct {
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

    uint32_t expected_rx_seen_count;
    uint32_t unexpected_rx_seen_count;
    uint32_t esp32_rx_count;
    uint32_t esp32_last_rx_id;
    uint32_t esp32_last_rx_dlc;
    uint32_t esp32_last_rx_counter;
    uint8_t esp32_last_rx_data[8];

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
