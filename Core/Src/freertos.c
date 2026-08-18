/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fdcan.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_PERIODIC_TX_ENABLE 1
#define CAN_RX_POLL_ENABLE 0
#define CAN_RX_INTERRUPT_ENABLE 1
#define CAN_STM32_TX_ID 0x120U
#define CAN_ESP32_TX_ID 0x321U
#define CAN_TX_PERIOD_MS 1000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile uint32_t tx_ok_count = 0;
volatile uint32_t tx_err_count = 0;
volatile uint32_t rx_irq_count = 0;
volatile uint32_t rx_task_count = 0;
volatile HAL_StatusTypeDef last_tx_ret = HAL_OK;
volatile CanFrame_t last_rx_frame;
volatile HAL_StatusTypeDef fdcan_start_ret = HAL_OK;
volatile uint32_t fdcan_error_code = 0;
volatile uint32_t tx_fifo_free_level = 0;
volatile uint32_t fdcan_bus_off = 0;
volatile uint32_t fdcan_error_passive = 0;
volatile uint32_t fdcan_warning = 0;
volatile uint32_t fdcan_last_error_code = 0;
volatile uint32_t fdcan_data_last_error_code = 0;
volatile uint32_t fdcan_activity = 0;
volatile uint32_t fdcan_rx_esi_flag = 0;
volatile uint32_t fdcan_tx_error_count = 0;
volatile uint32_t fdcan_rx_error_count = 0;
volatile uint32_t tx_full_skip_count = 0;
volatile uint32_t tx_complete_count = 0;
volatile uint32_t tx_fifo_empty_count = 0;
volatile uint32_t tx_abort_count = 0;
volatile uint32_t fdcan_error_callback_count = 0;
volatile uint32_t fdcan_error_status_callback_count = 0;
volatile uint32_t latest_tx_buffer = 0;
volatile uint32_t last_tx_complete_buffer = 0;
volatile uint32_t last_tx_abort_buffer = 0;
volatile uint32_t fdcan_rx_fifo0_fill = 0;
volatile uint32_t fdcan_cccr = 0;
volatile uint32_t fdcan_txfqs = 0;
volatile uint32_t fdcan_rxf0s = 0;
volatile uint32_t fdcan_ir = 0;
volatile uint32_t fdcan_ie = 0;
volatile uint32_t fdcan_ils = 0;
volatile uint32_t fdcan_ile = 0;
volatile uint32_t fdcan_txbrp = 0;
volatile uint32_t fdcan_txbto = 0;
volatile uint32_t fdcan_txbc = 0;
volatile uint32_t fdcan_txbcr = 0;
volatile uint32_t fdcan_txbcf = 0;
volatile uint32_t fdcan_txbtie = 0;
volatile uint32_t fdcan_txbcie = 0;
volatile uint32_t fdcan_test = 0;
volatile uint32_t fdcan_psr = 0;
volatile uint32_t fdcan_it0_irq_count = 0;
volatile uint32_t fdcan_it1_irq_count = 0;
volatile uint32_t rx_fifo0_callback_count = 0;
volatile uint32_t rx_fifo0_its_last = 0;
volatile uint32_t rx_get_ok_count = 0;
volatile uint32_t rx_get_err_count = 0;
volatile uint32_t rx_queue_put_ok_count = 0;
volatile uint32_t rx_queue_put_err_count = 0;
volatile osStatus_t rx_queue_put_last_status = osOK;
volatile uint32_t expected_rx_seen_count = 0;
volatile uint32_t unexpected_rx_seen_count = 0;
volatile uint32_t esp32_rx_count = 0;
volatile uint32_t esp32_last_rx_id = 0;
volatile uint32_t esp32_last_rx_dlc = 0;
volatile uint32_t esp32_last_rx_counter = 0;
volatile uint8_t esp32_last_rx_data[8] = {0};
volatile uint32_t rx_queue_count = 0;
volatile uint32_t rx_queue_space = 0;
volatile uint32_t rx_poll_count = 0;
volatile uint32_t rx_poll_fifo_seen = 0;
volatile uint32_t rx_poll_get_ok_count = 0;
volatile uint32_t rx_poll_get_err_count = 0;
volatile uint32_t fdcan_init_rx_fifo0_elements = 0;
volatile uint32_t fdcan_rx_pin_level = 0;
volatile uint32_t fdcan_rx_pin_high_seen = 0;
volatile uint32_t fdcan_rx_pin_low_seen = 0;
volatile uint32_t fdcan_rx_pin_edge_count = 0;
volatile uint32_t fdcan_rx_pin_sample_count = 0;
volatile uint32_t fdcan_tx_pin_level = 0;
volatile uint32_t fdcan_tx_pin_high_seen = 0;
volatile uint32_t fdcan_tx_pin_low_seen = 0;
volatile uint32_t fdcan_tx_pin_edge_count = 0;
volatile uint32_t fdcan_tx_pin_sample_count = 0;
volatile uint32_t fdcan_rx_probe_high_count = 0;
volatile uint32_t fdcan_rx_probe_low_count = 0;
volatile uint32_t fdcan_rx_probe_edge_count = 0;
volatile uint32_t fdcan_rx_probe_dominant_seen = 0;
volatile uint32_t fdcan_rx_probe_idle_high = 0;
volatile uint32_t fdcan_tx_probe_high_count = 0;
volatile uint32_t fdcan_tx_probe_low_count = 0;
volatile uint32_t fdcan_tx_probe_edge_count = 0;
volatile uint32_t fdcan_tx_probe_dominant_seen = 0;
volatile uint32_t fdcan_tx_probe_idle_high = 0;
volatile uint32_t fdcan_started = 0;
volatile HAL_StatusTypeDef fdcan_filter_ret = HAL_OK;
volatile HAL_StatusTypeDef fdcan_global_filter_ret = HAL_OK;
volatile HAL_StatusTypeDef fdcan_interrupt_line_ret = HAL_OK;
volatile HAL_StatusTypeDef fdcan_notification_ret = HAL_OK;
volatile uint32_t firmware_marker = 0x20260818U;
volatile uint32_t can_periodic_tx_runtime = CAN_PERIODIC_TX_ENABLE;
volatile uint32_t can_rx_interrupt_runtime = CAN_RX_INTERRUPT_ENABLE;
volatile uint32_t fdcan_filter_id1 = 0;
volatile uint32_t fdcan_filter_id2 = 0;
volatile uint32_t fdcan_rxf0s_fill = 0;
volatile uint32_t fdcan_mode_runtime = 0;
volatile uint32_t fdcan_cccr_init = 0;
volatile uint32_t fdcan_cccr_cce = 0;
volatile uint32_t fdcan_cccr_asm = 0;
volatile uint32_t fdcan_cccr_mon = 0;
volatile uint32_t fdcan_cccr_test = 0;
volatile uint32_t fdcan_test_lbck = 0;
volatile uint32_t fdcan_test_rx = 0;
volatile uint32_t fdcan_test_tx = 0;
volatile uint32_t fdcan_txfqs_free = 0;
volatile uint32_t fdcan_txfqs_get_index = 0;
volatile uint32_t fdcan_txfqs_put_index = 0;
volatile uint32_t fdcan_txfqs_full = 0;
volatile uint32_t fdcan_tx_pending_count = 0;
volatile uint32_t fdcan_tx_completed_count = 0;
volatile uint32_t fdcan_psr_lec = 0;
volatile uint32_t fdcan_psr_activity = 0;
volatile uint32_t fdcan_psr_error_passive = 0;
volatile uint32_t fdcan_psr_warning = 0;
volatile uint32_t fdcan_psr_bus_off = 0;
volatile uint32_t fdcan_psr_dlec = 0;
volatile uint32_t fdcan_kernel_clock_hz = 0;
volatile uint32_t fdcan_clock_source = 0;
volatile uint32_t fdcan_nominal_total_tq = 0;
volatile uint32_t fdcan_calculated_bitrate = 0;
volatile uint32_t fdcan_nominal_prescaler_runtime = 0;
volatile uint32_t fdcan_nominal_seg1_runtime = 0;
volatile uint32_t fdcan_nominal_seg2_runtime = 0;
volatile uint32_t fdcan_nominal_sjw_runtime = 0;
volatile uint32_t fdcan_message_ram_std_filter_sa = 0;
volatile uint32_t fdcan_message_ram_rxfifo0_sa = 0;
volatile uint32_t fdcan_message_ram_txfifoq_sa = 0;
volatile uint32_t fdcan_message_ram_end_sa = 0;
volatile uint32_t fdcan_std_filter_word0 = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canTxTask */
osThreadId_t canTxTaskHandle;
const osThreadAttr_t canTxTask_attributes = {
  .name = "canTxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void fdcan_app_start(void);
static void fdcan_snapshot_status(void);
static void fdcan_probe_rx_pin(void);
static void fdcan_note_received_frame(const CanFrame_t *frame);
static uint8_t fdcan_dlc_to_bytes(uint32_t dlc);
void can_tx_task(void *argument);
void can_rx_task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void can_tx_task(void *argument);
void can_rx_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of canRxQueue */
   canRxQueueHandle = osMessageQueueNew (16, sizeof(CanFrame_t), &canRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  fdcan_app_start();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of canTxTask */
  canTxTaskHandle = osThreadNew(can_tx_task, NULL, &canTxTask_attributes);

  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(can_rx_task, NULL, &canRxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_can_tx_task */
/**
* @brief Function implementing the canTxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_tx_task */
void can_tx_task(void *argument)
{
  /* USER CODE BEGIN can_tx_task */
	uint8_t counter = 0;
  /* Infinite loop */
  for(;;)
  {
        if (CAN_PERIODIC_TX_ENABLE == 0) {
            fdcan_snapshot_status();
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
				
        tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

        if (tx_fifo_free_level == 0U) {
            tx_full_skip_count++;
            fdcan_snapshot_status();
            osDelay(CAN_TX_PERIOD_MS);
            continue;
        }

        last_tx_ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);
        if (last_tx_ret == HAL_OK) {
            latest_tx_buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
            tx_ok_count++;
        } else {
            tx_err_count++;
        }
        fdcan_snapshot_status();
    osDelay(CAN_TX_PERIOD_MS);
  }
  /* USER CODE END can_tx_task */
}

/* USER CODE BEGIN Header_can_rx_task */
/**
* @brief Function implementing the canRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_can_rx_task */
void can_rx_task(void *argument)
{
  /* USER CODE BEGIN can_rx_task */
	CanFrame_t frame;
  /* Infinite loop */
  for(;;)
  {
#if CAN_RX_POLL_ENABLE
    rx_poll_count++;
    fdcan_probe_rx_pin();
    if ((rx_poll_count & 0x3FU) == 0U) {
        fdcan_snapshot_status();
    }
    fdcan_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    if (fdcan_rx_fifo0_fill > 0U) {
        FDCAN_RxHeaderTypeDef rxHeader = {0};
        frame.id = 0U;
        frame.dlc = 0U;
        for (uint32_t i = 0; i < sizeof(frame.data); i++) {
            frame.data[i] = 0U;
        }

        rx_poll_fifo_seen++;
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, frame.data) == HAL_OK) {
            frame.id = rxHeader.Identifier;
            frame.dlc = fdcan_dlc_to_bytes(rxHeader.DataLength);
            rx_task_count++;
            fdcan_note_received_frame(&frame);
            rx_poll_get_ok_count++;
            fdcan_snapshot_status();
        } else {
            rx_poll_get_err_count++;
        }
    }
    osDelay(1);
#else
    if (osMessageQueueGet(canRxQueueHandle, &frame, NULL, osWaitForever) == osOK) {
            rx_task_count++;
            fdcan_note_received_frame(&frame);
            fdcan_snapshot_status();
            __NOP();
        }
#endif
  }
  /* USER CODE END can_rx_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void fdcan_note_received_frame(const CanFrame_t *frame)
{
    last_rx_frame = *frame;

    if (frame->id == CAN_ESP32_TX_ID) {
        expected_rx_seen_count++;
        esp32_rx_count++;
        esp32_last_rx_id = frame->id;
        esp32_last_rx_dlc = frame->dlc;
        esp32_last_rx_counter = (frame->dlc > 0U) ? frame->data[0] : 0U;

        for (uint32_t i = 0; i < sizeof(esp32_last_rx_data); i++) {
            esp32_last_rx_data[i] = frame->data[i];
        }
    } else {
        unexpected_rx_seen_count++;
    }
}

static void fdcan_snapshot_status(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};

    can_periodic_tx_runtime = CAN_PERIODIC_TX_ENABLE;
    can_rx_interrupt_runtime = CAN_RX_INTERRUPT_ENABLE;
    tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    fdcan_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    if (canRxQueueHandle != NULL) {
        rx_queue_count = osMessageQueueGetCount(canRxQueueHandle);
        rx_queue_space = osMessageQueueGetSpace(canRxQueueHandle);
    }
    fdcan_error_code = HAL_FDCAN_GetError(&hfdcan1);
    fdcan_init_rx_fifo0_elements = hfdcan1.Init.RxFifo0ElmtsNbr;
    fdcan_kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    fdcan_clock_source = __HAL_RCC_GET_FDCAN_SOURCE();
    fdcan_nominal_prescaler_runtime = hfdcan1.Init.NominalPrescaler;
    fdcan_nominal_seg1_runtime = hfdcan1.Init.NominalTimeSeg1;
    fdcan_nominal_seg2_runtime = hfdcan1.Init.NominalTimeSeg2;
    fdcan_nominal_sjw_runtime = hfdcan1.Init.NominalSyncJumpWidth;
    fdcan_nominal_total_tq = 1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2;
    if ((fdcan_kernel_clock_hz != 0U) &&
        (hfdcan1.Init.NominalPrescaler != 0U) &&
        (fdcan_nominal_total_tq != 0U)) {
        fdcan_calculated_bitrate = fdcan_kernel_clock_hz /
                                   hfdcan1.Init.NominalPrescaler /
                                   fdcan_nominal_total_tq;
    } else {
        fdcan_calculated_bitrate = 0U;
    }
    fdcan_message_ram_std_filter_sa = hfdcan1.msgRam.StandardFilterSA;
    fdcan_message_ram_rxfifo0_sa = hfdcan1.msgRam.RxFIFO0SA;
    fdcan_message_ram_txfifoq_sa = hfdcan1.msgRam.TxFIFOQSA;
    fdcan_message_ram_end_sa = hfdcan1.msgRam.EndAddress;
    if (hfdcan1.msgRam.StandardFilterSA != 0U) {
        fdcan_std_filter_word0 = *(__IO uint32_t *)hfdcan1.msgRam.StandardFilterSA;
    }

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
        fdcan_last_error_code = protocol_status.LastErrorCode;
        fdcan_data_last_error_code = protocol_status.DataLastErrorCode;
        fdcan_activity = protocol_status.Activity;
        fdcan_bus_off = protocol_status.BusOff;
        fdcan_error_passive = protocol_status.ErrorPassive;
        fdcan_warning = protocol_status.Warning;
        fdcan_rx_esi_flag = protocol_status.RxESIflag;
    }

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
        fdcan_tx_error_count = error_counters.TxErrorCnt;
        fdcan_rx_error_count = error_counters.RxErrorCnt;
    }

    fdcan_cccr = hfdcan1.Instance->CCCR;
    fdcan_txfqs = hfdcan1.Instance->TXFQS;
    fdcan_rxf0s = hfdcan1.Instance->RXF0S;
    fdcan_rxf0s_fill = fdcan_rxf0s & 0x7FU;
    fdcan_ir = hfdcan1.Instance->IR;
    fdcan_ie = hfdcan1.Instance->IE;
    fdcan_ils = hfdcan1.Instance->ILS;
    fdcan_ile = hfdcan1.Instance->ILE;
    fdcan_txbrp = hfdcan1.Instance->TXBRP;
    fdcan_txbto = hfdcan1.Instance->TXBTO;
    fdcan_txbc = hfdcan1.Instance->TXBC;
    fdcan_txbcr = hfdcan1.Instance->TXBCR;
    fdcan_txbcf = hfdcan1.Instance->TXBCF;
    fdcan_txbtie = hfdcan1.Instance->TXBTIE;
    fdcan_txbcie = hfdcan1.Instance->TXBCIE;
    fdcan_test = hfdcan1.Instance->TEST;
    fdcan_psr = hfdcan1.Instance->PSR;
    fdcan_mode_runtime = hfdcan1.Init.Mode;
    fdcan_cccr_init = (fdcan_cccr >> 0) & 0x1U;
    fdcan_cccr_cce = (fdcan_cccr >> 1) & 0x1U;
    fdcan_cccr_asm = (fdcan_cccr >> 2) & 0x1U;
    fdcan_cccr_mon = (fdcan_cccr >> 5) & 0x1U;
    fdcan_cccr_test = (fdcan_cccr & FDCAN_CCCR_TEST) ? 1U : 0U;
    fdcan_test_lbck = (fdcan_test & FDCAN_TEST_LBCK) ? 1U : 0U;
    fdcan_test_tx = (fdcan_test & FDCAN_TEST_TX) >> FDCAN_TEST_TX_Pos;
    fdcan_test_rx = (fdcan_test & FDCAN_TEST_RX) >> FDCAN_TEST_RX_Pos;
    fdcan_txfqs_free = (fdcan_txfqs & FDCAN_TXFQS_TFFL) >> FDCAN_TXFQS_TFFL_Pos;
    fdcan_txfqs_get_index = (fdcan_txfqs & FDCAN_TXFQS_TFGI) >> FDCAN_TXFQS_TFGI_Pos;
    fdcan_txfqs_put_index = (fdcan_txfqs & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    fdcan_txfqs_full = (fdcan_txfqs & FDCAN_TXFQS_TFQF) ? 1U : 0U;
    fdcan_tx_pending_count = ((fdcan_txbrp & 0x1U) ? 1U : 0U) +
                             ((fdcan_txbrp & 0x2U) ? 1U : 0U) +
                             ((fdcan_txbrp & 0x4U) ? 1U : 0U);
    fdcan_tx_completed_count = ((fdcan_txbto & 0x1U) ? 1U : 0U) +
                               ((fdcan_txbto & 0x2U) ? 1U : 0U) +
                               ((fdcan_txbto & 0x4U) ? 1U : 0U);
    fdcan_psr_lec = (fdcan_psr >> 0) & 0x7U;
    fdcan_psr_activity = (fdcan_psr >> 3) & 0x3U;
    fdcan_psr_error_passive = (fdcan_psr >> 5) & 0x1U;
    fdcan_psr_warning = (fdcan_psr >> 6) & 0x1U;
    fdcan_psr_bus_off = (fdcan_psr >> 7) & 0x1U;
    fdcan_psr_dlec = (fdcan_psr >> 8) & 0x7U;
}

static void fdcan_probe_rx_pin(void)
{
    static uint32_t last_rx_level = 0xFFFFFFFFU;
    static uint32_t last_tx_level = 0xFFFFFFFFU;
    uint32_t rx_high_count = 0;
    uint32_t rx_low_count = 0;
    uint32_t rx_edge_count = 0;
    uint32_t tx_high_count = 0;
    uint32_t tx_low_count = 0;
    uint32_t tx_edge_count = 0;

    for (uint32_t i = 0; i < 16384U; i++) {
        uint32_t rx_level = (GPIOB->IDR & GPIO_PIN_8) ? 1U : 0U;
        uint32_t tx_level = (GPIOB->IDR & GPIO_PIN_9) ? 1U : 0U;

        fdcan_rx_pin_level = rx_level;
        fdcan_rx_pin_sample_count++;
        fdcan_tx_pin_level = tx_level;
        fdcan_tx_pin_sample_count++;

        if (rx_level != 0U) {
            fdcan_rx_pin_high_seen++;
            rx_high_count++;
        } else {
            fdcan_rx_pin_low_seen++;
            rx_low_count++;
        }

        if (tx_level != 0U) {
            fdcan_tx_pin_high_seen++;
            tx_high_count++;
        } else {
            fdcan_tx_pin_low_seen++;
            tx_low_count++;
        }

        if ((last_rx_level != 0xFFFFFFFFU) && (rx_level != last_rx_level)) {
            fdcan_rx_pin_edge_count++;
            rx_edge_count++;
        }

        if ((last_tx_level != 0xFFFFFFFFU) && (tx_level != last_tx_level)) {
            fdcan_tx_pin_edge_count++;
            tx_edge_count++;
        }

        last_rx_level = rx_level;
        last_tx_level = tx_level;
    }

    fdcan_rx_probe_high_count = rx_high_count;
    fdcan_rx_probe_low_count = rx_low_count;
    fdcan_rx_probe_edge_count = rx_edge_count;
    fdcan_rx_probe_dominant_seen = (rx_low_count > 0U) ? 1U : 0U;
    fdcan_rx_probe_idle_high = ((rx_high_count > 0U) && (rx_low_count == 0U)) ? 1U : 0U;
    fdcan_tx_probe_high_count = tx_high_count;
    fdcan_tx_probe_low_count = tx_low_count;
    fdcan_tx_probe_edge_count = tx_edge_count;
    fdcan_tx_probe_dominant_seen = (tx_low_count > 0U) ? 1U : 0U;
    fdcan_tx_probe_idle_high = ((tx_high_count > 0U) && (tx_low_count == 0U)) ? 1U : 0U;
}

static void fdcan_app_start(void)
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

    if (fdcan_started != 0U) {
        fdcan_snapshot_status();
        return;
    }

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;
    fdcan_filter_id1 = filter.FilterID1;
    fdcan_filter_id2 = filter.FilterID2;

    fdcan_filter_ret = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    fdcan_global_filter_ret = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                                           FDCAN_ACCEPT_IN_RX_FIFO0,
                                                           FDCAN_ACCEPT_IN_RX_FIFO0,
                                                           FDCAN_REJECT_REMOTE,
                                                           FDCAN_REJECT_REMOTE);

#if CAN_RX_INTERRUPT_ENABLE
    fdcan_interrupt_line_ret = HAL_FDCAN_ConfigInterruptLines(&hfdcan1, active_its, FDCAN_INTERRUPT_LINE0);
    fdcan_notification_ret = HAL_FDCAN_ActivateNotification(&hfdcan1,
                                                            active_its,
                                                            FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
#else
    (void)active_its;
    fdcan_interrupt_line_ret = HAL_OK;
    fdcan_notification_ret = HAL_OK;
#endif

    fdcan_start_ret = HAL_FDCAN_Start(&hfdcan1);
    if (fdcan_start_ret == HAL_OK) {
        fdcan_started = 1U;
    }
    fdcan_snapshot_status();
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    rx_fifo0_callback_count++;
    rx_fifo0_its_last = RxFifo0ITs;

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
            rx_get_err_count++;
            break;
        }

        frame.id = rxHeader.Identifier;
        frame.dlc = fdcan_dlc_to_bytes(rxHeader.DataLength);
        rx_irq_count++;
        rx_get_ok_count++;
        last_rx_frame = frame;

        if (canRxQueueHandle != NULL) {
            rx_queue_put_last_status = osMessageQueuePut(canRxQueueHandle, &frame, 0U, 0U);
            if (rx_queue_put_last_status == osOK) {
                rx_queue_put_ok_count++;
            } else {
                rx_queue_put_err_count++;
            }
        }
    }
}

static uint8_t fdcan_dlc_to_bytes(uint32_t dlc)
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

    last_tx_complete_buffer = BufferIndexes;
    tx_complete_count++;
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    last_tx_abort_buffer = BufferIndexes;
    tx_abort_count++;
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    tx_fifo_empty_count++;
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    fdcan_error_callback_count++;
    fdcan_error_code = HAL_FDCAN_GetError(hfdcan);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    fdcan_error_status_callback_count++;
    fdcan_psr = hfdcan->Instance->PSR;
    (void)ErrorStatusITs;
}

/* USER CODE END Application */
