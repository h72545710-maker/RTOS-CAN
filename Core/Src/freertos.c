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
volatile uint32_t fdcan_psr = 0;

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
  .name = " canRxQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void fdcan_loopback_start(void);
static void fdcan_snapshot_status(void);
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
   //canRxQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), & canRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	canRxQueueHandle = osMessageQueueNew(16, sizeof(CanFrame_t), NULL);
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
	fdcan_loopback_start();
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
		    FDCAN_TxHeaderTypeDef txHeader = {0};
        uint8_t txData[8] = {0};

        txHeader.Identifier = 0x123;
        txHeader.IdType = FDCAN_STANDARD_ID;
        txHeader.TxFrameType = FDCAN_DATA_FRAME;
        txHeader.DataLength = FDCAN_DLC_BYTES_8;
        txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        txHeader.MessageMarker = 0;

        txData[0] = counter++;
        txData[1] = 0x11;
        txData[2] = 0x22;
        txData[3] = 0x33;
        txData[4] = 0x44;
        txData[5] = 0x55;
        txData[6] = 0x66;
        txData[7] = 0x77;
				
        tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

        if (tx_fifo_free_level == 0U) {
            tx_full_skip_count++;
            fdcan_snapshot_status();
            osDelay(1000);
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
    osDelay(1000);
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
    if (osMessageQueueGet(canRxQueueHandle, &frame, NULL, osWaitForever) == osOK) {
            last_rx_frame = frame;
            rx_task_count++;
            __NOP();
        }
  }
  /* USER CODE END can_rx_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void fdcan_snapshot_status(void)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};

    tx_fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    fdcan_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    fdcan_error_code = HAL_FDCAN_GetError(&hfdcan1);

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK) {
        fdcan_bus_off = protocol_status.BusOff;
        fdcan_error_passive = protocol_status.ErrorPassive;
        fdcan_warning = protocol_status.Warning;
    }

    if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &error_counters) == HAL_OK) {
        fdcan_tx_error_count = error_counters.TxErrorCnt;
        fdcan_rx_error_count = error_counters.RxErrorCnt;
    }

    fdcan_cccr = hfdcan1.Instance->CCCR;
    fdcan_txfqs = hfdcan1.Instance->TXFQS;
    fdcan_rxf0s = hfdcan1.Instance->RXF0S;
    fdcan_ir = hfdcan1.Instance->IR;
    fdcan_ie = hfdcan1.Instance->IE;
    fdcan_ils = hfdcan1.Instance->ILS;
    fdcan_ile = hfdcan1.Instance->ILE;
    fdcan_txbrp = hfdcan1.Instance->TXBRP;
    fdcan_txbto = hfdcan1.Instance->TXBTO;
    fdcan_psr = hfdcan1.Instance->PSR;
}

static void fdcan_loopback_start(void)
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

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;

    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE);

    HAL_FDCAN_ConfigInterruptLines(&hfdcan1, active_its, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   active_its,
                                   FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);

    fdcan_start_ret = HAL_FDCAN_Start(&hfdcan1);
    fdcan_snapshot_status();
}



void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    FDCAN_RxHeaderTypeDef rxHeader = {0};
    CanFrame_t frame = {0};

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, frame.data) != HAL_OK) {
        return;
    }

    frame.id = rxHeader.Identifier;
    frame.dlc = 8;
    rx_irq_count++;

    if (canRxQueueHandle != NULL) {
        osMessageQueuePut(canRxQueueHandle, &frame, 0U, 0U);
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

