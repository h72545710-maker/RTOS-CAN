/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_BARE_CAN_BRINGUP 0
#define BARE_CAN_TX_ENABLE 1
#define BARE_CAN_TX_ID 0x120U
#define BARE_CAN_TX_PERIOD_MS 1000U
#define BARE_CAN_MAIN_LOOP_DELAY_MS 1U
#define BARE_CAN_SNAPSHOT_PERIOD_MS 100U
#define BARE_CAN_EXPECT_RX_ID 0x321U
#define BARE_CAN_RX_PROBE_ENABLE 1
#define BARE_CAN_RX_PROBE_SAMPLES 65536U
#define BARE_CAN_RX_GPIO GPIOB
#define BARE_CAN_RX_PIN GPIO_PIN_8
#define BARE_CAN_TX_GPIO GPIOB
#define BARE_CAN_TX_PIN GPIO_PIN_9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile HAL_StatusTypeDef bare_fdcan_filter_ret = HAL_OK;
volatile HAL_StatusTypeDef bare_fdcan_global_filter_ret = HAL_OK;
volatile HAL_StatusTypeDef bare_fdcan_start_ret = HAL_OK;
volatile HAL_StatusTypeDef bare_last_tx_ret = HAL_OK;
volatile HAL_StatusTypeDef bare_last_rx_ret = HAL_OK;

volatile uint32_t bare_fdcan_started = 0;
volatile uint32_t bare_tx_ok_count = 0;
volatile uint32_t bare_tx_err_count = 0;
volatile uint32_t bare_rx_ok_count = 0;
volatile uint32_t bare_rx_err_count = 0;
volatile uint32_t bare_rx_fifo0_fill = 0;
volatile uint32_t bare_tx_fifo_free = 0;
volatile uint32_t bare_last_rx_id = 0;
volatile uint32_t bare_last_rx_dlc = 0;
volatile uint8_t bare_last_rx_data[8] = {0};

volatile uint32_t bare_fdcan_ir = 0;
volatile uint32_t bare_fdcan_psr = 0;
volatile uint32_t bare_fdcan_cccr = 0;
volatile uint32_t bare_fdcan_rxf0s = 0;
volatile uint32_t bare_fdcan_txfqs = 0;
volatile uint32_t bare_fdcan_error_code = 0;
volatile uint32_t bare_fdcan_last_error_code = 0;
volatile uint32_t bare_fdcan_activity = 0;
volatile uint32_t bare_fdcan_rx_error_count = 0;
volatile uint32_t bare_fdcan_tx_error_count = 0;

volatile uint32_t bare_cccr_init = 0;
volatile uint32_t bare_cccr_cce = 0;
volatile uint32_t bare_cccr_asm = 0;
volatile uint32_t bare_cccr_mon = 0;
volatile uint32_t bare_cccr_test = 0;
volatile uint32_t bare_cccr_fdoe = 0;
volatile uint32_t bare_cccr_brse = 0;

volatile uint32_t bare_psr_lec = 0;
volatile uint32_t bare_psr_dlec = 0;
volatile uint32_t bare_psr_act = 0;
volatile uint32_t bare_psr_ep = 0;
volatile uint32_t bare_psr_ew = 0;
volatile uint32_t bare_psr_bo = 0;
volatile uint32_t bare_psr_pxe = 0;

volatile uint32_t bare_ir_rf0n = 0;
volatile uint32_t bare_ir_rf0f = 0;
volatile uint32_t bare_ir_rf0l = 0;
volatile uint32_t bare_ir_pea = 0;
volatile uint32_t bare_ir_ped = 0;
volatile uint32_t bare_ir_bo = 0;
volatile uint32_t bare_ir_ep = 0;
volatile uint32_t bare_ir_ew = 0;

volatile uint32_t bare_rxf0s_fill_direct = 0;
volatile uint32_t bare_rxf0s_get_index = 0;
volatile uint32_t bare_rxf0s_put_index = 0;
volatile uint32_t bare_rxf0s_full = 0;
volatile uint32_t bare_rxf0s_lost = 0;

volatile uint32_t bare_rx_pin_level = 0;
volatile uint32_t bare_tx_pin_level = 0;
volatile uint32_t bare_rx_pin_high_seen = 0;
volatile uint32_t bare_rx_pin_low_seen = 0;
volatile uint32_t bare_rx_pin_edge_count = 0;
volatile uint32_t bare_tx_pin_high_seen = 0;
volatile uint32_t bare_tx_pin_low_seen = 0;
volatile uint32_t bare_tx_pin_edge_count = 0;

volatile uint32_t bare_rx_probe_window_count = 0;
volatile uint32_t bare_rx_probe_last_high_count = 0;
volatile uint32_t bare_rx_probe_last_low_count = 0;
volatile uint32_t bare_rx_probe_last_edge_count = 0;
volatile uint32_t bare_rx_probe_low_seen_latch = 0;
volatile uint32_t bare_rx_probe_edge_seen_latch = 0;
volatile uint32_t bare_rx_probe_low_window_count = 0;
volatile uint32_t bare_rx_probe_edge_window_count = 0;
volatile uint32_t bare_rx_probe_last_low_tick = 0;
volatile uint32_t bare_rx_probe_last_edge_tick = 0;
volatile uint32_t bare_poll_count = 0;
volatile uint32_t bare_snapshot_count = 0;
volatile uint32_t bare_tx_attempt_count = 0;
volatile uint32_t bare_tx_skip_no_fifo_count = 0;
volatile uint32_t bare_expected_rx_id = BARE_CAN_EXPECT_RX_ID;
volatile uint32_t bare_expected_rx_seen_count = 0;
volatile uint32_t bare_fdcan_init_mode = 0;
volatile uint32_t bare_fdcan_is_normal_mode = 0;
volatile uint32_t bare_last_poll_tick = 0;
volatile uint32_t bare_last_snapshot_tick = 0;
volatile uint32_t bare_last_tx_tick = 0;
volatile uint32_t bare_sysclk_hz = 0;
volatile uint32_t bare_hclk_hz = 0;
volatile uint32_t bare_can_kernel_clock_hz = 0;
volatile uint32_t bare_can_calculated_bitrate = 0;
volatile uint32_t bare_can_alive = 0xBEEFCAFE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_BareMetal_Start(void);
static void CAN_BareMetal_Poll(void);
#if BARE_CAN_TX_ENABLE
static void CAN_BareMetal_SendDemo(void);
#endif
static void CAN_BareMetal_ProbeRxPin(void);
static void CAN_BareMetal_Snapshot(void);
static uint8_t CAN_BareMetal_DlcToBytes(uint32_t dlc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
#if USE_BARE_CAN_BRINGUP
  CAN_BareMetal_Start();
#endif

  /* USER CODE END 2 */

#if !USE_BARE_CAN_BRINGUP
  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
#endif

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if USE_BARE_CAN_BRINGUP
    CAN_BareMetal_Poll();
    HAL_Delay(BARE_CAN_MAIN_LOOP_DELAY_MS);
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void CAN_BareMetal_Start(void)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;

  bare_fdcan_filter_ret = HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  bare_fdcan_global_filter_ret = HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                                              FDCAN_ACCEPT_IN_RX_FIFO0,
                                                              FDCAN_ACCEPT_IN_RX_FIFO0,
                                                              FDCAN_REJECT_REMOTE,
                                                              FDCAN_REJECT_REMOTE);
  bare_fdcan_start_ret = HAL_FDCAN_Start(&hfdcan1);
  bare_fdcan_started = (bare_fdcan_start_ret == HAL_OK) ? 1U : 0U;

  CAN_BareMetal_Snapshot();
}

static void CAN_BareMetal_Poll(void)
{
  FDCAN_RxHeaderTypeDef rxHeader = {0};
  uint8_t rxData[8] = {0};
  uint8_t rxBytes = 0;
  uint32_t now = HAL_GetTick();
  uint32_t snapshotNeeded = 0U;

  bare_poll_count++;
  bare_last_poll_tick = now;
#if BARE_CAN_RX_PROBE_ENABLE
  CAN_BareMetal_ProbeRxPin();
#endif
  bare_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);

  while (bare_rx_fifo0_fill > 0U) {
    bare_last_rx_ret = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData);

    if (bare_last_rx_ret == HAL_OK) {
      bare_last_rx_id = rxHeader.Identifier;
      bare_last_rx_dlc = CAN_BareMetal_DlcToBytes(rxHeader.DataLength);
      rxBytes = (bare_last_rx_dlc > 8U) ? 8U : (uint8_t)bare_last_rx_dlc;

      for (uint32_t i = 0; i < 8U; i++) {
        bare_last_rx_data[i] = (i < rxBytes) ? rxData[i] : 0U;
      }

      bare_rx_ok_count++;
      if (bare_last_rx_id == BARE_CAN_EXPECT_RX_ID) {
        bare_expected_rx_seen_count++;
      }
      snapshotNeeded = 1U;
    } else {
      bare_rx_err_count++;
      snapshotNeeded = 1U;
    }

    bare_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
  }

#if BARE_CAN_TX_ENABLE
  CAN_BareMetal_SendDemo();
#endif

  if (bare_last_tx_ret != HAL_OK) {
    snapshotNeeded = 1U;
  }

  now = HAL_GetTick();
  if (((now - bare_last_snapshot_tick) >= BARE_CAN_SNAPSHOT_PERIOD_MS) || (snapshotNeeded != 0U)) {
    CAN_BareMetal_Snapshot();
  }
}

#if BARE_CAN_TX_ENABLE
static void CAN_BareMetal_SendDemo(void)
{
  static uint32_t lastTxMs = 0;
  static uint8_t counter = 0;
  FDCAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  uint32_t now = HAL_GetTick();

  if ((now - lastTxMs) < BARE_CAN_TX_PERIOD_MS) {
    return;
  }

  lastTxMs = now;
  bare_last_tx_tick = now;
  bare_tx_attempt_count++;
  bare_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);

  if (bare_tx_fifo_free == 0U) {
    bare_tx_skip_no_fifo_count++;
    bare_tx_err_count++;
    return;
  }

  txHeader.Identifier = BARE_CAN_TX_ID;
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

  bare_last_tx_ret = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData);

  if (bare_last_tx_ret == HAL_OK) {
    bare_tx_ok_count++;
  } else {
    bare_tx_err_count++;
  }
}
#endif

static void CAN_BareMetal_ProbeRxPin(void)
{
  static uint32_t lastRxLevel = 0xFFFFFFFFU;
  uint32_t highCount = 0U;
  uint32_t lowCount = 0U;
  uint32_t edgeCount = 0U;

  for (uint32_t i = 0U; i < BARE_CAN_RX_PROBE_SAMPLES; i++) {
    uint32_t rxLevel = ((BARE_CAN_RX_GPIO->IDR & BARE_CAN_RX_PIN) != 0U) ? 1U : 0U;

    if (rxLevel != 0U) {
      highCount++;
    } else {
      lowCount++;
    }

    if ((lastRxLevel != 0xFFFFFFFFU) && (rxLevel != lastRxLevel)) {
      edgeCount++;
    }

    lastRxLevel = rxLevel;
  }

  bare_rx_probe_window_count++;
  bare_rx_probe_last_high_count = highCount;
  bare_rx_probe_last_low_count = lowCount;
  bare_rx_probe_last_edge_count = edgeCount;

  if (lowCount != 0U) {
    bare_rx_probe_low_seen_latch = 1U;
    bare_rx_probe_low_window_count++;
    bare_rx_probe_last_low_tick = HAL_GetTick();
  }

  if (edgeCount != 0U) {
    bare_rx_probe_edge_seen_latch = 1U;
    bare_rx_probe_edge_window_count++;
    bare_rx_probe_last_edge_tick = HAL_GetTick();
  }
}

static void CAN_BareMetal_Snapshot(void)
{
  FDCAN_ProtocolStatusTypeDef protocolStatus = {0};
  FDCAN_ErrorCountersTypeDef errorCounters = {0};
  uint32_t bitTimeQuanta = 0;
  static uint32_t lastRxLevel = 0xFFFFFFFFU;
  static uint32_t lastTxLevel = 0xFFFFFFFFU;

  bare_snapshot_count++;
  bare_last_snapshot_tick = HAL_GetTick();
  bare_fdcan_init_mode = hfdcan1.Init.Mode;
  bare_fdcan_is_normal_mode = (hfdcan1.Init.Mode == FDCAN_MODE_NORMAL) ? 1U : 0U;
  bare_fdcan_ir = hfdcan1.Instance->IR;
  bare_fdcan_psr = hfdcan1.Instance->PSR;
  bare_fdcan_cccr = hfdcan1.Instance->CCCR;
  bare_fdcan_rxf0s = hfdcan1.Instance->RXF0S;
  bare_fdcan_txfqs = hfdcan1.Instance->TXFQS;
  bare_rx_fifo0_fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
  bare_tx_fifo_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
  bare_fdcan_error_code = HAL_FDCAN_GetError(&hfdcan1);

  bare_cccr_init = (bare_fdcan_cccr & FDCAN_CCCR_INIT) >> FDCAN_CCCR_INIT_Pos;
  bare_cccr_cce = (bare_fdcan_cccr & FDCAN_CCCR_CCE) >> FDCAN_CCCR_CCE_Pos;
  bare_cccr_asm = (bare_fdcan_cccr & FDCAN_CCCR_ASM) >> FDCAN_CCCR_ASM_Pos;
  bare_cccr_mon = (bare_fdcan_cccr & FDCAN_CCCR_MON) >> FDCAN_CCCR_MON_Pos;
  bare_cccr_test = (bare_fdcan_cccr & FDCAN_CCCR_TEST) >> FDCAN_CCCR_TEST_Pos;
  bare_cccr_fdoe = (bare_fdcan_cccr & FDCAN_CCCR_FDOE) >> FDCAN_CCCR_FDOE_Pos;
  bare_cccr_brse = (bare_fdcan_cccr & FDCAN_CCCR_BRSE) >> FDCAN_CCCR_BRSE_Pos;

  bare_psr_lec = (bare_fdcan_psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos;
  bare_psr_dlec = (bare_fdcan_psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos;
  bare_psr_act = (bare_fdcan_psr & FDCAN_PSR_ACT) >> FDCAN_PSR_ACT_Pos;
  bare_psr_ep = (bare_fdcan_psr & FDCAN_PSR_EP) >> FDCAN_PSR_EP_Pos;
  bare_psr_ew = (bare_fdcan_psr & FDCAN_PSR_EW) >> FDCAN_PSR_EW_Pos;
  bare_psr_bo = (bare_fdcan_psr & FDCAN_PSR_BO) >> FDCAN_PSR_BO_Pos;
  bare_psr_pxe = (bare_fdcan_psr & FDCAN_PSR_PXE) >> FDCAN_PSR_PXE_Pos;

  bare_ir_rf0n = (bare_fdcan_ir & FDCAN_IR_RF0N) >> FDCAN_IR_RF0N_Pos;
  bare_ir_rf0f = (bare_fdcan_ir & FDCAN_IR_RF0F) >> FDCAN_IR_RF0F_Pos;
  bare_ir_rf0l = (bare_fdcan_ir & FDCAN_IR_RF0L) >> FDCAN_IR_RF0L_Pos;
  bare_ir_pea = (bare_fdcan_ir & FDCAN_IR_PEA) >> FDCAN_IR_PEA_Pos;
  bare_ir_ped = (bare_fdcan_ir & FDCAN_IR_PED) >> FDCAN_IR_PED_Pos;
  bare_ir_bo = (bare_fdcan_ir & FDCAN_IR_BO) >> FDCAN_IR_BO_Pos;
  bare_ir_ep = (bare_fdcan_ir & FDCAN_IR_EP) >> FDCAN_IR_EP_Pos;
  bare_ir_ew = (bare_fdcan_ir & FDCAN_IR_EW) >> FDCAN_IR_EW_Pos;

  bare_rxf0s_fill_direct = (bare_fdcan_rxf0s & FDCAN_RXF0S_F0FL) >> FDCAN_RXF0S_F0FL_Pos;
  bare_rxf0s_get_index = (bare_fdcan_rxf0s & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
  bare_rxf0s_put_index = (bare_fdcan_rxf0s & FDCAN_RXF0S_F0PI) >> FDCAN_RXF0S_F0PI_Pos;
  bare_rxf0s_full = (bare_fdcan_rxf0s & FDCAN_RXF0S_F0F) >> FDCAN_RXF0S_F0F_Pos;
  bare_rxf0s_lost = (bare_fdcan_rxf0s & FDCAN_RXF0S_RF0L) >> FDCAN_RXF0S_RF0L_Pos;

  bare_rx_pin_level = ((BARE_CAN_RX_GPIO->IDR & BARE_CAN_RX_PIN) != 0U) ? 1U : 0U;
  bare_tx_pin_level = ((BARE_CAN_TX_GPIO->IDR & BARE_CAN_TX_PIN) != 0U) ? 1U : 0U;

  if (bare_rx_pin_level != 0U) {
    bare_rx_pin_high_seen++;
  } else {
    bare_rx_pin_low_seen++;
  }

  if (bare_tx_pin_level != 0U) {
    bare_tx_pin_high_seen++;
  } else {
    bare_tx_pin_low_seen++;
  }

  if ((lastRxLevel != 0xFFFFFFFFU) && (bare_rx_pin_level != lastRxLevel)) {
    bare_rx_pin_edge_count++;
  }

  if ((lastTxLevel != 0xFFFFFFFFU) && (bare_tx_pin_level != lastTxLevel)) {
    bare_tx_pin_edge_count++;
  }

  lastRxLevel = bare_rx_pin_level;
  lastTxLevel = bare_tx_pin_level;

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocolStatus) == HAL_OK) {
    bare_fdcan_last_error_code = protocolStatus.LastErrorCode;
    bare_fdcan_activity = protocolStatus.Activity;
  }

  if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &errorCounters) == HAL_OK) {
    bare_fdcan_rx_error_count = errorCounters.RxErrorCnt;
    bare_fdcan_tx_error_count = errorCounters.TxErrorCnt;
  }

  bare_sysclk_hz = HAL_RCC_GetSysClockFreq();
  bare_hclk_hz = HAL_RCC_GetHCLKFreq();
  bare_can_kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  bitTimeQuanta = 1U + hfdcan1.Init.NominalTimeSeg1 + hfdcan1.Init.NominalTimeSeg2;

  if ((bare_can_kernel_clock_hz != 0U) && (hfdcan1.Init.NominalPrescaler != 0U) && (bitTimeQuanta != 0U)) {
    bare_can_calculated_bitrate = bare_can_kernel_clock_hz / hfdcan1.Init.NominalPrescaler / bitTimeQuanta;
  } else {
    bare_can_calculated_bitrate = 0U;
  }
}

static uint8_t CAN_BareMetal_DlcToBytes(uint32_t dlc)
{
  switch (dlc) {
    case FDCAN_DLC_BYTES_0:
      return 0;
    case FDCAN_DLC_BYTES_1:
      return 1;
    case FDCAN_DLC_BYTES_2:
      return 2;
    case FDCAN_DLC_BYTES_3:
      return 3;
    case FDCAN_DLC_BYTES_4:
      return 4;
    case FDCAN_DLC_BYTES_5:
      return 5;
    case FDCAN_DLC_BYTES_6:
      return 6;
    case FDCAN_DLC_BYTES_7:
      return 7;
    case FDCAN_DLC_BYTES_8:
    default:
      return 8;
  }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
