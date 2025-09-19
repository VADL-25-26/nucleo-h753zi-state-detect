/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flight_state.h"   // legacy state machine (requires enum fix noted above)
#include "imu.h"            // IMU_GetLatestData / IMU_CalculateGroundAltitude
// IMU simulator tick we call at 200 Hz tonight (implemented in imu_sim.c)
void IMU_Sim_Tick_200Hz(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
// 200 Hz “run state task” flag set by TIM7 ISR
static volatile uint8_t g_state_tick = 0;

// Track last printed state to print only on transitions
static FlightState_t g_last_state = FLIGHT_STATE_GROUND_IDLE;

static volatile uint8_t simulation_active = 0;

// Counter for timing the printing of IMU values in console
static uint32_t print_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
// Helpers: printing state transition and driving LEDs based on flight state
static char* PrintState(FlightState_t s);
static void DriveStateLED(FlightState_t s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// UART print of state transitions using printf routed to COM1 by BSP
static char* PrintState(FlightState_t s) {
	char *name = "UNK";
	switch (s) {
	case FLIGHT_STATE_GROUND_IDLE:
		name = "GROUND";
		break;
	case FLIGHT_STATE_POWERED_FLIGHT:
		name = "POWERED";
		break;
	case FLIGHT_STATE_UNPOWERED_FLIGHT:
		name = "UNPOWERED";
		break;
	case FLIGHT_STATE_DESCENT:
		name = "DESCENT";
		break;
	case FLIGHT_STATE_LANDED:
		name = "LANDED";
		break;
	default:
		break;
	}
//	printf("STATE CHANGE -> %s\r\n", name);
	return name;
}

// Printing IMU data
static void PrintIMUData(IMU_Data_t data, char* state) {
//	printf("[%s] IMU: accel=%.2f,%.2f,%.2f g alt=%.1f ft\r\n", state, data.accelX,
//			data.accelY, data.accelZ, data.altitude);
	printf("[%s] IMU: accel, pitch, roll=%.2f,%.2f,%.2f g alt=%.1f ft\r\n", state, data.yaw,
				data.accelX, data.roll, data.altitude);
}

// Simple visual LED mapping per state (uses Nucleo BSP LEDs)
static void DriveStateLED(FlightState_t s) {
	static uint32_t last_toggle_ms = 0;
	uint32_t now = HAL_GetTick();

	// Example mapping:
	// GREEN for “active flight”, BLUE for descent modes, RED for LANDED solid
	switch (s) {
	case FLIGHT_STATE_GROUND_IDLE:
		BSP_LED_Off(LED_GREEN);
		BSP_LED_Off(LED_BLUE);
		BSP_LED_Off(LED_RED);
		break;

	case FLIGHT_STATE_POWERED_FLIGHT:
		if (now - last_toggle_ms > 100) {
			BSP_LED_Toggle(LED_GREEN);
			last_toggle_ms = now;
		}
		BSP_LED_Off(LED_BLUE);
		BSP_LED_Off(LED_RED);
		break;

	case FLIGHT_STATE_UNPOWERED_FLIGHT:
		BSP_LED_On(LED_GREEN);
		BSP_LED_Off(LED_BLUE);
		BSP_LED_Off(LED_RED);
		break;

	case FLIGHT_STATE_DESCENT:
		if (now - last_toggle_ms > 600) {
			BSP_LED_Toggle(LED_BLUE);
			last_toggle_ms = now;
		}
		BSP_LED_Off(LED_GREEN);
		BSP_LED_Off(LED_RED);
		break;

	case FLIGHT_STATE_LANDED:
		BSP_LED_Off(LED_GREEN);
		BSP_LED_Off(LED_BLUE);
		BSP_LED_On(LED_RED);
		break;
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	// (nothing needed here)
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	// (nothing needed here)
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	// (nothing needed here)
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  imu_setup(&huart1);
	// Initialize legacy flight state machine; it will call IMU_CalculateGroundAltitude()
	FlightState_Init();
	// Record initial state for transition prints
	g_last_state = FlightState_GetCurrent();
	PrintState(g_last_state);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

	/* Initialize leds */
	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_BLUE);
	BSP_LED_Init(LED_RED);

	/* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

	/* Initialize COM1 port so that it can be used with PuTTy (115200, 8 bits (7-bit data + 1 stop bit), no parity */
	BspCOMInit.BaudRate = 115200;
	BspCOMInit.WordLength = COM_WORDLENGTH_8B;
	BspCOMInit.StopBits = COM_STOPBITS_1;
	BspCOMInit.Parity = COM_PARITY_NONE;
	BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
	if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
		Error_Handler();
	}

	/* -- Sample board code to send message over COM1 port ---- */
	printf("\r\n\r\n*** IMU Simulator & Flight State Demo ***\r\n");
	printf("Press USER button to start/stop simulation\r\n");

	/* -- Sample board code to switch on leds ---- */
	BSP_LED_On(LED_GREEN);
	BSP_LED_On(LED_BLUE);
	BSP_LED_On(LED_RED);

  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

//		/* -- Sample board code for User push-button in interrupt mode ---- */
//		if (BspButtonState == BUTTON_PRESSED) {
//			/* Update button state */
//			BspButtonState = BUTTON_RELEASED;
//
//			/* Toggle Simulated IMU */
//			simulation_active = !simulation_active;
//
//			if (simulation_active) {
//				printf("\r\n *** SIMULATION STARTED ***\r\n");
//				extern volatile uint32_t t_ms;
//				t_ms = 0;
//				/* Reset flight state */
//				FlightState_Init();
				g_last_state = FlightState_GetCurrent();
				PrintState(g_last_state);
//			} else {
//				printf("\r\n*** SIMULATION STOPPED ***\r\n");
//				/* Turn off all LEDs when simulation stops */
//				BSP_LED_Off(LED_GREEN);
//				BSP_LED_Off(LED_BLUE);
//				BSP_LED_Off(LED_RED);
//			}
//		}
//
//		/* USER CODE BEGIN 3 */
//		// ---- SUPERLOOP: run legacy state update at 200 Hz when the TIM7 flag is set ----
//		//if (g_state_tick || simulation_active) {
//			g_state_tick = 0;
//
//			// Run your legacy state machine once per tick.

//
//			// If state changed, print and update LED pattern

//			DriveStateLED(now);

			IMU_Data_t imu_data; //get imu data
			imu_read(&imu_data);
			FlightState_Update(imu_data); //update imu data
			FlightState_t now = FlightState_GetCurrent(); //find current state
			char* curState = PrintState(now);
			if (now != g_last_state) {

				printf("STATE CHANGE -> %s\r\n", curState);
				g_last_state = now;
			}
			PrintIMUData(imu_data, curState);
//			PrintIMUData(imu_data, 'a');

		//}

		// May add sleep until next interrupt to save power (okay to omit)
		// __WFI();
		/* USER CODE END 3 */
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// ---- SUPERLOOP: run legacy state update at 200 Hz when the TIM7 flag is set ----
//		if (g_state_tick && simulation_active) {
//			g_state_tick = 0;
//
//			// Run your legacy state machine once per tick.
//			FlightState_Update();
//
//			// If state changed, print and update LED pattern
//			FlightState_t now = FlightState_GetCurrent();
//			char* curState = PrintState(now);
//
//			if (now != g_last_state) {
//
//				printf("STATE CHANGE -> %s\r\n", curState);
//				g_last_state = now;
//			}
//			DriveStateLED(now);
//
//			print_counter++;
//			if (print_counter >= 20) {
//				IMU_Data_t imu_data;
//				imu_read(&imu_data);
//				PrintIMUData(imu_data, curState);
//				print_counter = 0;
//			}
//		}

		// May add sleep until next interrupt to save power (okay to omit)
		// __WFI();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
	// (CubeMX will generate GPIO inits for LEDs if you checked them; BSP can also handle LEDs)
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
	// (no manual GPIO here; using BSP LEDs)
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// User button callback is already below in BSP_PB_Callback
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
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
	while (1) {
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
