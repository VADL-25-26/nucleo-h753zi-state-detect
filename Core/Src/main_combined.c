/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with IMU selection capability
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
#include "usart.h"
#include "dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flight_state.h"   // flight state machine
#include "imu.h"            // generic IMU interface
#include "imu_source.h"     // IMU source selection
#include <stdio.h>

/* IMU interface implementations */
void IMU_Sim_Tick_200Hz(void); // IMU simulator tick (implemented in imu_sim.c)

#include "imu_real.h"      // Real IMU interface
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

UART_HandleTypeDef huart1; /* Console UART */
UART_HandleTypeDef huart2; /* IMU UART */
DMA_HandleTypeDef hdma_usart2_rx;
TIM_HandleTypeDef htim6;   /* 200Hz timer for state updates */

/* USER CODE BEGIN PV */
// 200 Hz "run state task" flag set by TIM6 ISR
static volatile uint8_t g_state_tick = 0;

// Track last printed state to print only on transitions
static FlightState_t g_last_state = FLIGHT_STATE_GROUND_IDLE;

// Current IMU source selection
static IMU_Source_t g_imu_source = IMU_SOURCE_SIMULATED;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
// Helpers: print a state transition and drive LEDs by state
static void PrintState(FlightState_t s);
static void DriveStateLED(FlightState_t s);
static void SwitchIMUSource(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Timer ISR hook: TIM6 @ 200 Hz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim6) {
    // Generate one new simulated IMU sample if using simulation
    if (g_imu_source == IMU_SOURCE_SIMULATED) {
      IMU_Sim_Tick_200Hz();
    }
    
    // Set the state-tick flag for main loop processing
    g_state_tick = 1;
  }
}

// UART Idle Callback - called from HAL_UART_IRQHandler
void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2 && g_imu_source == IMU_SOURCE_REAL) {
    IMU_Real_UART_IdleCallback();
  }
}

// Simple UART print of state transitions using printf
static void PrintState(FlightState_t s)
{
  const char* name = "UNK";
  switch (s) {
    case FLIGHT_STATE_GROUND_IDLE:
      name = "GROUND_IDLE";
      break;
    case FLIGHT_STATE_POWERED_FLIGHT:
      name = "POWERED_FLIGHT";
      break;
    case FLIGHT_STATE_UNPOWERED_FLIGHT:
      name = "UNPOWERED_FLIGHT";
      break;
    case FLIGHT_STATE_DESCENT:
      name = "DESCENT";
      break;
    case FLIGHT_STATE_LANDED:
      name = "LANDED";
      break;
  }
  printf("STATE -> %s\r\n", name);
}

// Simple visual LED mapping per state
static void DriveStateLED(FlightState_t s)
{
  static uint32_t last_toggle_ms = 0;
  uint32_t now = HAL_GetTick();

  // Example mapping:
  // GREEN for "active flight", BLUE for descent modes, RED for LANDED solid
  switch (s) {
    case FLIGHT_STATE_GROUND_IDLE:
      BSP_LED_Off(LED_RED);
      BSP_LED_Off(LED_BLUE);
      BSP_LED_Off(LED_GREEN);
      break;
    case FLIGHT_STATE_POWERED_FLIGHT:
      BSP_LED_Off(LED_RED);
      BSP_LED_Off(LED_BLUE);
      BSP_LED_On(LED_GREEN);
      break;
    case FLIGHT_STATE_UNPOWERED_FLIGHT:
      BSP_LED_Off(LED_RED);
      BSP_LED_Off(LED_BLUE);
      // Blink green at 5 Hz
      if (now - last_toggle_ms > 100) {
        BSP_LED_Toggle(LED_GREEN);
        last_toggle_ms = now;
      }
      break;
    case FLIGHT_STATE_DESCENT:
      BSP_LED_Off(LED_RED);
      BSP_LED_On(LED_BLUE);
      BSP_LED_Off(LED_GREEN);
      break;
    case FLIGHT_STATE_LANDED:
      BSP_LED_On(LED_RED);
      BSP_LED_Off(LED_BLUE);
      BSP_LED_Off(LED_GREEN);
      break;
  }
}

// Switch between simulated and real IMU data sources
static void SwitchIMUSource(void)
{
  if (g_imu_source == IMU_SOURCE_SIMULATED) {
    printf("Switching to REAL IMU data source\r\n");
    g_imu_source = IMU_SOURCE_REAL;
    
    // Initialize the real IMU hardware
    IMU_Real_Init(&huart2, &hdma_usart2_rx);
    IMU_Real_Start();
  } else {
    printf("Switching to SIMULATED IMU data source\r\n");
    g_imu_source = IMU_SOURCE_SIMULATED;
    
    // No special initialization required for simulated IMU
  }
  
  // Reset the flight state since we've changed data sources
  FlightState_Init();
  g_last_state = FlightState_GetCurrent();
  PrintState(g_last_state);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  /* Initialize LEDs */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  
  /* Initialize flight state machine with simulated IMU by default */
  g_imu_source = IMU_SOURCE_SIMULATED;
  FlightState_Init();

  /* Start TIM6 @ 200 Hz */
  HAL_TIM_Base_Start_IT(&htim6);

  /* Optional: banner */
  printf("FlightState with IMU source selection @200Hz\r\n");
  printf("Press USER button to toggle between simulated and real IMU\r\n");

  /* Record initial state for transition prints */
  g_last_state = FlightState_GetCurrent();
  PrintState(g_last_state);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Process real IMU data if we're using it */
    if (g_imu_source == IMU_SOURCE_REAL) {
      IMU_Real_Process();
    }
    
    /* Check if it's time to update the flight state */
    if (g_state_tick) {
      g_state_tick = 0;
      
      /* Update the flight state */
      FlightState_Update();
      
      /* Check for state changes */
      FlightState_t current_state = FlightState_GetCurrent();
      if (current_state != g_last_state) {
        PrintState(current_state);
        g_last_state = current_state;
      }
      
      /* Update LEDs based on current state */
      DriveStateLED(current_state);
    }
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* System Clock Configuration code would go here */
  /* Generated by STM32CubeMX */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USART1 initialization code would go here */
  /* Generated by STM32CubeMX */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USART2 initialization code would go here */
  /* Generated by STM32CubeMX */
  
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief DMA Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA_Init(void)
{
  /* DMA initialization code would go here */
  /* Generated by STM32CubeMX */
  
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  
  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO initialization code would go here */
  /* Generated by STM32CubeMX */
}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{
  /* TIM6 initialization code would go here */
  /* Generated by STM32CubeMX */
  
  /* Configure TIM6 for 200Hz operation */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = (SystemCoreClock/2/200000) - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER) {
    /* User button pressed - toggle IMU source */
    SwitchIMUSource();
  }
}
/* USER CODE END 4 */

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
    /* Toggle RED LED rapidly to indicate error */
    BSP_LED_Toggle(LED_RED);
    HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
