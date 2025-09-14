/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : This was the main program that ashley wrote to test the IMU.
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
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IMU_BAUD 	115200
#define RX_BUF_SIZE 512
#define IMU_UART 	huart2
#define FRAME_SYNC     0xFA
#define FRAME_AFTERSYNC_LEN 35
#define FRAME_TOTAL_LEN     (1 + FRAME_AFTERSYNC_LEN)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//RxDataBuffer
static uint8_t rx_dma_buf[RX_BUF_SIZE];
static volatile uint32_t dma_last = 0;
volatile uint8_t  rx_idle_flag = 0;

typedef struct
{
	float yaw, pitch, roll; //degree
	float a_x, a_y, a_z; //F
	float pressure; //kPa
	float altitude; //feet
}IMUData;

// double buffer for parsed IMUdata
// we can safely update one without corrupting the one the main loop is reading
static IMUData imu_buf[2];
static volatile uint8_t imu_front = 0;

//finds the last written position in DMA buffer
static inline uint32_t dma_write_idx(void) {
    //NDTR counts down
    return RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(IMU_UART.hdmarx);
}

//function to setup DMA UART reading
void imu_uart_start_dma(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&IMU_UART); //idle flag cleared so it doesn't immediately fire again
	__HAL_UART_ENABLE_IT(&IMU_UART, UART_IT_IDLE); //enable interrupt
	HAL_UART_Receive_DMA(&IMU_UART, rx_dma_buf, RX_BUF_SIZE); //start receiving
}

//initial message sent to IMU
void imu_send_init(void) {
	const char cmd[] = "$VNWRG,75,2,4,05,0108,0020*XX\r\n";
	HAL_UART_Transmit(&huart2, (uint8_t*)cmd, sizeof(cmd)-1, 50);
}

//checksum calculation (from data sheet)
static uint16_t vn_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;
    for (uint32_t i = 0; i < len; ++i) {
        crc = (crc >> 8) | ((crc << 8) & 0xFFFF);
        crc ^= data[i];
        crc ^= (crc & 0xFF) >> 4;
        crc ^= (crc << 12) & 0xFFFF;
        crc ^= ((crc & 0xFF) << 5) & 0xFFFF;
    }
    return crc;
}


static float little_endian_2_float(const uint8_t *p)
{
	float f;
	memcpy(&f, p, sizeof(float)); //convert stm little endian to float
	return f;
}

//parse one frame of data
//m is one message frame, out is output IMUData structure
static int process_one_imu_frame(const uint8_t *m, IMUData *out){
	//verify checksum
	uint16_t calc = vn_crc16(m, 33);
	uint16_t recv = (uint16_t)((m[33] << 8) | m[34]); // big-endian
	if (calc != recv) return 0;

	//slice data
	float yaw   	= little_endian_2_float(&m[5]);
	float pitch 	= little_endian_2_float(&m[9]);
	float roll  	= little_endian_2_float(&m[13]);
	float ax 		= little_endian_2_float(&m[17]);
	float ay 		= little_endian_2_float(&m[21]);
	float az 		= little_endian_2_float(&m[25]);
	float pressure 	= little_endian_2_float(&m[29]);

	//add data to imu struct
	IMUData d;
	d.yaw = yaw;
	d.pitch = pitch;
	d.roll = roll;
	const float g = 9.80665f; //gravity
	d.a_x = ax / g;
	d.a_y = ay / g;
	d.a_z = az / g;
	d.pressure = pressure;

	if (pressure <= 0.0f) {
		d.altitude = NAN;
	} else {
		// 145366.45 * (1 - (P/101.325)^0.190284)
		float ratio = pressure / 101.325f;
		d.altitude = 145366.45f * (1.0f - powf(ratio, 0.190284f));
	}

	*out = d;
	return 1;
}

//this is where we read data from ring buffer and parse data
static void imu_process_rx(void)
{
	//find where new data has been written up to (write pointer)
	uint32_t write = dma_write_idx();

	//calculate how much available bytes
	// if write >= dma_last (since last time we processed), no wraparound
	// else, buffer wrapped
	uint32_t available_bytes = (write >= dma_last) ? (write - dma_last) : (RX_BUF_SIZE - dma_last + write);

	uint32_t i = dma_last;

	while (available_bytes){

		//look for frame sync byte (0xFA), if current byte isn't a sync, skip and keep scanning
		if (rx_dma_buf[i] != FRAME_SYNC){
			i = (i + 1) % RX_BUF_SIZE; //move to the next byte and wrap if needed
			--available_bytes;
			continue;
		}

		//at this point we've found a potential frame
		if (available_bytes < FRAME_TOTAL_LEN)
			break; //not enough data, wait longer

		//copy the next 35 bytes after sync byte (0xFA) into a linear temp array
		uint8_t raw_imu_data[FRAME_AFTERSYNC_LEN];
		for (uint32_t j = 0; j < FRAME_AFTERSYNC_LEN; ++j){
			raw_imu_data[j] = rx_dma_buf[(i + 1 + j) % RX_BUF_SIZE];
		}

		//process one frame of data
		IMUData tmp;
		if (process_one_imu_frame(raw_imu_data, &tmp)){
			uint8_t back = 1 - imu_front;
			imu_buf[back] = tmp;      // copy whole struct
			imu_front = back;         // flip front index (atomic publish)
			i = (i + FRAME_TOTAL_LEN) % RX_BUF_SIZE;
		}else{
			i = (i+1) % RX_BUF_SIZE;
		}

		available_bytes = (write >= i) ? (write - i) : (RX_BUF_SIZE - i + write); //recalculate available bytes

	}

	dma_last = i;

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Enable USART2 global interrupt */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  //init imu
  imu_uart_start_dma();
  HAL_Delay(20);
  imu_send_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (rx_idle_flag) {
		  rx_idle_flag = 0;
		  imu_process_rx();
	  }
	  IMUData cur = imu_buf[imu_front];
	  //for visualizing data
	  float yaw 		= cur.yaw;
	  float pitch 		= cur.pitch;
	  float roll 		= cur.roll;
	  float a_x 		= cur.a_x;
	  float a_y 		= cur.a_y;
	  float a_z 		= cur.a_z;
	  float pres 		= cur.pressure;
	  float altitude 	= cur.altitude;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  /* DMA1_Stream5_IRQn interrupt configuration */
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
