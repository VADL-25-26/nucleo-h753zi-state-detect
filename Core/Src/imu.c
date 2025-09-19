/*
 * imu.c
 *
 *  Created on: Sep 14, 2025
 *      Author: fuash
 */
#include "imu.h"
#include <string.h>
#include <math.h>
#include "stm32h7xx_hal.h"

/* ---- Configuration ---- */
#define IMU_RX_BUF_SIZE         512
#define IMU_FRAME_SYNC          0xFA
#define IMU_FRAME_AFTERSYNC_LEN 35
#define IMU_FRAME_TOTAL_LEN     (1 + IMU_FRAME_AFTERSYNC_LEN)

/* ---- Module state (private) ---- */
static UART_HandleTypeDef *s_uart = NULL;
static uint8_t  s_rx[IMU_RX_BUF_SIZE];
static uint32_t s_dma_last = 0;

static IMU_Data_t  s_db[2];
static uint8_t  s_front = 0;
static uint8_t  s_has_new = 0;

/* ---- Helper Functions ---- */

/*
 * Calculates Checksum, from VN100 datasheet
 */
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

/*
 * Converts little endian (STM32 format) to float point
 */
static float little_endian_2_float(const uint8_t *p) { float f; memcpy(&f, p, 4); return f; }

/*
 * Parse one frame of IMU data m
 */
static int process_one_imu_frame(const uint8_t *m, IMU_Data_t *out)
{
	// verify checksum
    uint16_t calc = vn_crc16(m, 33);
    uint16_t recv = (uint16_t)((m[33] << 8) | m[34]);
    if (calc != recv) return 0;

    // parse and process data
    IMU_Data_t d;
    d.yaw   = little_endian_2_float(&m[5]);
    d.pitch = little_endian_2_float(&m[9]);
    d.roll  = little_endian_2_float(&m[13]);

    const float g = 9.80665f;

    d.accelX = little_endian_2_float(&m[17]) / g;
    d.accelY = little_endian_2_float(&m[21]) / g;
    d.accelZ = little_endian_2_float(&m[25]) / g;
    d.pressure = little_endian_2_float(&m[29]);

    if (d.pressure <= 0.0f){
    	d.altitude = NAN;
    } else {
        float ratio = d.pressure / 101.325f;
        //TODO: current altitude calibration (9/14/2025), may or may not require update, depends on testing
        d.altitude = 145366.45f * (1.0f - powf(ratio, 0.190284f));
    }
    *out = d;
    return 1;
}

/*
 * Find the current dma write index
 */
static inline uint32_t dma_widx(void)
{
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(s_uart->hdmarx);
    uint32_t idx  = IMU_RX_BUF_SIZE - ndtr;
    return (idx == IMU_RX_BUF_SIZE) ? 0 : idx;
}

/*
 * Process data on RX line
 */
static void process_rx(void)
{
    uint32_t w = dma_widx(); //find current dma write index
    uint32_t i = s_dma_last;
    //calculate available bits
    uint32_t avail = (w >= i) ? (w - i) : (IMU_RX_BUF_SIZE - i + w);

    while (avail) {
    	//look for frame sync byte (0xFA), if current byte isn't a sync, skip and keep scanning
        if (s_rx[i] != IMU_FRAME_SYNC) {
        	i = (i + 1) % IMU_RX_BUF_SIZE; //move to the next byte and wrap if needed
        	--avail;
        	continue;
        }

        //at this point we've found a potential frame
        if (avail < IMU_FRAME_TOTAL_LEN) break; //break if not enough bytes

        uint8_t raw[IMU_FRAME_AFTERSYNC_LEN];

        //copy the next 35 bytes after sync byte (0xFA) into a linear temp array
        for (uint32_t j = 0; j < IMU_FRAME_AFTERSYNC_LEN; ++j)
            raw[j] = s_rx[(i + 1 + j) % IMU_RX_BUF_SIZE];

        //process one frame of data
        IMU_Data_t tmp;

        if (process_one_imu_frame(raw, &tmp)) {
            uint8_t back = 1u - s_front;
            s_db[back] = tmp;  	// copy whole struct
            s_front = back; 	// flip front index (atomic publish)
            s_has_new = 1;
            i = (i + IMU_FRAME_TOTAL_LEN) % IMU_RX_BUF_SIZE;
            avail = (w >= i) ? (w - i) : (IMU_RX_BUF_SIZE - i + w); //recalculate available bytes
        } else {
            i = (i + 1) % IMU_RX_BUF_SIZE;
            --avail;
        }
    }
    s_dma_last = i; //update index
}

/* ---- IMU commands ---- */
static void imu_send_init(void)
{
    const char cmd[] = "$VNWRG,75,2,8,05,0108,0020*XX\r\n";
    /*
	 *  $VNWRG - write register
		75 		- register 75 is "binary output message configuration"
		2 		- message sent out on port 2. port 2 uses 3V logic levels. port 1 uses 12V
		8 		- rate divisor. 800 / 8 = 100 Hz
		05 		- selecting groups 1 and 3
		0108 	- selecting YPR and Acceleration from group 1. convert binary to hex
				   (see table 17)
		0020 	- selecting pres from group 3. convert binary to hex
				   (see table 17)
	 */
    HAL_UART_Transmit(s_uart, (uint8_t*)cmd, sizeof(cmd)-1, 50);
    HAL_Delay(20);
}
static void imu_reset(void)
{
    const char cmd_reset[] = "$VNRFS*XX\r\n"; //reset imu to factory
    HAL_UART_Transmit(s_uart, (uint8_t*)cmd_reset, sizeof(cmd_reset)-1, 50);
    HAL_Delay(500);
    const char cmd_pause[] = "$VNASY,0*XX\r\n"; //pause default async reading after IMU factory reset
    HAL_UART_Transmit(s_uart, (uint8_t*)cmd_pause, sizeof(cmd_pause)-1, 50);
    HAL_Delay(500);
}

/* -------- Public API -------- */

int imu_setup(void *huart)
{
    s_uart = (UART_HandleTypeDef*)huart;
    if (!s_uart) return -1;

    /* Configure IMU and start RX DMA*/
    imu_reset();
    HAL_Delay(20);
    imu_send_init();
    HAL_Delay(20);

    // Start circular RX DMA + enable IDLE interrupt
	if (HAL_UART_Receive_DMA(s_uart, s_rx, IMU_RX_BUF_SIZE) != HAL_OK) return -1;
	__HAL_UART_CLEAR_FLAG(s_uart, UART_CLEAR_IDLEF);   // H7: clear via ICR
	__HAL_UART_ENABLE_IT(s_uart, UART_IT_IDLE);
	return 0;
}

int imu_read(IMU_Data_t *out)
{
    if (!out) return 0;
    if (!s_has_new) return 0;
    *out = s_db[s_front];
    s_has_new = 0;
    return 1;
}

/* ===== IRQ hook (call from USARTx_IRQHandler) ===== */
void imu_on_usart_irq(void *huart) {
    if ((UART_HandleTypeDef*)huart != s_uart) return;

    if (__HAL_UART_GET_FLAG(s_uart, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(s_uart, UART_IT_IDLE)) {
        __HAL_UART_CLEAR_FLAG(s_uart, UART_CLEAR_IDLEF);
        process_rx();
    }
}

