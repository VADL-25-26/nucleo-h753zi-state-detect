/*
 * imu_real.c
 *  Adaptation of Ashley's main function implementation of the IMU
 *  into a standalone source file. Alternative to simulated IMU.
 *  Created on: Sep 12, 2025
 *      Author: Neal
 * 
 */

#include "imu_real.h"
#include <string.h>
#include <math.h>

/* Private defines */
#define RX_BUF_SIZE 512
#define FRAME_SYNC     0xFA
#define FRAME_AFTERSYNC_LEN 35
#define FRAME_TOTAL_LEN     (1 + FRAME_AFTERSYNC_LEN)

/* Private variables */
static UART_HandleTypeDef *p_huart;
static DMA_HandleTypeDef *p_hdma_rx;

static uint8_t rx_dma_buf[RX_BUF_SIZE];
static volatile uint32_t dma_last = 0;
static volatile uint8_t rx_idle_flag = 0;

typedef struct {
    float yaw, pitch, roll; //degree
    float a_x, a_y, a_z; //F
    float pressure; //kPa
    float altitude; //feet
} IMUData;

/* Double buffer for parsed IMUdata 
   We can safely update one without corrupting the one the main loop is reading */
static IMUData imu_buf[2];
static volatile uint8_t imu_front = 0;

/* Latest IMU data in format compatible with flight_state */
static volatile IMU_Data_t g_latest;

/* Private function prototypes */
static inline uint32_t dma_write_idx(void);
static uint16_t vn_crc16(const uint8_t *data, uint32_t len);
static float little_endian_2_float(const uint8_t *p);
static int process_one_imu_frame(const uint8_t *m, IMUData *out);
static void imu_process_rx(void);

/* Function implementations */

/* Initialize IMU with UART and DMA handles */
void IMU_Real_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma) {
    p_huart = huart;
    p_hdma_rx = hdma;
}

/* Start IMU DMA reception */
void IMU_Real_Start(void) {
    /* Clear idle flag and enable interrupt */
    __HAL_UART_CLEAR_IDLEFLAG(p_huart);
    __HAL_UART_ENABLE_IT(p_huart, UART_IT_IDLE);
    
    /* Start DMA reception */
    HAL_UART_Receive_DMA(p_huart, rx_dma_buf, RX_BUF_SIZE);
    
    /* Send initialization command to IMU */
    const char cmd[] = "$VNWRG,75,2,4,05,0108,0020*XX\r\n";
    HAL_UART_Transmit(p_huart, (uint8_t*)cmd, sizeof(cmd)-1, 50);
}

/* Process received IMU data - call from main loop */
void IMU_Real_Process(void) {
    /* Check if new data is available */
    if (rx_idle_flag) {
        rx_idle_flag = 0;
        imu_process_rx();
    }
}

/* Calculate ground altitude from IMU data */
float IMU_Real_CalculateGroundAltitude(void) {
    /* For real implementation, you might want to average multiple readings */
    IMU_Data_t data = IMU_Real_GetLatestData();
    return data.altitude;
}

/* Get latest IMU data */
IMU_Data_t IMU_Real_GetLatestData(void) {
    return g_latest;
}

/* UART Idle Callback - to be called from UART IRQ Handler */
void IMU_Real_UART_IdleCallback(void) {
    rx_idle_flag = 1;
}

/* Private functions */

/* Find the last written position in DMA buffer */
static inline uint32_t dma_write_idx(void) {
    /* NDTR counts down */
    return RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(p_huart->hdmarx);
}

/* Checksum calculation (from data sheet) */
static uint16_t vn_crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t b = data[i];
        crc ^= (uint16_t)b << 8;
        for (uint32_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* Convert little endian bytes to float */
static float little_endian_2_float(const uint8_t *p) {
    float f;
    memcpy(&f, p, sizeof(float)); /* Convert STM little endian to float */
    return f;
}

/* Parse one frame of IMU data */
static int process_one_imu_frame(const uint8_t *m, IMUData *out) {
    /* Verify checksum */
    uint16_t calc = vn_crc16(m, 33);
    uint16_t recv = (uint16_t)((m[33] << 8) | m[34]); /* Big-endian */
    if (calc != recv) return 0;

    /* Slice data */
    float yaw      = little_endian_2_float(&m[5]);
    float pitch    = little_endian_2_float(&m[9]);
    float roll     = little_endian_2_float(&m[13]);
    float ax       = little_endian_2_float(&m[17]);
    float ay       = little_endian_2_float(&m[21]);
    float az       = little_endian_2_float(&m[25]);
    float pressure = little_endian_2_float(&m[29]);

    /* Add data to IMU struct */
    IMUData d;
    d.yaw = yaw;
    d.pitch = pitch;
    d.roll = roll;
    const float g = 9.80665f; /* Gravity constant */
    d.a_x = ax / g;
    d.a_y = ay / g;
    d.a_z = az / g;
    d.pressure = pressure;

    /* Calculate altitude from pressure */
    if (pressure <= 0.0f) {
        d.altitude = NAN;
    } else {
        /* Standard atmospheric pressure at sea level in kPa */
        const float p0 = 101.325f;
        /* Calculate altitude using barometric formula */
        d.altitude = 145366.45f * (1.0f - powf(pressure / p0, 0.190284f));
    }

    *out = d;
    
    /* Update the global latest data struct for flight state */
    g_latest.accelX = d.a_x;
    g_latest.accelY = d.a_y;
    g_latest.accelZ = d.a_z;
    g_latest.altitude = d.altitude;
    
    return 1;
}

/* Process received data from DMA buffer */
static void imu_process_rx(void) {
    /* Find where new data has been written up to (write pointer) */
    uint32_t write = dma_write_idx();

    /* Calculate how much available bytes 
       If write >= dma_last (since last time we processed), no wraparound
       Else, buffer wrapped */
    uint32_t available_bytes = (write >= dma_last) ? 
                              (write - dma_last) : 
                              (RX_BUF_SIZE - dma_last + write);

    uint32_t i = dma_last;

    while (available_bytes) {
        /* Look for sync byte */
        if (rx_dma_buf[i] == FRAME_SYNC && available_bytes >= FRAME_TOTAL_LEN) {
            /* We have a full frame, process it */
            IMUData new_data;
            if (process_one_imu_frame(&rx_dma_buf[i+1], &new_data)) {
                /* Update the buffer that's not currently being read from */
                uint8_t back = 1 - imu_front;
                imu_buf[back] = new_data;
                /* Atomic swap of buffer index */
                imu_front = back;
            }
            
            /* Skip ahead past this frame */
            i = (i + FRAME_TOTAL_LEN) % RX_BUF_SIZE;
            available_bytes -= FRAME_TOTAL_LEN;
        } else {
            /* No sync byte found, move to next byte */
            i = (i + 1) % RX_BUF_SIZE;
            available_bytes--;
        }
    }

    dma_last = i;
}
