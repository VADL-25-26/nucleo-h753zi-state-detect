/*
 * imu.h
 *
 *  Created on: Sep 14, 2025
 *      Author: fuash
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw, pitch, roll;  // deg
    float accelX, accelY, accelZ;     // G
    float pressure;          // kPa
    float altitude;          // ft
} IMU_Data_t;

/* Initialize/configure IMU and start RX DMA.
 * Pass the UART handle that talks to the IMU.
 * Returns 0 on success, -1 on error.
 */
int imu_setup(void *huart /* UART_HandleTypeDef* */);

/* Non-blocking: copy the latest fully-parsed measurement.
 * Returns 1 if a fresh sample was copied into *out, else 0.
 */
int imu_read(IMU_Data_t *out);

/*
 * Call this from USARTx_IRQHandler (whichever imu usart being used)
 */
void imu_on_usart_irq(void *huart /* UART_HandleTypeDef* */);

#ifdef __cplusplus
}
#endif


#endif /* INC_IMU_H_ */
