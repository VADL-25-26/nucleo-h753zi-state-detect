/*
 * imu.h
 * Just a stub for testing purposes - waiting for actual implementation from Ashley
 *
 *  Created on: Sep 12, 2025
 *      Author: Owner
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#pragma once
#include <stdint.h>

typedef struct {
  float accelX, accelY, accelZ;   // g (as your thresholds assume)
  float altitude;                 // feet (matches header constants tonight)
} IMU_Data_t;

float IMU_CalculateGroundAltitude(void);
IMU_Data_t IMU_GetLatestData(void);


#endif /* INC_IMU_H_ */
