/*
 * imu_source.h
 * Definition for IMU source selection
 *
 *  Created on: Sep 12, 2025
 *      Author: Owner
 */

#ifndef INC_IMU_SOURCE_H_
#define INC_IMU_SOURCE_H_

/* Definition for IMU source selection */
typedef enum {
  IMU_SOURCE_SIMULATED,
  IMU_SOURCE_REAL
} IMU_Source_t;

/* Global variable declaration - defined in main.c */
extern IMU_Source_t g_imu_source;

#endif /* INC_IMU_SOURCE_H_ */
