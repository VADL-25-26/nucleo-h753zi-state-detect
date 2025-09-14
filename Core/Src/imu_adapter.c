/*
 * imu_adapter.c
 * Interface adapter for IMU implementations (sim and real)
 *
 *  Created on: Sep 12, 2025
 *      Author: Owner
 */

#include "imu.h"
//#include "imu_real.h"
#include "imu_source.h"

/* Calculate ground altitude */
float IMU_CalculateGroundAltitude(void) {
//    if (g_imu_source == IMU_SOURCE_REAL) {
//        return IMU_Real_CalculateGroundAltitude();
//    } else {
        // Use the simulated ground altitude
//        extern float IMU_Sim_CalculateGroundAltitude(void);
//        return IMU_Sim_CalculateGroundAltitude();
//    }
	extern float IMU_Sim_CalculateGroundAltitude(void);
	return IMU_Sim_CalculateGroundAltitude();
}

/* Get latest IMU data from selected source */
IMU_Data_t IMU_GetLatestData(void) {
//    if (g_imu_source == IMU_SOURCE_REAL) {
//        return IMU_Real_GetLatestData();
//    } else {
//        // Use the simulated data
//        extern IMU_Data_t IMU_Sim_GetLatestData(void);
//        return IMU_Sim_GetLatestData();
//    }
    extern IMU_Data_t IMU_Sim_GetLatestData(void);
	return IMU_Sim_GetLatestData();
}
