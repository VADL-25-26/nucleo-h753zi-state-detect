/*
 * imu_sim.c
 *
 *  Created on: Sep 12, 2025
 *      Author: Owner
 */


#include "imu.h"

// simple scripted profile: PAD 2s → POWER 1s → COAST 3s → DESCENT 5s → LANDED
static volatile IMU_Data_t g_latest;
volatile uint32_t t_ms = 0;
static float ground_ft = 0.0f;

void IMU_Sim_Tick_200Hz(void) {
  t_ms += 5;
  float ax_g = 0.0f, alt_ft = ground_ft;

  if (t_ms < 2000) {                // PAD
    ax_g = 0.0f;  alt_ft = ground_ft;
  } else if (t_ms < 3000) {         // POWERED
    ax_g = 2.0f;  alt_ft = ground_ft + 200.0f + 50.0f * ((t_ms-2000)/1000.0f);
  } else if (t_ms < 6000) {         // UNPOWERED/COAST
    ax_g = 0.0f;  alt_ft = ground_ft + 300.0f;
  } else if (t_ms < 11000) {        // DESCENT
    ax_g = -0.2f; alt_ft = ground_ft + 300.0f - 0.05f*(t_ms-6000);
  } else {                          // LANDED
    ax_g = 0.0f;  alt_ft = ground_ft + 30.0f;
  }

  g_latest.accelX = ax_g;           // we only use accelX for launch detection in legacy code
  g_latest.accelY = 0.0f;
  g_latest.accelZ = 1.0f;           // keep ~1g if you need it later
  g_latest.altitude = alt_ft;
}

float IMU_Sim_CalculateGroundAltitude(void) { ground_ft = 500.0f; return ground_ft; }
IMU_Data_t IMU_Sim_GetLatestData(void) { return g_latest; }
