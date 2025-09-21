/*
 * flight_state.h
 *
 *  Created on: Sep 12, 2025
 *      Author: Owner
 */

#ifndef INC_FLIGHT_STATE_H_
#define INC_FLIGHT_STATE_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"
#include "imu.h"
#include "kalman.h"

typedef enum {
    FLIGHT_STATE_GROUND_IDLE,     /* Rocket on ground, pre-launch */
    FLIGHT_STATE_POWERED_FLIGHT,  /* Motor burning, accelerating */
    FLIGHT_STATE_UNPOWERED_FLIGHT,/* Coasting to apogee */
    FLIGHT_STATE_DESCENT,         /* Descending under parachute */
    FLIGHT_STATE_LANDED           /* Landed, mission complete */
} FlightState_t;

/* Flight detection thresholds */
#define LAUNCH_ACCELERATION_THRESHOLD  1.25f   /* Acceleration threshold in g */
#define LAUNCH_DETECTION_COUNT         30      /* Number of consecutive readings above threshold */
#define APOGEE_DETECTION_COUNT         50      /* Readings with negative velocity for apogee */
#define LANDING_DETECTION_COUNT        1000    /* Readings with no new minimum altitude for landing */
#define MIN_ALTITUDE_FOR_APOGEE        100.0f  /* Minimum altitude (ft) for apogee detection */
#define LAUNCH_LANDING_ALT_DELTA	   20.0f   /* Maximum altitude (ft) difference between launch and landing. Used for landing detection */
#define MAIN_CHUTE_ALTITUDE            500.0f  /* Altitude (ft) for main chute deployment */
#define MOTOR_BURN_TIME_MS             1000    /* Motor burn time in milliseconds */

void FlightState_Init(void);
void FlightState_Update(void);
FlightState_t FlightState_GetCurrent(void);
float FlightState_GetGroundAltitude(void);
uint32_t FlightState_GetLaunchTime(void);
uint32_t FlightState_GetApogeeTime(void);
uint32_t FlightState_GetLandingTime(void);
float FlightState_GetMaxAltitude(void);
float FlightState_GetMaxVelocity(void);

#endif /* INC_FLIGHT_STATE_H_ */
