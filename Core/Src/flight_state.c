/*
 * flight_state.c
 * Converted from ACS flight state detection program
 *
 *  Created on: Sep 12, 2025
 *      Author: Neal B
 */


#include "flight_state.h"


/* State variables */
static FlightState_t currentState = FLIGHT_STATE_GROUND_IDLE;
static float groundAltitude = 0.0f;
static uint32_t launchTime = 0;
static uint32_t apogeeTime = 0;
static uint32_t landingTime = 0;
static uint8_t consecutiveReadingsLaunch = 0;
static uint8_t consecutiveReadingsApogee = 0;
static uint16_t consecutiveReadingsLanding = 0;
static float previousAltitude = 0.0f;


/* Init flight state machine */
void FlightState_Init(void) {
    currentState = FLIGHT_STATE_GROUND_IDLE;
    groundAltitude = IMU_CalculateGroundAltitude();

}

void FlightState_Update(IMU_Data_t imuData) {
//	IMU_Data_t imuData;
//	imu_read(&imuData);

    float altitude = imuData.altitude;
    float filteredAltitude, filteredVelocity;

    KalmanFilter_Update(altitude, &filteredAltitude, &filteredVelocity);

    /*Ground Logic*/
    if (currentState == FLIGHT_STATE_GROUND_IDLE) {
        if (fabsf(imuData.accelX) > LAUNCH_ACCELERATION_THRESHOLD) {
            consecutiveReadingsLaunch++;
        } else {
            consecutiveReadingsLaunch = 0;
        }

        if (consecutiveReadingsLaunch >= LAUNCH_DETECTION_COUNT) {
            currentState = FLIGHT_STATE_POWERED_FLIGHT;
            launchTime = HAL_GetTick(); // or whatever function polls the RTS on the STM32
            // DataLogger_LogEvent("Launch Detected");
        }
    }

    /*Apogee Detection*/
    if (currentState == FLIGHT_STATE_POWERED_FLIGHT || currentState == FLIGHT_STATE_UNPOWERED_FLIGHT) {

        /*State Change from powered to unpowered ascent*/
        if (currentState == FLIGHT_STATE_POWERED_FLIGHT &&
            (HAL_GetTick() - launchTime) > MOTOR_BURN_TIME_MS)
            currentState = FLIGHT_STATE_UNPOWERED_FLIGHT;
            //DataLogger_LogEvent("Motor Burnout")

        /*Check for Apogee*/
        if (filteredVelocity < 0.0f && altitude > (groundAltitude + MIN_ALTITUDE_FOR_APOGEE)) {
            consecutiveReadingsApogee++;
        } else {
            consecutiveReadingsApogee = 0;
        }

        if (consecutiveReadingsApogee >= APOGEE_DETECTION_COUNT) {
            currentState = FLIGHT_STATE_DESCENT;
            apogeeTime = HAL_GetTick();
            //DataLogger_LogEvent("Apogee Detected");
            previousAltitude = altitude;
        }
    }

    /*Landing Detection*/
    if (currentState == FLIGHT_STATE_DESCENT) {

        /*Check for Main Chute Deployment*/
        // if (currentState == FLIGHT_STATE_DROGUE_DESCENT && altitude < (groundAltitude + MAIN_CHUTE_ALTITUDE)) {
        //     currentState = FLIGHT_STATE_MAIN_DESCENT;
        //     //DataLogger_LogEvent("Main Descent Detected")
        // }

        /*Landed State Condition - No New Altitude in (x) Cycles. Altitude within delta of launch altitude.*/
        if (altitude < previousAltitude && fabsf(altitude - groundAltitude) < LAUNCH_LANDING_ALT_DELTA) {
            previousAltitude = altitude;
            consecutiveReadingsLanding = 0;
        } else {
            consecutiveReadingsLanding++;
        }

        if (consecutiveReadingsLanding > LANDING_DETECTION_COUNT) {
            currentState = FLIGHT_STATE_LANDED;
            // landingTime = HAL_GetTick();
            // DataLogger_LogEvent("Landing Detected");

            // PayloadActivate();
        }
    }

//    DataLogger_LogState(currentState, altitude, filteredAltitude, filteredVelocity);
}

/* Get the current flight state */
FlightState_t FlightState_GetCurrent(void) {
    return currentState;
}

/* Get the ground altitude */
float FlightState_GetGroundAltitude(void) {
    return groundAltitude;
}

/* Get the launch time */
uint32_t FlightState_GetLaunchTime(void) {
    return launchTime;
}

/* Get the apogee time */
uint32_t FlightState_GetApogeeTime(void) {
    return apogeeTime;
}

/* Get the landing time */
uint32_t FlightState_GetLandingTime(void) {
    return landingTime;
}

/* Variables for max altitude and velocity tracking */
static float maxAltitude = 0.0f;
static float maxVelocity = 0.0f;

/* Get the maximum altitude recorded */
float FlightState_GetMaxAltitude(void) {
    return maxAltitude;
}

/* Get the maximum velocity recorded */
float FlightState_GetMaxVelocity(void) {
    return maxVelocity;
}
