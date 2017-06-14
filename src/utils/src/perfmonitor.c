/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * perfmon.c - Implementation of logging variables for system performance monitoring
 */

#include "log.h"

// Measures the effective loop time of the stabilizer task - found in stabilizer.c
extern uint32_t stabilizerLoopTimeUs;

// Measures the end-to-end latency of the IMU data from when it's read off the part
// to when the motors are updated using the read values - found in stabilizer.c
extern uint32_t sensorToOutputLatencyUs;

// Measures the effective loop time of the estimator - found in estimator_complementary.c
extern uint32_t estimatorLoopTimeUs;

// Measures the latency of the IMU data from when it's read off the part to when
// the orientation estimator consumes the data - found in estimator_complementary.c
extern uint32_t sensorToEstLatencyUs;

// Measures the effective loop time of the attitude PID - found in controller_pid.c
extern uint32_t attitudePIDLoopTimeUs;

// Measures the latency of the IMU data from when it's read off the part to when
// the attitude PID consumes the data - found in controller_pid.c
extern uint32_t sensorToAttitudePIDLatencyUs;

// Measures the effective loop time of the rate PID - found in controller_pid.c
extern uint32_t ratePIDLoopTimeUs;

// Measures the latency of the IMU data from when it's read off the part to when
// the rate PID consumes the data - found in controller_pid.c
extern uint32_t sensorToRatePIDLatencyUs;

// Measures the effective loop time of the sensor data ready line - found in sensors_cf2.c
extern uint32_t sensorDataReadyLoopTimeUs;

LOG_GROUP_START(perfmon)
LOG_ADD(LOG_UINT32, stabLoopTimeUs, &stabilizerLoopTimeUs)
LOG_ADD(LOG_UINT32, sensorToOutputUs, &sensorToOutputLatencyUs)
LOG_ADD(LOG_UINT32, estLoopTimeUs, &estimatorLoopTimeUs)
LOG_ADD(LOG_UINT32, sensorToEstUs, &sensorToEstLatencyUs)
LOG_ADD(LOG_UINT32, attPIDLoopTimeUs, &attitudePIDLoopTimeUs)
LOG_ADD(LOG_UINT32, sensorToAttUs, &sensorToAttitudePIDLatencyUs)
LOG_ADD(LOG_UINT32, ratePIDLoopTimeUs, &ratePIDLoopTimeUs)
LOG_ADD(LOG_UINT32, sensorToRateUs, &sensorToRatePIDLatencyUs)
LOG_ADD(LOG_UINT32, sensorLoopTimeUs, &sensorDataReadyLoopTimeUs)
LOG_GROUP_STOP(perfmon)