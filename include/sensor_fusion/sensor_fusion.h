/**
 * @file sensor_fusion.h
 *
 * @brief Sensor fusion device driver custom API.
 *
 * Copyright (c) 2024 Tomislav Milkovic
 * SPDX-License-Identifier: MIT
 */

#ifndef __SENSOR_FUSION_H__
#define __SENSOR_FUSION_H__

#ifdef __cplusplus
extern "C"
{
#endif

//--------------------------------- INCLUDES ----------------------------------
#include <zephyr/drivers/sensor.h>

//---------------------------------- MACROS -----------------------------------
//-------------------------------- DATA TYPES ---------------------------------
/**
 * @brief Sensor fusion channels.
 */
enum sensor_fusion_channel
{
    /** Euler angle for yaw, in degrees. */
    SENSOR_FUSION_CHAN_YAW = SENSOR_CHAN_PRIV_START,
    /** Euler angle for pitch, in degrees. */
    SENSOR_FUSION_CHAN_PITCH,
    /** Euler angle for roll, in degrees. */
    SENSOR_FUSION_CHAN_ROLL,
    /** Euler angle for all three axes: yaw, pitch and roll, in degrees. */
    SENSOR_FUSION_CHAN_YAW_PITCH_ROLL,
};
//---------------------- PUBLIC FUNCTION PROTOTYPES ---------------------------

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_FUSION_H__
