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
    /** linear acceleration measurement equal to the accelerometer
        measurement with the 1 g of gravity removed. */
    SENSOR_FUSION_CHAN_LINEAR_ACCEL_XYZ,
    /** Earth acceleration measurement equal to accelerometer measurement
        in the Earth coordinate frame with the 1 g of gravity removed. */
    SENSOR_FUSION_CHAN_EARTH_ACCEL_XYZ,

};

/**
 * @brief Sensor fusion attribute types.
 */
enum sensor_fusion_attribute {
    /** Magnetometer hard ironing 3D vector, in X-Y-Z order */
    SENSOR_FUSION_ATTR_MAGN_HARD_IRON_VECTOR = SENSOR_ATTR_PRIV_START,
    /** Magnetometer soft ironing 3x3 matrix, in row-major order */
    SENSOR_FUSION_ATTR_MAGN_SOFT_IRON_MATRIX,
};

//---------------------- PUBLIC FUNCTION PROTOTYPES ---------------------------

#ifdef __cplusplus
}
#endif

#endif // __SENSOR_FUSION_H__
