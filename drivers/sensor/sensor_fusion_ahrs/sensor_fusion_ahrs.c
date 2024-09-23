/**
 * @file sensor_fusion_ahrs.c
 *
 * @brief Module which integrates AHRS sensor fusion
 *        library from XIO Technologies into Zephyr.
 *
 * Copyright (c) 2024 Tomislav Milkovic
 * SPDX-License-Identifier: MIT
 */

//--------------------------------- INCLUDES ----------------------------------
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <sensor_fusion/sensor_fusion.h>
#include <FusionAhrs.h>

//---------------------------------- MACROS -----------------------------------
#define DT_DRV_COMPAT zephyr_sensor_fusion_ahrs

LOG_MODULE_REGISTER(sensor_fusion_ahrs, CONFIG_SENSOR_FUSION_AHRS_LOG_LEVEL);

//-------------------------------- DATA TYPES ---------------------------------
struct sensor_fusion_ahrs_data
{
        FusionAhrs ahrs;
        FusionEuler euler;
        uint32_t last_fetch_timestamp;
};

struct sensor_fusion_ahrs_config
{
        struct device *accel_dev;
        struct device *gyro_dev;
        struct device *magn_dev;
};

//---------------------- PRIVATE FUNCTION PROTOTYPES --------------------------

//------------------------- STATIC DATA & CONSTANTS ---------------------------

//------------------------------- GLOBAL DATA ---------------------------------

//------------------------------ PUBLIC FUNCTIONS -----------------------------

//---------------------------- PRIVATE FUNCTIONS ------------------------------
static int sensor_fusion_ahrs_sample_fetch(const struct device *dev,
                                           enum sensor_channel chan)
{
        struct sensor_fusion_ahrs_data *data = dev->data;
        const struct sensor_fusion_ahrs_config *config = dev->config;
        struct sensor_value imu_values[9];
        FusionVector accel_vector;
        FusionVector gyro_vector;
        FusionVector magn_vector;
        float delta_time_seconds;
        uint32_t new_fetch_timestamp;

        __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

        if (sensor_sample_fetch(config->accel_dev))
        {
                LOG_ERR("Accelerometer data fetch failed");
        }
        else
        {
                err = sensor_channel_get(config->accel_dev, SENSOR_CHAN_ACCEL_XYZ, &imu_values[0]);
        }

        if (config->gyro_dev != config->accel_dev)
        {
                if (sensor_sample_fetch(config->gyro_dev))
                {
                        LOG_ERR("Gyroscope data fetch failed");
                }
                else
                {
                        err = sensor_channel_get(config->gyro_dev, SENSOR_CHAN_GYRO_XYZ, &imu_values[3]);
                }
        }
        else
        {
                err = sensor_channel_get(config->gyro_dev, SENSOR_CHAN_GYRO_XYZ, &imu_values[3]);
        }

        if (config->magn_dev)
        {
                if ((config->magn_dev != config->accel_dev) && (config->magn_dev != config->gyro_dev))
                {
                        if (sensor_sample_fetch(config->magn_dev))
                        {
                                LOG_ERR("Magnetometer data fetch failed");
                        }
                        else
                        {
                                err = sensor_channel_get(config->magn_dev, SENSOR_CHAN_MAGN_XYZ, &imu_values[6]);
                        }
                }
                else
                {
                        err = sensor_channel_get(config->magn_dev, SENSOR_CHAN_MAGN_XYZ, &imu_values[6]);
                }
        }

        /* Update fetch timestamps */
        new_fetch_timestamp = k_uptime_get();
        delta_time_seconds =
            (new_fetch_timestamp - data->last_fetch_timestamp) / 1000.0f;
        data->last_fetch_timestamp = new_fetch_timestamp;

        /* Convert sensor values to floats */
        accel_vector.axis.x = sensor_value_to_float(&imu_values[0]);
        accel_vector.axis.y = sensor_value_to_float(&imu_values[1]);
        accel_vector.axis.z = sensor_value_to_float(&imu_values[2]);
        gyro_vector.axis.x = (float)sensor_rad_to_degrees(&imu_values[3]);
        gyro_vector.axis.y = (float)sensor_rad_to_degrees(&imu_values[4]);
        gyro_vector.axis.z = (float)sensor_rad_to_degrees(&imu_values[5]);
        magn_vector.axis.x = sensor_value_to_float(&imu_values[6]);
        magn_vector.axis.y = sensor_value_to_float(&imu_values[7]);
        magn_vector.axis.z = sensor_value_to_float(&imu_values[8]);

        if (config->magn_dev)
        {
                FusionAhrsUpdate(&data->ahrs,
                                 gyro_vector,
                                 accel_vector,
                                 magn_vector,
                                 delta_time_seconds);
        }
        else
        {
                FusionAhrsUpdateNoMagnetometer(&data->ahrs,
                                               gyro_vector,
                                               accel_vector,
                                               delta_time_seconds);
        }

        /* Calculate euler angles */
        data->euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&data->ahrs));

        return 0;
}

static int sensor_fusion_ahrs_channel_get(const struct device *dev,
                                          enum sensor_channel chan,
                                          struct sensor_value *val)
{
        struct sensor_fusion_ahrs_data *data = dev->data;

        if (chan == SENSOR_FUSION_CHAN_YAW)
        {
                sensor_value_from_float(val, data->euler.angle.yaw);
        }
        else if (chan == SENSOR_FUSION_CHAN_PITCH)
        {
                sensor_value_from_float(val, data->euler.angle.pitch);
        }
        else if (chan == SENSOR_FUSION_CHAN_ROLL)
        {
                sensor_value_from_float(val, data->euler.angle.roll);
        }
        else if (chan == SENSOR_FUSION_CHAN_YAW_PITCH_ROLL)
        {
                sensor_value_from_float(&val[0], data->euler.angle.yaw);
                sensor_value_from_float(&val[1], data->euler.angle.pitch);
                sensor_value_from_float(&val[2], data->euler.angle.roll);
        }
        else
        {
                return -ENOTSUP;
        }

        return 0;
}

static const struct sensor_driver_api sensor_fusion_ahrs_driver_api = {
    .sample_fetch = sensor_fusion_ahrs_sample_fetch,
    .channel_get = sensor_fusion_ahrs_channel_get,
};

int sensor_fusion_ahrs_init(const struct device *dev)
{
        struct sensor_fusion_ahrs_data *data = dev->data;
        const struct sensor_fusion_ahrs_config *config = dev->config;

        /* Check if accelerometer is ready */
        if (!device_is_ready(config->accel_dev))
        {
                LOG_ERR("Accelerometer device not ready!")
                return -ENODEV;
        }

        /* Check if gyroscope is ready */
        if (!device_is_ready(config->gyro_dev))
        {
                LOG_ERR("Gyroscope device not ready!")
                return -ENODEV;
        }

        /* Check if magnetometer is used and if it is ready */
        if ((config->magn_dev) && (!device_is_ready(config->magn_dev)))
        {
                LOG_ERR("Magnetometer device not ready!")
                return -ENODEV;
        }

        /* Initialise the AHRS algorithm structure */
        FusionAhrsInitialise(&data->ahrs);

        data->last_fetch_timestamp = 0;

        return 0;
}

//---------------------------- INTERRUPT HANDLERS -----------------------------

#define SENSOR_FUSION_AHRS_DEFINE(inst)                                                   \
        static struct sensor_fusion_ahrs_data sensor_fusion_ahrs_data_##inst;             \
                                                                                          \
        static const struct sensor_fusion_ahrs_config sensor_fusion_ahrs_config##inst = { \
            .accel_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), accel_dev)),         \
            .gyro_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), gyro_dev)),           \
            .magn_dev = DEVICE_DT_GET(DT_PHANDLE(DT_DRV_INST(inst), magn_dev))};          \
                                                                                          \
        SENSOR_DEVICE_DT_INST_DEFINE(inst, sensor_fusion_ahrs_init, NULL,                 \
                                     &sensor_fusion_ahrs_data_##inst,                     \
                                     &sensor_fusion_ahrs_config##inst, POST_KERNEL,       \
                                     CONFIG_SENSOR_INIT_PRIORITY,                         \
                                     &sensor_fusion_ahrs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_FUSION_AHRS_DEFINE)
