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
    FusionOffset gyro_offset_vector;
    FusionVector magn_calib_hard_iron;
    FusionMatrix magn_calib_soft_iron;
    int64_t last_fetch_timestamp;
};

struct sensor_fusion_ahrs_config
{
    struct device *accel_dev;
    struct device *gyro_dev;
    struct device *magn_dev;
    FusionAhrsSettings ahrs_settings;
    uint32_t target_sample_rate_hz;
};

//---------------------- PRIVATE FUNCTION PROTOTYPES --------------------------

//------------------------- STATIC DATA & CONSTANTS ---------------------------

//------------------------------- GLOBAL DATA ---------------------------------

//------------------------------ PUBLIC FUNCTIONS -----------------------------

//---------------------------- PRIVATE FUNCTIONS ------------------------------
static int sensor_fusion_ahrs_trim_low_imu_values(FusionVector *accel_vector,
                                                  FusionVector *gyro_vector)
{
    uint8_t i;

    /* Trim accelerometer values around 0 to 0 */
    for (i = 0; i < 3; i++)
    {
        if ((accel_vector->array[i] < (CONFIG_SENSOR_FUSION_AHRS_ACCEL_ZERO_TRIM_THRESHOLD_MILLIG / 1000.0f)) && (accel_vector->array[i] > (-CONFIG_SENSOR_FUSION_AHRS_ACCEL_ZERO_TRIM_THRESHOLD_MILLIG / 1000.0f)))
        {
            accel_vector->array[i] = 0.0f;
        }
    }

    /* Trim gyroscope values around 0 to 0 */
    for (i = 0; i < 3; i++)
    {
        if ((gyro_vector->array[i] < (CONFIG_SENSOR_FUSION_AHRS_GYRO_ZERO_TRIM_THRESHOLD_MILLIDEGREES / 1000.0f)) && (gyro_vector->array[i] > (-CONFIG_SENSOR_FUSION_AHRS_GYRO_ZERO_TRIM_THRESHOLD_MILLIDEGREES / 1000.0f)))
        {
            gyro_vector->array[i] = 0.0f;
        }
    }

    return 0;
}

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

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

    if (sensor_sample_fetch(config->accel_dev))
    {
        LOG_ERR("Accelerometer data fetch failed");
    }
    else
    {
        err = sensor_channel_get(config->accel_dev, SENSOR_CHAN_ACCEL_XYZ, &imu_values[0]);
    }

    /* Check if combined device is used (gyro and accel are in the same IC) */
    if (config->gyro_dev != config->accel_dev)
    {
        /* Fetch gyroscope sample separately */
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
        /* Gyro sample already fetched because gyro and accel are in the same IC */
        err = sensor_channel_get(config->gyro_dev, SENSOR_CHAN_GYRO_XYZ, &imu_values[3]);
    }

    /* Check if magnetometer is used for sensor fusion */
    if (config->magn_dev)
    {
        /* Check if combined device is used (magn and accel, or magn and gyro are in the same IC) */
        if ((config->magn_dev != config->accel_dev) && (config->magn_dev != config->gyro_dev))
        {
            /* Fetch magnetometer sample separately */
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
            /* Magn sample already fetched because magn and accel, or magn and gyro are in the same IC */
            err = sensor_channel_get(config->magn_dev, SENSOR_CHAN_MAGN_XYZ, &imu_values[6]);
        }
    }

    /* Update fetch timestamps */
    delta_time_seconds = k_uptime_delta(&data->last_fetch_timestamp) / (float)MSEC_PER_SEC;

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

    /* Apply gyro offset correction */
    gyro_vector = FusionOffsetUpdate(&data->gyro_offset_vector,
                                     gyro_vector);

    /* Ignore low IMU values */
    sensor_fusion_trim_low_imu_values(&accel_vector, &gyro_vector);

    if (config->magn_dev)
    {
        /* Apply magnetometer soft and hard ironing */
        magn_vector = FusionCalibrationMagnetic(magn_vector,
                                                data->magn_calib_soft_iron,
                                                data->magn_calib_hard_iron);

        /* Run AHRS algorithm with accelerometer, gyroscope and magnetometer data */
        FusionAhrsUpdate(&data->ahrs,
                         gyro_vector,
                         accel_vector,
                         magn_vector,
                         delta_time_seconds);
    }
    else
    {
        /* Run AHRS algorithm with only accelerometer and gyroscope data */
        FusionAhrsUpdateNoMagnetometer(&data->ahrs,
                                       gyro_vector,
                                       accel_vector,
                                       delta_time_seconds);
    }

    /* Calculate euler angles from quaternions */
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
    else if (chan == SENSOR_FUSION_CHAN_LINEAR_ACCEL_XYZ)
    {
        FusionVector fusion_vector;
        fusion_vector = FusionAhrsGetLinearAcceleration(&data->ahrs);

        sensor_value_from_float(&val[0], fusion_vector.axis.x);
        sensor_value_from_float(&val[1], fusion_vector.axis.y);
        sensor_value_from_float(&val[2], fusion_vector.axis.z);
    }
    else if (chan == SENSOR_FUSION_CHAN_EARTH_ACCEL_XYZ)
    {
        FusionVector fusion_vector;
        fusion_vector = FusionAhrsGetEarthAcceleration(&data->ahrs);

        sensor_value_from_float(&val[0], fusion_vector.axis.x);
        sensor_value_from_float(&val[1], fusion_vector.axis.y);
        sensor_value_from_float(&val[2], fusion_vector.axis.z);
    }
    else
    {
        return -ENOTSUP;
    }

    return 0;
}

static int sensor_fusion_ahrs_attr_set(const struct device *dev, enum sensor_channel chan,
                                       enum sensor_attribute attr, const struct sensor_value *val)
{
    const struct sensor_fusion_ahrs_config *config = dev->config;
    struct sensor_fusion_ahrs_data *data = dev->data;
    uint8_t i;
    uint8_t j;
    int ret;

    switch (chan)
    {
    case SENSOR_CHAN_ACCEL_XYZ:
        /* Forward the attribute to accelerometer device */
        ret = sensor_attr_set(config->accel_dev, chan, attr, val);
        break;
    case SENSOR_CHAN_GYRO_XYZ:
        /* Forward the attribute to gyroscope device */
        ret = sensor_attr_set(config->gyro_dev, chan, attr, val);
        break;
    case SENSOR_CHAN_MAGN_XYZ:
        switch (attr)
        {
        case SENSOR_FUSION_ATTR_MAGN_HARD_IRON_VECTOR:
            for (i = 0; i < 3; i++)
            {
                /* Fill the 3D vector in X-Y-Z order */
                data->magn_calib_hard_iron.array[i] = sensor_value_to_float(&val[i]);
            }
            ret = 0;
            break;
        case SENSOR_FUSION_ATTR_MAGN_SOFT_IRON_MATRIX:
            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    /* Fill the matrix in row-major order */
                    data->magn_calib_soft_iron.array[i][j] = sensor_value_to_float(&val[3 * i + j]);
                }
            }
            ret = 0;
            break;
        default:
            /* Forward the attribute to magnetometer device */
            ret = sensor_attr_set(config->magn_dev, chan, attr, val);
            break
        }
        break;
    default:
        LOG_ERR("Sensor fusion AHRS attribute not supported.");
        ret = -ENOTSUP;
        break;
    }

    return ret;
}

static int sensor_fusion_ahrs_attr_get(const struct device *dev, enum sensor_channel chan,
                                       enum sensor_attribute attr, struct sensor_value *val)
{
    struct sensor_fusion_ahrs_data *data = dev->data;
    uint8_t i;
    uint8_t j;
    int ret;

    switch (chan)
    {
    case SENSOR_CHAN_ACCEL_XYZ:
        /* Request the attribute from accelerometer device */
        ret = sensor_attr_get(config->accel_dev, chan, attr, val);
        break;
    case SENSOR_CHAN_GYRO_XYZ:
        /* Request the attribute from gyroscope device */
        ret = sensor_attr_get(config->gyro_dev, chan, attr, val);
        break;
    case SENSOR_CHAN_MAGN_XYZ:
        switch (attr)
        {
        case SENSOR_FUSION_ATTR_MAGN_HARD_IRON_VECTOR:
            for (i = 0; i < 3; i++)
            {
                /* Fill the 3D vector in X-Y-Z order */
                sensor_value_from_float(&val[i], data->magn_calib_hard_iron.array[i]);
            }
            ret = 0;
            break;
        case SENSOR_FUSION_ATTR_MAGN_SOFT_IRON_MATRIX:
            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    /* Fill the matrix in row-major order */
                    sensor_value_from_float(&val[3 * i + j], data->magn_calib_soft_iron.array[i][j]);
                }
            }
            ret = 0;
            break;
        default:
            /* Request the attribute from magnetometer device */
            ret = sensor_attr_get(config->magn_dev, chan, attr, val);
            break
        }
        break;
    default:
        LOG_ERR("Sensor fusion AHRS attribute not supported.");
        ret = -ENOTSUP;
        break;
    }

    return ret;
}

static const struct sensor_driver_api sensor_fusion_ahrs_driver_api = {
    .sample_fetch = sensor_fusion_ahrs_sample_fetch,
    .channel_get = sensor_fusion_ahrs_channel_get,
    .attr_set = sensor_fusion_ahrs_attr_set,
    .attr_get = sensor_fusion_ahrs_attr_get,
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

    /* Initialize gyroscope offset vector */
    FusionOffsetInitialise(&data->gyro_offset_vector, config->target_sample_rate_hz);

    /* Set AHRS algorithm settings */
    FusionAhrsSetSettings(&data->ahrs, &config->ahrs_settings);

    return 0;
}

//---------------------------- INTERRUPT HANDLERS -----------------------------

#define SENSOR_FUSION_AHRS_DEFINE(inst)                                               \
    static struct sensor_fusion_ahrs_data sensor_fusion_ahrs_data_##inst = {          \
        .magn_calib_soft_iron.element.xx = 1,                                         \
        .magn_calib_soft_iron.element.yy = 1,                                         \
        .magn_calib_soft_iron.element.zz = 1,                                         \
    };                                                                                \
                                                                                      \
    static const struct sensor_fusion_ahrs_config sensor_fusion_ahrs_config##inst = { \
        .accel_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, accel_dev)),                 \
        .gyro_dev = DEVICE_DT_GET(DT_INST_PHANDLE(inst, gyro_dev)),                   \
        .magn_dev = DEVICE_DT_GET_OR_NULL(DT_INST_PHANDLE(inst, magn_dev)),           \
        .target_sample_rate_hz = DT_INST_PROP(inst, target_sample_rate_hz),           \
        .ahrs_settings = {                                                            \
            .convention = DT_INST_PROP(inst, earth_axes_convention),                  \
            .gain = DT_INST_PROP(inst, gyro_gain) / 100.0f,                           \
            .gyroscopeRange = DT_INST_PROP(inst, gyro_range),                         \
            .accelerationRejection = DT_INST_PROP(inst, accel_rejection),             \
            .magneticRejection = DT_INST_PROP(inst, magn_rejection),                  \
            .recoveryTriggerPeriod = DT_INST_PROP(inst, recovery_trigger_period),     \
        },                                                                            \
    };                                                                                \
                                                                                      \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, sensor_fusion_ahrs_init, NULL,                 \
                                 &sensor_fusion_ahrs_data_##inst,                     \
                                 &sensor_fusion_ahrs_config##inst, POST_KERNEL,       \
                                 CONFIG_SENSOR_INIT_PRIORITY,                         \
                                 &sensor_fusion_ahrs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_FUSION_AHRS_DEFINE)
