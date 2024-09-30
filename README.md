# Zephyr AHRS sensor fusion library
This repo implements a Zephyr RTOS module which provides a Zephyr sensor fusion device driver which autonomously fetches
accelerometer, gyroscope and (optionally) magnetometer data and does sensor fusion using
a revised Madgwick AHRS algorithm implemented by XIO Technologies. More info about the algorith
can be found here: https://github.com/xioTechnologies/Fusion/

## Usage

1. Add module to your project manifest file

    Include this module in your project's `west.yml` manifest file, e.g.:

    ```
    manifest:
      self:
        path: <name_of_your_project>

      projects:
        - name: sensor-fusion-ahrs
          path: modules/lib/sensor-fusion-ahrs
          url: https://github.com/tmilkovic51/zephyr_sensor_fusion
          revision: v1.2.6
          submodules: true
    ```

    Take note of the `submodules: true` line. If this line is not present, the module
    won't be able to clone the XIO Technologies' AHRS sensor fusion library when running
    `west update` command.

2. Add device tree node to your board device tree

    In your board device tree, under a root node, create a device tree node with
    `zephyr,sensor-fusion-ahrs` compatible string, e.g.:

    ```
    / {
        sensor_fusion: fusion {
            status = "okay";
            compatible = "zephyr,sensor-fusion-ahrs";
            accel-dev = <&accel0>; /* Accelerometer device phandle */
            gyro-dev = <&gyro0>; /* Gyroscope device phandle */
            magn-dev = <&magn0>; /* Magnetometer device phandle */
        }
    }
    ```

    You need to have accelerometer, gyroscope and, optionally, magnetometer devices
    present and enabled in your board device tree.

    To configure other AHRS algorithm settings, please take a look at the
    [bindings file](dts/bindings/sensor/zephyr,sensor-fusion-ahrs.yaml).

3. Use the driver from your application code

    Include `<sensor_fusion/sensor_fusion.h>` header file in your application and
    call Zephyr sensor API `fetch` and `get` functions with custom-made sensor fusion channels:
    - `SENSOR_FUSION_CHAN_YAW` - Euler angle - yaw, in degrees
    - `SENSOR_FUSION_CHAN_PITCH` Euler angle - pitch, in degrees
    - `SENSOR_FUSION_CHAN_ROLL` Euler angle - roll, in degrees
    - `SENSOR_FUSION_CHAN_YAW_PITCH_ROLL` - all three Euler angles (yaw, pitch, roll), in degrees
    - `SENSOR_FUSION_CHAN_LINEAR_ACCEL_XYZ` - Linear acceleration measurement equal to the accelerometer
        measurement with 1 g of gravity removed, in g
    - `SENSOR_FUSION_CHAN_EARTH_ACCEL_XYZ` - Earth acceleration measurement equal to accelerometer measurement
        in the Earth coordinate frame with 1 g of gravity removed, in g
    - `SENSOR_FUSION_CHAN_MAGNETIC_HEADING` - magnetic heading in convention configured in device tree, in degrees

    There are also custom-made attributes that can be used with Zephyr `attr_set` and `attr_get` API:
    - `SENSOR_FUSION_ATTR_MAGN_HARD_IRON_VECTOR` - magnetometer hard ironing 3D vector, in X-Y-Z order, applied to each magnetometer measurement
    - `SENSOR_FUSION_ATTR_MAGN_SOFT_IRON_MATRIX` - magnetometer soft ironing 3x3 matrix, in row-major order, applied to each magnetometer measurement
