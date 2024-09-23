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

3. Use the driver from your application code

    Include `<sensor_fusion/sensor_fusion.h>` header file in your application and
    call Zephyr sensor API functions with custom-made sensor fusion channels:
    - `SENSOR_FUSION_CHAN_YAW` for yaw value, in degrees
    - `SENSOR_FUSION_CHAN_PITCH` for pitch value, in degrees
    - `SENSOR_FUSION_CHAN_ROLL` for roll value, in degrees
    - `SENSOR_FUSION_CHAN_YAW_PITCH_ROLL` for all values, in degrees
