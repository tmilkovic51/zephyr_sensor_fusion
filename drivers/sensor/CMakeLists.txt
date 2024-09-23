# Copyright (c) 2024 Tomislav Milkovic
# SPDX-License-Identifier: MIT

# Add sources to build only if driver is enabled
add_subdirectory_ifdef(
    CONFIG_SENSOR_FUSION_AHRS
    sensor_fusion_ahrs
)
