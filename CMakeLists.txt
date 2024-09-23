# Copyright (c) 2024 Tomislav Milkovic
# SPDX-License-Identifier: MIT

# Public header for custom driver API
zephyr_include_directories(
  include
)

# Sensor fusion driver sources
add_subdirectory(drivers)
