# Copyright (c) 2024 Tomislav Milkovic
# SPDX-License-Identifier: MIT

# Create a new library
zephyr_library()

# Find all C source files inside XIO fusion submodule
file(GLOB
    xio_fusion_sources
    "xio_fusion/Fusion/*.c"
)

# Add all C sources to the library
zephyr_library_sources(
    sensor_fusion_ahrs.c
    ${xio_fusion_sources}
)

# Add XIO fusion directory to the private include directory list for this library
zephyr_library_include_directories(
    xio_fusion/Fusion/
)
