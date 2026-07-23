cmake_minimum_required(VERSION 3.20.0)

# Adding board defined in this project to the board searching list
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(caterpillar)
# Embed the FLPR (RISC-V coprocessor) firmware into this image so app
# OTA updates carry it (version-atomic).  Build it first:
#   west build -b caterpillar/nrf54l15/cpuflpr --build-dir build-flpr flpr --no-sysbuild
set(FLPR_BIN ${CMAKE_CURRENT_SOURCE_DIR}/build-flpr/zephyr/zephyr.bin)
if(NOT EXISTS ${FLPR_BIN})
    message(FATAL_ERROR
        "FLPR image not found: ${FLPR_BIN}\n"
        "Build it first:\n"
        "  west build -b caterpillar/nrf54l15/cpuflpr --build-dir build-flpr flpr --no-sysbuild")
endif()
set(FLPR_GEN_DIR ${ZEPHYR_BINARY_DIR}/misc/generated/flpr)
generate_inc_file_for_target(app ${FLPR_BIN} ${FLPR_GEN_DIR}/flpr_image.inc)
target_include_directories(app PRIVATE ${FLPR_GEN_DIR})

target_sources(app PRIVATE
    src/main.c
    src/flpr_launch.c
    src/drivers/driver_stbb1_apur.c
    src/drivers/driver_drv8212.c
    src/drivers/max5419.c
    src/drivers/driver_pwm.c
    src/drivers/driver_led.c
    src/drivers/driver_vdc_sense.c
    src/interface/ble_interface.c
)

# For common/imu_shared.h (shared with the FLPR image)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
