cmake_minimum_required(VERSION 3.20.0)

# Adding board defined in this project to the board searching list
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(caterpillar)
# Embed the FLPR (RISC-V coprocessor) firmware into this image so app
# OTA updates carry it (version-atomic).  The FLPR is configured here on
# first use and incrementally rebuilt on every app build, so a plain
# `west build` of the app picks up FLPR source changes automatically.
set(FLPR_BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build-flpr)
set(FLPR_BIN ${FLPR_BUILD_DIR}/zephyr/zephyr.bin)

if(NOT EXISTS ${FLPR_BUILD_DIR}/build.ninja)
    message(STATUS "FLPR: first-time configure+build of ${FLPR_BUILD_DIR}")
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -m west build
                -b caterpillar/nrf54l15/cpuflpr
                --build-dir ${FLPR_BUILD_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/flpr --no-sysbuild
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        RESULT_VARIABLE flpr_rc
    )
    if(NOT flpr_rc EQUAL 0 OR NOT EXISTS ${FLPR_BIN})
        message(FATAL_ERROR "FLPR build failed (see output above)")
    endif()
endif()

# The staged copy's mtime only changes when the FLPR binary content
# changed, so an unchanged FLPR does not re-embed or re-link the app.
set(FLPR_STAGED ${CMAKE_BINARY_DIR}/flpr_staged.bin)
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${FLPR_BIN} ${FLPR_STAGED})
add_custom_target(flpr_build
    COMMAND ${CMAKE_COMMAND} --build ${FLPR_BUILD_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different ${FLPR_BIN} ${FLPR_STAGED}
    BYPRODUCTS ${FLPR_STAGED}
    USES_TERMINAL
    COMMENT "FLPR: incremental rebuild"
)

set(FLPR_GEN_DIR ${ZEPHYR_BINARY_DIR}/misc/generated/flpr)
generate_inc_file_for_target(app ${FLPR_STAGED} ${FLPR_GEN_DIR}/flpr_image.inc)
add_dependencies(app flpr_build)
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
