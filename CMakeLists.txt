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

# First-time bootstrap runs a nested west build; the outer (ARM) app
# configure leaks toolchain environment into it, which poisons the
# RISC-V compiler discovery (verified from a clean clone: FLPR files
# compiled with arm-zephyr-eabi-gcc).  Scrub those variables.
if(NOT EXISTS ${FLPR_BIN})
    message(STATUS "FLPR: first-time configure+build of ${FLPR_BUILD_DIR}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env
                --unset=CC --unset=CXX --unset=ASM
                --unset=CROSS_COMPILE --unset=TOOLCHAIN_HOME
                --unset=ZEPHYR_TOOLCHAIN_VARIANT
                --unset=CMAKE_TOOLCHAIN_FILE
                ${PYTHON_EXECUTABLE} -m west build
                -b caterpillar/nrf54l15/cpuflpr
                --build-dir ${FLPR_BUILD_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/flpr --no-sysbuild --pristine
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

# Guard against settings drift: settings_gen.h (and protocol_limits.py,
# docs/settings.md) are generated MANUALLY from settings.yml and
# committed — the build itself has no pyyaml dependency.  This check
# makes forgetting to regenerate a hard error instead of silent drift.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             ${CMAKE_CURRENT_SOURCE_DIR}/settings.yml)
file(SHA1 ${CMAKE_CURRENT_SOURCE_DIR}/settings.yml SETTINGS_YML_SHA1)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/src/settings/settings_gen.h
     SETTINGS_GEN_CONTENT)
string(FIND "${SETTINGS_GEN_CONTENT}" "${SETTINGS_YML_SHA1}" SETTINGS_HASH_POS)
if(SETTINGS_HASH_POS EQUAL -1)
    message(FATAL_ERROR
        "settings.yml changed but generated files are stale.\n"
        "Run:  python scripts/generate_settings.py")
endif()

target_sources(app PRIVATE
    src/main.c
    src/app/app.c
    src/app/session.c
    src/app/device_cmd.c
    src/imu/imu_pump.c
    src/imu/imu_log.c
    src/imu/flpr_launch.c
    src/settings/settings_store.c
    src/ble/ble_interface.c
    src/ble/ble_transport.c
    src/drivers/driver_stbb1_apur.c
    src/drivers/driver_drv8212.c
    src/drivers/max5419.c
    src/drivers/driver_pwm.c
    src/drivers/driver_led.c
    src/drivers/driver_vdc_sense.c
)

# For common/imu_shared.h (shared with the FLPR image)
# Repo root (for common/imu_shared.h) and src/ (module-qualified
# includes: "app/session.h", "imu/imu_log.h", "ble/ble_transport.h"...)
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src)
