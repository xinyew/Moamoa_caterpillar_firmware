cmake_minimum_required(VERSION 3.20.0)

# Adding board defined in this project to the board searching list
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(caterpillar)
target_sources(app PRIVATE
    src/main.c
    src/drivers/driver_stbb1_apur.c
    src/drivers/driver_drv8212.c
    src/drivers/driver_asm330lhh.c
    src/drivers/max5419.c
    src/drivers/driver_pwm.c
    src/drivers/driver_led.c
    src/interface/ble_interface.c
)
