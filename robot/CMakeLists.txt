cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(capbot_firmware)

include_directories(inc)
target_sources(app PRIVATE src/main.c src/capbot_board.c src/capbot_motor.c src/ble_rcs.c)
