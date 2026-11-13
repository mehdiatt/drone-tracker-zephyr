# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(blink_basic)

target_sources(app PRIVATE
    src/main.c
    src/mpu6050.c
    src/rgb_led.c
    src/ldr.c
    src/si7021.c
    src/telemetry.c
    src/imu.c
)

target_include_directories(app PRIVATE)