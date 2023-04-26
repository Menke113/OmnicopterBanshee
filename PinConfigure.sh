#!/bin/sh
# This script is called on startup to set the pins
# PWM for motors
config-pin P9.14 pwm
config-pin P9.29 pwm
config-pin P9.31 pwm
config-pin P8.46 pwm
config-pin P8.13 pwm
config-pin P8.19 pwm
config-pin P8.34 pwm
config-pin P8.36 pwm
# UART for receiver
config-pin P9.24 uart
config-pin P9.26 uart
# GPIO for Distance sensors
config-pin P8.15 gpio
config-pin P8.16 gpio
config-pin P8.17 gpio
config-pin P8.18 gpio
