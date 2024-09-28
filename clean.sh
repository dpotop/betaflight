#!/bin/bash

set -x

find src -name *.o -exec rm -f {} \;
find lib -name *.o -exec rm -f {} \;

rm -f betaflight_4.6.0_STM32F405_SPEEDYBEEF405V4.hex betaflight_STM32F405_SPEEDYBEEF405V4.elf betaflight_STM32F405_SPEEDYBEEF405V4.map
