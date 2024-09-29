# 29-09-2024 - Separate compilation scripts

I have added in [compilation.md](compilation.md) instructions that work on the current version of betaflight (the version from the documentation is no longer complete and correct).

To be able to finely control compilation, what is included and what not into my executables, I have extracted separate compilation scripts ([compile.sh](compile.sh),[link.sh](link.sh),[clean.sh](clean.sh)). To extract them, I simply dry-run the compilation process with ```make -n```.

There may be some limits to my scripts, as I still don't compile the ```newlib``` C library by myself, and I don't exactly know which files are included during compilation (the library has ```crt``` files) even though its compilation seems to be done so that it does not include the interrupt vectors. After executing ```make arm_sdk_install``` (according to the compilation instructions above), the rules for compiling gcc, binutils, and the newlib_nano library can be found under [betaflight/tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/13.2.Rel1-darwin-x86_64-arm-none-eabi-manifest.txt](betaflight/tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/13.2.Rel1-darwin-x86_64-arm-none-eabi-manifest.txt). 

The interrupt architecture of ARM Cortex-M can be found 
[here](https://wiki.segger.com/Arm_Cortex-M_interrupts). Note that this is a general architecture. Each chip adds specific vectors. In the case of my FC this chip is the [STM32F4](https://stm32f4-discovery.net/2014/08/stm32f4-external-interrupts-tutorial/), and the [Speedy Bee f405 v4](https://www.speedybee.com/speedybee-f405-v4-bls-55a-30x30-fc-esc-stack/#Parameters) determines what the interrupt lines are assigned to, as detailed in the assembly file containing the interrupt vector table for my drone FC platform ([src/main/startup/stm32/startup_stm32f40xx.s](src/main/startup/stm32/startup_stm32f40xx.s)). To find the manipulations of the vector table, do a ```grep -R isr_table *``` at the top-level of the source hierarchy. I have removed the files pertaining to other HW platforms, so the results should be all pertinent.

I have now to start and see if I find all these vectors and their configuration, both at linker script level and in C, both statically and dynamically (at startup).
