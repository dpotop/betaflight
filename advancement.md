# 29-09-2024 - Separate compilation scripts

I have added in [compilation.md](compilation.md) instructions that work on the current version of betaflight (the version from the documentation is no longer complete and correct).

To be able to finely control compilation, what is included and what not into my executables, I have extracted separate compilation scripts (```compile.sh```,```link.sh```,```clean.sh```).

There may be some limits to my scripts, as the newlib library may contain the interrupt vectors, and I still don't know how to compile newlib. The configuration of newlib_nano can be found under ``betaflight/tools/arm-gnu-toolchain-13.2.Rel1-darwin-x86_64-arm-none-eabi/13.2.Rel1-darwin-x86_64-arm-none-eabi-manifest.txt`` after executing ```make arm_sdk_install```.

https://wiki.segger.com/Arm_Cortex-M_interrupts

I have now to start 