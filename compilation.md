The compilation instructions from [the documentation](https://betaflight.com/docs/development/building/Building-in-Mac-OSX) 
are currently (28 sep 2024) incorrect. There are significant changes to the commands due to the move to release 4.5.0, 
and maybe others.

The modified sequence of operations that worked for me is the following:
1. Install the arm-none-eabi toolchain and newlib_nano C library with the command ```make arm_sdk_install```. This downloads pre-compiled binaries for both the compiler/linker and for the C library. I have no idea where these versions come from, nor how to recompile them.
2. Execute ```make configs```. This downloads available configurations under ```src/config/configs```. SPEEDYBEEF405V4 is one of these configurations.
3. Execute ```make SPEEDYBEEF405V4```.
