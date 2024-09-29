This is a fork of the [Betaflight](https://github.com/betaflight/betaflight) flight control software. For up-to-date general information about Betaflight, visit [the upstream github](https://github.com/betaflight/betaflight).

I'm a newcomer to the drone community, where I came together with my son.

My objective is to understand the way the flight controller works in order to be able to extend it with new control algorithms. The medium-term objective I have is to be able to allow my quad drone to maintain a stick in equilibrium on top of it. This is a form of [inverted pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum), and a nice ersatz for the basic control problems behind space launchers.

For now, I have managed to:
* Order the hardware and build the drone. I will add the list shortly.
* Sort out the initial communication and configuration problems (charging a battery, assigning aux channels...)
* Recompile betaflight and flash it (and the drone still works!).
* Identify the basic elements of the computer hardware and software of my SpeedyBee f405 v4 flight controller stack (e.g. interrupts, determine exactly what files get compiled for my configuration, and under which options...).
* Start to understand the basics of rate adjusting (for throttle, pitch, roll, yaw), which will allow me at some point to achieve stable flight indoors. The drone I bought is a tad too powerful, in order to support, when the project is completed, the inverted pendulum stick. This means that the learning curve for piloting is steep (I already learned to have a lot of respect for the propellers, after one of them tried to grind one of my fingers).

The next steps, not necessarily in order:
* Flash to the FS-I6 transmitter and the fs-ai6b receiver to add 10 channels (under ibus protocol).
* Add ibus telemetry (which requires some soldering of a diode and resistor).
* Further tune the rates to allow indoors stable non-jittery flight. I guess my next try will be to have very small rates.
* Add a small echolocator or LIDAR to measure distance to ground and then maintain altitude in a given mode without having to push throttle, or simply add a ceiling and make it so throttle cannot force breaking the ceiling.
* Identify the PID controller and then imagine a new control law.
* Better understand the SW and HW architecture. Right now, I can use my own compilation scripts, so I can finely tune what goes in and out of the binary. Now I have to see if I can modify the code in meaningful ways.

As a long-term objective, being a full-time embedded systems researcher, I would like to see if modern software design methods for embedded control systems (like those used for commercial aircraft control software) can be applied here. In any case, I'm starting slow, one step at a time.
