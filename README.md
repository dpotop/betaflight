This is a fork of the [Betaflight](https://github.com/betaflight/betaflight) flight control software. For up-to-date general information about Betaflight, visit [the upstream github](https://github.com/betaflight/betaflight).

I'm a newcomer to the drone community, where I came together with my son.

My objective is to understand the way the flight controller works in order to be able to extend it with new control algorithms. The medium-term objective I have is to be able to allow my quad drone to [maintain a stick in equilibrium on top of it](https://www.youtube.com/watch?v=XmYRQi48s-8). This is a form of [inverted pendulum](https://en.wikipedia.org/wiki/Inverted_pendulum), and a nice ersatz for the basic control problems behind space launchers.

For now, I have managed to:
* Order the hardware and build the drone. I will add the list shortly.
* Sort out the initial communication and configuration problems (charging a battery, assigning aux channels...). All of this is new for me.
* Recompile betaflight and flash it (and the drone still works!).
  * Recompilation is done under full control, with my own scripts extracted from the automated compilation process.
* Identify the basic elements of the computer hardware and software of my SpeedyBee f405 v4 flight controller stack:
  * determine exactly what files get compiled for my configuration, and under which options...
  * interrupt vectors, and I determined which vectors are non-empty. For instance SpeedyBee does not use SVC. The result of this analysis is under [tasks-and-interrupts.txt](tasks-and-interrupts.txt).
  * tasks (and periods, priorities, functions to call...). The data is also under [tasks-and-interrupts.txt](tasks-and-interrupts.txt).
  * the PID controller, which is one of the tasks.
  * I don't yet have the link from interrupts to tasks (and I did not yet understand the scheduling algorithm).
* Start to understand the basics of rate adjusting (for throttle, pitch, roll, yaw), which will allow me at some point to achieve stable flight indoors. The drone I bought is a tad too powerful. But I needed it this way, in order to support, when the project is completed, the inverted pendulum stick. This means that the learning curve for piloting is steep (I already learned to have a lot of respect for the propellers, after one of them tried to grind one of my fingers).
  * I have started to play with the configuration of the rates, but the result is not yet intuitive to me, even though I have drastically reduced the max power on throttle and the 3 degrees of freedom. 
* Add ibus telemetry, following the instruction of [this page](https://betaflight.com/docs/wiki/guides/current/ibus-telemetry). This required a diode, a resistor, and some soldering. I am really impressed at how easy it was to re-configure the drone afterwards using the [configurator app](https://app.betaflight.com).

The next steps, not necessarily in order:
* Flash to the FS-I6 transmitter and the fs-ai6b receiver to add 10 channels (under ibus protocol).
* Further tune the rates to allow indoors stable non-jittery flight. I guess my next try will be to have very small rates.
* Add a small echolocator or LIDAR to measure distance to ground and then maintain altitude in a given mode without having to push throttle, or simply add a ceiling and make it so throttle cannot force breaking the ceiling.
* Imagine a new control law.
* Better understand the SW and HW architecture, to see if I can modify the code in meaningful ways.

As a long-term objective, being a full-time embedded systems researcher, I would like to see if modern software design methods for embedded control systems (like those used for commercial aircraft control software) can be applied here. In any case, I'm starting slow, one step at a time.
