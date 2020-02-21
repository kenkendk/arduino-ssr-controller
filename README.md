# Arduino SSR Controller
An Arduino project for controlling a solid-state-relay for a water heater

# Overview
A 10k ohm potentiometer is connected to an analog input. Turning it translates into values in the range `[0-1023]`.

An attached [SSR](https://en.wikipedia.org/wiki/Solid-state_relay) is connected to a water heater. By turning the SSR on and off, it is possible to limit the time the heater is supplied with power, and thus reduce the amount of heat it emits.

The potentiometer then turns the value into the period that it is turned on. Since the 230V AC frequency (in Europe) is 50 Hz, we can at most turn the SSR on and off with 100 Hz. For that reason, the `[0-1023]` is capped at the ends (to make sure we can always turn fully off and fully on), giving a range of `[20-1003]`, which is turned into `[0-100]`.

For a water heater, the heat dissipates slowly, so we can simply pick a fragment of cycles to keep the SSR on, and the off for the other cycles. We can use a long cycle, as we are interested in the average heat emitted. To support other use cases, there is also a switch that can toggle mode where the cycle is shorter (10ms) and the flips happen as frequently as possible. This can be used for pumps and other equipment that would not behave properly with frequent stop/start.

The LCD display shows the current power output in percent, as well as the mode and the obtained time between each SSR output update. A small 10k ohm trimmer potentiometer is required to adjust the LCD brightness.

![Schematics](https://raw.githubusercontent.com/kenkendk/arduino-ssr-controller/master/arduino-ssr-controller_bb.png)
