# SensESP Windlass Chain Counter

This Windlass Counter is a fork of .... https://github.com/raffmont/SensESP-Windlass-Controller 
but with the "control" elements deleted , retaining just the "counter" and associated logic.
It requires electrical access to READ the Up & Down control signals from the windlass, AND the chain counter sensor (reed or hall effect) 
It utilises Expressif ESP32 WROM microcontroller and the SensESP framework (https://github.com/SignalK/SensESP).

The device connects to an onboard Signal K server (https://signalk.org), providing the following features:

* Count deployed anchor rode in meters.
* Measure the up/down chain speed.
* Sense the windlass status as going up, down, free down, free up, and off.
* Enables remote/automatic windlass control reading the status.
* Fully customizable Signal K keys and calibration parameters. 
* Local chain counter reset button
* Remote reset of chain counter


The device and all the software are provided with no warrinty or responsibility for correct or incorrect usage, eventually generating damages to people or freights.

This work has been inspired and forked from https://github.com/raffmont/SensESP-Windlass-Controller; 
which was partially  inspired by this repository https://github.com/AK-Homberger/ESP8266_AnchorChainControl_WLAN,  the SensESP example "Chain Counter".
