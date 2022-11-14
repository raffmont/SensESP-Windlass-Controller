# SensESP Windlass Controller

The Windlass Controller implements an auxiliary device to be connected to the anchor windlass relay box and the chain counter leveraging Expressif ESP32 WROM microcontroller and the SensESP framework (https://github.com/SignalK/SensESP).

The device connects to an onboard Signal K server (https://signalk.org), providing the following features:

* Count deployed anchor rods in meters.
* Measure the up/down chain speed.
* Sense the windlass status as going up, down, free down, free up, and off.
* Enables remote/automatic windlass control reading the status.
* Fully customizable Signal K keys and calibration parameters. 

A KiCad drawing and schematics are available for download and implementation.

The device and all the software are provided with no warrinty or responsibility for correct or incorrect usage, eventually generating damages to people or freights.

This work has partially been inspired by this repository https://github.com/AK-Homberger/ESP8266_AnchorChainControl_WLAN and the SensESP example "Chain Counter".