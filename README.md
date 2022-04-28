# Callisto

Callisto is a digital clock that uses a Russian IV-18 vacuum fluorescent tube for the display. The clock uses an ESP32 to automatically sync time.

The name *Callisto* was chosen so that the name could be displayed on an 8-digit seven-segment display.

![DSC01592 (2)](https://user-images.githubusercontent.com/20251225/147790506-1a9c013b-f025-4b8d-8492-5346c15807aa.JPG)

## Features:
- Sync time, retrieved using WiFi
- Automatically switch to daylight saving time
- Clock display settings configured using web browser
- Automatic shutoff during night hours in order to decrease energy usage and extend tube life
- Auto-dimming based on ambient light conditions

## Special thanks

The software and hardware design of the clock was ~~stolen from~~ heavily inspired by the following projects:

- [Ice Tube Clock by Lady Ada](https://learn.adafruit.com/ice-tube-clock-kit/)
- [Flora-ESP8266 by mcer12](https://github.com/mcer12/Flora-ESP8266)

