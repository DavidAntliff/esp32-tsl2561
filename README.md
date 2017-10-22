# esp32-tsl2561

## Introduction

This component provides useful access functions for the TAOS TSL2561 Light-to-Digital Converter device.

It is written and tested for the [ESP-IDF](https://github.com/espressif/esp-idf) environment, version 2.1, using the xtensa-esp32-elf toolchain (gcc version 5.2.0).

## Dependencies

Requires [esp32-smbus](https://github.com/DavidAntliff/esp32-smbus).

## Example

An example of this component in use can be found at [DavidAntliff/esp32-tsl2561-example](https://github.com/DavidAntliff/esp32-tsl2561-example)

## Features

 * Retrieval of device ID and revision number.
 * Configuration of integration time (13, 101 or 402 milliseconds).
 * Configuration of gain (1x or 16x).
 * Calculation of Lux approximation.

## Documentation

Automatically generated API documentation (doxygen) is available [here](https://davidantliff.github.io/esp32-tsl2561/index.html).

## Source Code

The source is available from [GitHub](https://www.github.com/DavidAntliff/esp32-tsl2561).

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

 * [TSL2560, TSL2561 datasheet (via Adafruit)](https://cdn-shop.adafruit.com/datasheets/TSL2561.pdf)
 * [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)
 
## Acknowledgements

 * Acknowledgements to Kevin Townsend for the Adafruit TSL2561 driver: https://github.com/adafruit/Adafruit_TSL2561
 * Acknowledgements to https://github.com/lexruee/tsl2561 for a second working reference.
 * "SMBus" is a trademark of Intel Corporation.

## Roadmap

The following features are anticipated but not yet implemented:

 * Interrupt support with upper and lower thresholds.
 * Automatic gain selection.
 * Manual integration time.

 
