# AVRDUDE

[![Build Status](https://github.com/avrdudes/avrdude/actions/workflows/build.yml/badge.svg)](https://github.com/avrdudes/avrdude/actions/workflows/build.yml)
[![Gh-pages docs](https://img.shields.io/badge/Docs-online-blue)](https://avrdudes.github.io/avrdude/)
[![GitHub Release](https://img.shields.io/github/v/release/avrdudes/avrdude?color=yellow)](https://github.com/avrdudes/avrdude/releases)

AVRDUDE - AVR Downloader Uploader - is a program for downloading and uploading
the on-chip memories of Microchip’s [AVR microcontrollers](https://en.wikipedia.org/wiki/AVR_microcontrollers).
It can program the Flash and EEPROM, and where supported by the programming
protocol, it can program fuse and lock bits.
AVRDUDE also supplies a direct instruction mode allowing one to issue any
programming instruction to the AVR chip regardless of whether AVRDUDE
implements that specific feature of a particular chip.

AVRDUDE was originally written in 2003 by Brian S. Dean. Since 2006, AVRDUDE has been maintained by Jörg Wunsch,
with the help of [various contributors](./AUTHORS).

The latest version of AVRDUDE is always available here:\
<https://github.com/avrdudes/avrdude>

## Documentation

Documentation for current and previous releases is [on Github Pages](https://avrdudes.github.io/avrdude/). Git main is documented only with the most recent [avrdude.pdf](https://github.com/avrdudes/avrdude/blob/main/avrdude.pdf).

## Getting AVRDUDE for Windows

To get AVRDUDE for Windows, install the latest version from the [Releases](https://github.com/avrdudes/avrdude/releases) page.

Alternatively, you may [build AVRDUDE](https://github.com/avrdudes/avrdude/wiki) yourself from source.

## Getting AVRDUDE for Linux

To install AVRDUDE for Linux, install the package `avrdude` using the software package manager. For example, under Debian/Ubuntu, you can use the following commands:

```console
sudo apt-get install avrdude
```

Alternatively, you may [build AVRDUDE](https://github.com/avrdudes/avrdude/wiki) yourself from source.

## Getting AVRDUDE for macOS

On macOS, AVRDUDE can be installed through MacPorts or Homebrew.

Alternatively, you may [build AVRDUDE](https://github.com/avrdudes/avrdude/wiki) yourself from source.

## Using AVRDUDE

AVRDUDE is a command-line application. Run the command `avrdude` without any arguments for a list of options.

A typical command to program your HEX file into your AVR microcontroller looks like this:

```console
avrdude -c <programmer> -p <part> -U flash:w:<file>:i
```

For instance, to program an **Arduino Uno** connected to the serial port **COM1** with a HEX file called `blink.hex`,
you would run the following command:

```console
avrdude -c arduino -P COM1 -b 115200 -p atmega328p -D -U flash:w:objs/blink.hex:i
```

There are many different programmers and options that may be required for the programming to succeed.

For more information, refer to the [AVRDUDE documentation](https://avrdudes.github.io/avrdude/).

## Using the AVRDUDE GUI demonstrator

Starting with version 8, a GUI implementation has been added, to demonstrate the functionality of `libavrdude` is suitable to implement a native GUI (as opposed to CLI wrapper).

The GUI is based on the Qt toolkit and its Python bindings, called _PySide_.
Either Qt5 with PySide2, or Qt6 with PySide6 are supported.

A script named `avrdude-gui` is installed into the same location as the AVRDUDE CLI program. It can be used to start the GUI. There is a builtin help describing the usage.
