.. _arduino_mkrzero:

Arduino MKRZero
###############

Overview
********

The Arduino MKRZero is built around Atmel’s SAMD21 MCU, which features
a 32-bit ARM Cortex M0+ core. It comes in a smaller format than the
Arduino Zero based on that same MCU.

.. image:: img/arduino_mkrzero.jpg
     :width: 500px
     :align: center
     :alt: Arduino MKRZero

Hardware
********

- ATSAMD21G18A ARM Cortex-M0+ processor at 48 MHz
- 32.768 kHz crystal oscillator
- 256 KiB flash memory and 32 KiB of RAM
- 1 user LED
- One reset button
- Native USB port

Supported Features
==================

The arduino_mkrzero board configuration supports the following hardware
features:

+-----------+------------+--------------------------------------+
| Interface | Controller | Driver/Component                     |
+===========+============+======================================+
| NVIC      | on-chip    | nested vector interrupt controller   |
+-----------+------------+--------------------------------------+
| Flash     | on-chip    | Can be used with NFFS to store files |
+-----------+------------+--------------------------------------+
| SYSTICK   | on-chip    | systick                              |
+-----------+------------+--------------------------------------+
| WDT       | on-chip    | Watchdog                             |
+-----------+------------+--------------------------------------+
| GPIO      | on-chip    | I/O ports                            |
+-----------+------------+--------------------------------------+
| USART     | on-chip    | Serial ports                         |
+-----------+------------+--------------------------------------+
| SPI       | on-chip    | Serial Peripheral Interface ports    |
+-----------+------------+--------------------------------------+
| I2C       | on-chip    | I2C ports (experimental)             |
+-----------+------------+--------------------------------------+
| USB       | on-chip    | USB device                           |
+-----------+------------+--------------------------------------+

Other hardware features are not currently supported by Zephyr.

The default configuration can be found in the Kconfig
:file:`boards/arm/arduino_mkrzero/arduino_mkrzero_defconfig`.

Connections and IOs
===================

The `Arduino store`_ has detailed information about board
connections. Download the `Arduino MKRZero Schematic`_ for more detail.

System Clock
============

The SAMD21 MCU is configured to use the 32.768 kHz external oscillator
with the on-chip PLL generating the 48 MHz system clock.  The internal
APB and GCLK unit are set up in the same way as the upstream Arduino
libraries.

Serial Port
===========

The SAMD21 MCU has 6 SERCOM peripherals, each configurable to operate as either:

- USART
- SPI
- I2C  

SERCOM5 is configured as Arduino MKRZero's unique UART interface.

SPI Port
========

SERCOM1 is configured as Arduino MKRZero's unique SPI interface.

I2C Port
========

SERCOM0 is configured as Arduino MKRZero's unique I2C interface.

USB Device Port
===============

The SAMD21 MCU has a USB device port that can be used to communicate
with a host PC.  See the :ref:`usb-samples` sample applications for
more, such as the :ref:`usb_cdc-acm` sample which sets up a virtual
serial port that echos characters back to the host PC.

Programming and Debugging
*************************

Debugging and programming the Arduino MKRZero requires using the
Atmel-ICE development tool with OpenOCD.

Flashing
========

#. Build the Zephyr kernel and the :ref:`hello_world` sample application:

   .. zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: arduino_mkrzero
      :goals: build
      :compact:

#. Connect the Arduino MKRZero to your host computer using the USB debug
   port.

#. Run your favorite terminal program to listen for output. Under Linux the
   terminal should be :code:`/dev/ttyACM0`. For example:

   .. code-block:: console

      $ minicom -D /dev/ttyACM0 -o

   The -o option tells minicom not to send the modem initialization
   string. Connection should be configured as follows:

   - Speed: 115200
   - Data: 8 bits
   - Parity: None
   - Stop bits: 1

#. To flash an image:

   .. zephyr-app-commands::
      :zephyr-app: samples/hello_world
      :board: arduino_mkrzero
      :goals: flash
      :compact:

   You should see "Hello World! arm" in your terminal.

References
**********

.. target-notes::

.. _Arduino Store:
    https://store.arduino.cc/arduino-mkrzero

.. _Arduino MKRZero Schematic:
    https://www.arduino.cc/en/uploads/Main/ArduinoMKRZero-schematic.pdf
