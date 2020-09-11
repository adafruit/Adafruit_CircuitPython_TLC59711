"""TLC5971 / TLC59711."""

__doc__ = """
tlc59711_fastset.py - TLC59711 fast set example.

showcases the usage of set_pixel_16bit_value for fastests setting of values.
for speed comparision of all the available set calls
look at the tlc59711_multi_dev.py file.

Simple demo of the TLC59711 16-bit 12 channel LED PWM driver.
Shows setting channel values in a few ways.
Author: Tony DiCola, Stefan Krüger
"""

import board
import busio

import adafruit_tlc59711


# Define SPI bus connected to chip.  You only need the clock and MOSI (output)
# line to use this chip.
spi = busio.SPI(board.SCK, MOSI=board.MOSI)

# Define the TLC59711 instance.
leds = adafruit_tlc59711.TLC59711(spi)
# Optionally you can disable the auto_show behavior that updates the chip
# as soon as any channel value is written.  The default is True/on but you can
# disable and explicitly call show to control when updates happen for better
# animation or atomic RGB LED updates.
# leds = adafruit_tlc59711.TLC59711(spi, auto_show=False)

# There are a couple ways to control the channels of the chip.
# The first is using an interface like a strip of NeoPixels.  Index into the
# class for the channel and set or get its R, G, B tuple value.  Note the
# component values are 16-bit numbers that range from 0-65535 (off to full
# brightness).  Remember there are only 4 channels available too (0 to 3).
# For example set channel 0 (R0, G0, B0) to half brightness:
leds[0] = (32767, 32767, 32767)
# Dont forget to call show if you disabled auto_show above.
# leds.show()

# Or to set channel 1 to full red only (green and blue off).
leds[1] = (65535, 0, 0)

# You can also explicitly control each R0, G0, B0, R1, B1, etc. channel
# by getting and setting its 16-bit value directly with properties.
# For example set channel 2 to full green (i.e. G2 to maximum):
leds.g2 = 65535
# Again don't forget to call show if you disabled auto_show above.
# leds.show()
