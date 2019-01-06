"""TLC5971 / TLC59711 Multi."""

__doc__ = """
tlc59711_multi.py - TLC59711TLC59711Multi minimal example.

Enjoy the colors :-)
"""

import time

import board
import busio

# from adafruit_tlc59711.adafruit_tlc59711 import TLC59711
from adafruit_tlc59711.adafruit_tlc59711_multi import TLC59711Multi


##########################################
pixel_count = 16

spi = busio.SPI(board.SCK, MOSI=board.MOSI)
pixels = TLC59711Multi(spi, pixel_count=pixel_count)


##########################################
# test function

offset = 0


def channelcheck_update_pixel():
    """Channel check pixel."""
    global offset  #noqa
    # print("offset", offset)

    pixels.set_pixel_16bit_value(offset, 1000, 100, 0)
    # clear last pixel
    last = offset-1
    if last < 0:
        last = pixel_count-1
    pixels.set_pixel_16bit_value(last, 0, 0, 1)
    pixels.show()

    offset += 1
    if offset >= pixel_count:
        time.sleep(0.2)
        offset = 0
        print("clear")
        pixels.set_pixel_all((0, 1, 0))
        pixels.show()


def channelcheck_update():
    """Channel check."""
    global offset  #noqa
    # print("offset", offset)

    pixels.set_channel(offset, 1000)
    # clear last set channel
    last = offset-1
    if last < 0:
        last = pixels.channel_count-1
    pixels.set_channel(last, 0)
    pixels.show()

    offset += 1
    if offset >= pixels.channel_count:
        offset = 0


def test_main():
    """Test Main."""
    print(42 * '*', end="")
    print(__doc__, end="")
    print(42 * '*')

    BCValues = TLC59711Multi.calculate_BCData(
        Ioclmax=18,
        IoutR=18,
        IoutG=11,
        IoutB=13,
    )
    print("BCValues = {}".format(BCValues))
    pixels.bcr = BCValues[0]
    pixels.bcg = BCValues[1]
    pixels.bcb = BCValues[2]
    pixels.update_BCData()
    pixels.show()

    print("loop:")
    while True:
        channelcheck_update_pixel()
        time.sleep(0.2)
        # channelcheck_update()


##########################################
# main loop

if __name__ == '__main__':

    test_main()
