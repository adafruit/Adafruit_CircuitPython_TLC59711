"""TLC5971 / TLC59711."""

__doc__ = """
tlc59711_fastset.py - TLC59711 fast set example.

showcases the usage of set_pixel_16bit_value for fastests setting of values.
for speed comparision of all the available set calls
look at the tlc59711_multi_dev.py file.

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

def channelcheck_update_pixel(offset):
    """Channel check pixel."""
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
        pixels.set_pixel_all_16bit_value(0, 1, 0)
        pixels.show()
    return offset


def test_main():
    """Test Main."""
    print(42 * '*', end="")
    print(__doc__, end="")
    print(42 * '*')

    bcvalues = TLC59711Multi.calculate_BCData(
        Ioclmax=18,
        IoutR=18,
        IoutG=11,
        IoutB=13,
    )
    print("bcvalues = {}".format(bcvalues))
    pixels.bcr = bcvalues[0]
    pixels.bcg = bcvalues[1]
    pixels.bcb = bcvalues[2]
    pixels.update_BCData()
    pixels.show()

    offset = 0

    print("loop:")
    while True:
        offset = channelcheck_update_pixel(offset)
        time.sleep(0.2)


##########################################
# main loop

if __name__ == '__main__':

    test_main()
