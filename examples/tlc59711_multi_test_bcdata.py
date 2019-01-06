"""TLC5971 / TLC59711 Multi."""

__doc__ = """
TLC59711Multi development helper.

this sketch contains a bunch of timing tests and other development helpers..
Enjoy the colors :-)
"""

import time

import board
import busio
import supervisor

from adafruit_tlc59711.adafruit_tlc59711_multi import TLC59711Multi


##########################################
pixel_count = 16*8

spi = busio.SPI(board.SCK, MOSI=board.MOSI)
pixels = TLC59711Multi(spi, pixel_count=pixel_count)

##########################################


def test_main():
    """Test Main."""
    print(42 * '*', end="")
    print(__doc__, end="")
    print(42 * '*')
    # print()
    # time.sleep(0.5)
    # print(42 * '*')

    print("set pixel all to 100, 100, 100")
    pixels.set_pixel_all((1000, 1000, 1000))
    print(
        "bcr: {:>3}\n"
        "bcg: {:>3}\n"
        "bcb: {:>3}\n"
        "".format(
            pixels.bcr,
            pixels.bcg,
            pixels.bcb,
        )
    )
    pixels.show()
    time.sleep(0.1)

    if supervisor.runtime.serial_connected:
        print(
            "you can change the global brightness control:\n"
            "use format: 'Ioclmax; IoutR; IoutG; IoutB'\n"
            "example: '18; 7; 15; 17'"
        )
    while True:
        if supervisor.runtime.serial_bytes_available:
            new_value = input()
            if "v" in new_value:
                try:
                    value = int(new_value[1:])
                except ValueError as e:
                    print("Exception: ", e)
                pixels.set_pixel_all_16bit_value(value, value, value)
            else:
                try:
                    Ioclmax, IoutR, IoutG, IoutB = new_value.split(';')
                    Ioclmax = float(Ioclmax)
                    IoutR = float(IoutR)
                    IoutG = float(IoutG)
                    IoutB = float(IoutB)
                except ValueError as e:
                    print("Exception: ", e)
                BCValues = TLC59711Multi.calculate_BCData(
                    Ioclmax=Ioclmax,
                    IoutR=IoutR,
                    IoutG=IoutG,
                    IoutB=IoutB,
                )
                pixels.bcr = BCValues[0]
                pixels.bcg = BCValues[1]
                pixels.bcb = BCValues[2]
                print(
                    "bcr: {:>3}\n"
                    "bcg: {:>3}\n"
                    "bcb: {:>3}\n"
                    "".format(
                        pixels.bcr,
                        pixels.bcg,
                        pixels.bcb,
                    )
                )
                pixels.update_BCData()
            pixels.show()
            # prepare new input
            print("\nenter new values:")


##########################################
# main loop

if __name__ == '__main__':

    test_main()
