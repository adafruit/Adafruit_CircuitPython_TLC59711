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
pixel_count = 16*8

spi = busio.SPI(board.SCK, MOSI=board.MOSI)
pixels = TLC59711Multi(spi, pixel_count=pixel_count)


##########################################
# test function

offset = 0
value_high = 1000


def channelcheck_update_pixel():
    """ChannelCheck pixel."""
    global offset  #noqa
    # print("offset", offset)

    pixels[offset] = (value_high, 0, 0)
    # clear last pixel
    last = offset-1
    if last < 0:
        last = pixel_count-1
    pixels[last] = (0, 0, 1)
    # pixels[offset] = (0xAAAA, 0xBBBB, 0xCCCC)
    pixels.show()

    offset += 1
    if offset >= pixel_count:
        # time.sleep(0.5)
        offset = 0
        # print("clear")
        # set_all_black()
        # set_all((0, 1, 0))
        # pixels.show()
        # print()
        # time.sleep(2)


def channelcheck_update():
    """ChannelCheck."""
    global offset  #noqa
    # print("offset", offset)

    pixels.set_channel(offset, value_high)
    # clear last set channel
    last = offset-1
    if last < 0:
        last = pixels.channel_count-1
    pixels.set_channel(last, 0)
    pixels.show()

    offset += 1
    if offset >= pixels.channel_count:
        offset = 0


def set_all_black():
    """Set all Pixel to Black."""
    set_all((0, 0, 0))


def set_all(color):
    """Set all Pixel to color."""
    for i in range(pixel_count):
        # pixels[i // 4][i % 4] = color
        pixels[i] = color


##########################################

def time_meassurement_call(test_function, loop_count=1000):
    """Meassure timming."""
    duration = 0
    start_time = time.monotonic()
    for _index in range(loop_count):
        start_time = time.monotonic()
        test_function()
        end_time = time.monotonic()
        duration += end_time - start_time
    # print(
    #     "duration:\n"
    #     "\t{}s for {} loops\n"
    #     "\t{:.2f}ms per call"
    #     "".format(
    #         duration,
    #         loop_count,
    #         (duration/loop_count)*1000
    #     )
    # )
    print(
        "\t{:.2f}ms per call"
        "".format((duration / loop_count) * 1000)
    )


def time_meassurement_pixels_show():
    """Meassure timming."""
    print("pixels show:")
    loop_count = 1000

    def _test():
        pixels.show()
    print("'pixels.show()'")
    time_meassurement_call(_test, loop_count)


def time_meassurement_pixels_set():
    """Meassure timming pixels set."""
    print("pixels set:")
    loop_count = 1000

    def _test():
        pixels[3] = (0, 0, 1000)
    print("'pixels[3] = (0, 0, 1000)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        pixels[3] = (500, 40500, 1000)
    print("'pixels[3] = (500, 40500, 1000)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        pixels[12] = (0.5, 0.5, 0.5)
    print("'pixels[12] = (0.5, 0.5, 0.5)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        for i in range(16):
            pixels[i] = (0.5, 0.5, 0.5)
    print("'pixels[for 0..16] = (0.5, 0.5, 0.5)'")
    time_meassurement_call(_test, 100)

    def _test():
        for i in range(pixel_count):
            pixels[i] = (0.5, 0.5, 0.5)
    print("'pixels[for 0..{}] = (0.5, 0.5, 0.5)'".format(pixel_count))
    time_meassurement_call(_test, 10)


def time_meassurement_channel_set():
    """Meassure timming channel set."""
    print("channel set:")
    loop_count = 1000

    def _test():
        pixels.set_channel(0, 10000)
    print("'set_channel(0, 10000)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        pixels.set_channel(0, 10000)
        pixels.set_channel(1, 10000)
        pixels.set_channel(2, 10000)
    print("'set_channel(0..2, 10000)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        for i in range(pixel_count * 3):
            pixels.set_channel(i, 500)
    print(
        "'set_channel(for 0..{}, 10000)'"
        "".format(pixel_count * 3)
    )
    time_meassurement_call(_test, 10)


def time_meassurement_channel_set_internal():
    """Meassure timming channel set internal."""
    print("channel set internal:")
    loop_count = 1000

    def _test():
        pixels._set_channel_16bit_value(0, 10000)
    print("'_set_channel_16bit_value(0, 10000)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        pixels._set_channel_16bit_value(0, 10000)
        pixels._set_channel_16bit_value(1, 10000)
        pixels._set_channel_16bit_value(2, 10000)
    print("'_set_channel_16bit_value(0..2, 10000)'")
    time_meassurement_call(_test, loop_count)

    def _test():
        for i in range(pixel_count * 3):
            pixels._set_channel_16bit_value(i, 500)
    print(
        "'_set_channel_16bit_value(for 0..{}, 10000)'"
        "".format(pixel_count * 3)
    )
    time_meassurement_call(_test, 10)


def time_meassurement():
    """Meassure timming."""
    print("meassure timming:")
    time_meassurement_pixels_show()
    time_meassurement_pixels_set()
    time_meassurement_channel_set()
    time_meassurement_channel_set_internal()
    set_all((0, 1, 1))

##########################################


def test_main():
    """Test Main."""
    print()
    print(42 * '*')
    print(__doc__)
    print(42 * '*')
    print()
    time.sleep(0.5)
    print(42 * '*')

    time_meassurement()
    time.sleep(0.5)
    print(42 * '*')

    while True:
        channelcheck_update()
        time.sleep(0.5)


##########################################
# main loop

if __name__ == '__main__':

    test_main()
