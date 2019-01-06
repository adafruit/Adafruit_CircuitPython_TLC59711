"""TLC5971 / TLC59711 Multi."""

__doc__ = """
TLC59711Multi development helper.

this sketch contains a bunch of timing tests and other development helpers..
Enjoy the colors :-)
"""

import time

import board
import busio

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
    """Channel check pixel."""
    global offset  #noqa
    # print("offset", offset)

    # pixels[offset] = (value_high, 0, 0)
    pixels.set_pixel_16bit_value(offset, value_high, 0, 0)
    # clear last pixel
    last = offset-1
    if last < 0:
        last = pixel_count-1
    # pixels[last] = (0, 0, 1)
    pixels.set_pixel_16bit_value(last, 0, 0, 1)
    # pixels[offset] = (0xAAAA, 0xBBBB, 0xCCCC)
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


##########################################

def time_measurement_call(message, test_function, loop_count=1000):
    """Measure timing."""
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
    # print(
    #     "\t{:.2f}ms per call"
    #     "".format((duration / loop_count) * 1000)
    # )
    # "{:>8.2f}ms".format(3.56)
    print(
        "{call_duration:>8.2f}ms\t{message}"
        "".format(
            call_duration=(duration / loop_count) * 1000,
            message=message,
        )
    )


def time_measurement_pixels_show():
    """Measure timing."""
    print("*** pixels show:")
    loop_count = 1000

    def _test():
        pixels.show()
    time_measurement_call(
        "'pixels.show()'",
        _test,
        loop_count
    )


def time_measurement_pixels_set_single():
    """Measure timing pixels set."""
    print("*** pixels set single:")
    loop_count = 1000

    def _test():
        pixels[3] = (500, 40500, 1000)
    time_measurement_call(
        "'pixels[3] = (500, 40500, 1000)'",
        _test,
        loop_count
    )

    def _test():
        pixels[3] = (0.1, 0.5, 0.9)
    time_measurement_call(
        "'pixels[3] = (0.1, 0.5, 0.9)'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_pixel(3, (500, 40500, 1000))
    time_measurement_call(
        "'pixels.set_pixel(3, (500, 40500, 1000))'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_pixel(3, (0.1, 0.5, 0.9))
    time_measurement_call(
        "'pixels.set_pixel(3, (0.1, 0.5, 0.9))'",
        _test,
        loop_count
    )


def time_measurement_pixels_set_loop():
    """Measure timing pixels set."""
    print("*** pixels set loop:")
    loop_count = 10

    def _test():
        for i in range(pixel_count):
            pixels[i] = (500, 40500, 1000)
    time_measurement_call(
        "'pixels[for 0..{}] = (500, 40500, 1000)'".format(pixel_count),
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count):
            pixels[i] = (0.1, 0.5, 0.9)
    time_measurement_call(
        "'pixels[for 0..{}] = (0.1, 0.5, 0.9)'".format(pixel_count),
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel(i, (500, 40500, 1000))
    time_measurement_call(
        "'pixels.set_pixel(0..{}, (500, 40500, 1000))'".format(pixel_count),
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel(i, (0.1, 0.5, 0.9))
    time_measurement_call(
        "'pixels.set_pixel(0..{}, (0.1, 0.5, 0.9))'".format(pixel_count),
        _test,
        loop_count
    )


def time_measurement_pixels_set_all():
    """Measure timing pixels set."""
    print("*** pixels set all:")
    loop_count = 10

    def _test():
        pixels.set_pixel_all((500, 40500, 1000))
    time_measurement_call(
        "'pixels.set_pixel_all((500, 40500, 1000))'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_pixel_all((0.1, 0.5, 0.9))
    time_measurement_call(
        "'pixels.set_pixel_all((0.1, 0.5, 0.9))'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_pixel_all_16bit_value(500, 40500, 1000)
    time_measurement_call(
        "'pixels.set_pixel_all_16bit_value(500, 40500, 1000)'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_all_black()
    time_measurement_call(
        "'pixels.set_all_black()'",
        _test,
        loop_count
    )


def time_measurement_pixels_set_16bit():
    """Measure timing pixels set."""
    print("*** pixels set 16bit:")
    loop_count = 1000

    def _test():
        pixels.set_pixel_16bit_value(3, 500, 40500, 1000)
    time_measurement_call(
        "'pixels.set_pixel_16bit_value(3, 500, 40500, 1000)'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_pixel_16bit_color(3, (500, 40500, 1000))
    time_measurement_call(
        "'pixels.set_pixel_16bit_color(3, (500, 40500, 1000))'",
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel_16bit_value(i, 500, 40500, 1000)
    time_measurement_call(
        "'pixels.set_pixel_16bit_value(0..{}, 500, 40500, 1000)'"
        "".format(pixel_count),
        _test,
        10
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel_16bit_color(i, (500, 40500, 1000))
    time_measurement_call(
        "'pixels.set_pixel_16bit_color(0..{}, (500, 40500, 1000))'"
        "".format(pixel_count),
        _test,
        10
    )


def time_measurement_pixels_set_float():
    """Measure timing pixels set."""
    print("*** pixels set float:")
    loop_count = 1000

    def _test():
        pixels.set_pixel_float_value(3, 0.1, 0.5, 0.9)
    time_measurement_call(
        "'pixels.set_pixel_float_value(3, 0.1, 0.5, 0.9)'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_pixel_float_color(3, (0.1, 0.5, 0.9))
    time_measurement_call(
        "'pixels.set_pixel_float_color(3, (0.1, 0.5, 0.9))'",
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel_float_value(i, 0.1, 0.5, 0.9)
    time_measurement_call(
        "'pixels.set_pixel_float_value(0..{}, 0.1, 0.5, 0.9)'"
        "".format(pixel_count),
        _test,
        10
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel_float_color(i, (0.1, 0.5, 0.9))
    time_measurement_call(
        "'pixels.set_pixel_float_color(0..{}, (0.1, 0.5, 0.9))'"
        "".format(pixel_count),
        _test,
        10
    )

    def _test():
        for i in range(pixel_count):
            pixels.set_pixel_16bit_value(
                i,
                int(0.1 * 65535),
                int(0.5 * 65535),
                int(0.9 * 65535)
            )
    time_measurement_call(
        "'pixels.set_pixel_16bit_value(0..{}, f2i 0.1, f2i 0.5, f2i 0.9)'"
        "".format(pixel_count),
        _test,
        10
    )


def time_measurement_channel_set():
    """Measure timing channel set."""
    print("*** channel set:")
    loop_count = 1000

    def _test():
        pixels.set_channel(0, 10000)
    time_measurement_call(
        "'set_channel(0, 10000)'",
        _test,
        loop_count
    )

    def _test():
        pixels.set_channel(0, 10000)
        pixels.set_channel(1, 10000)
        pixels.set_channel(2, 10000)
    time_measurement_call(
        "'set_channel(0..2, 10000)'",
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count * 3):
            pixels.set_channel(i, 500)
    time_measurement_call(
        "'set_channel(for 0..{}, 10000)'"
        "".format(pixel_count * 3),
        _test,
        10
    )


def time_measurement_channel_set_internal():
    """Measure timing channel set internal."""
    print("*** channel set internal:")
    loop_count = 1000

    def _test():
        pixels._set_channel_16bit_value(0, 10000)
    time_measurement_call(
        "'_set_channel_16bit_value(0, 10000)'",
        _test,
        loop_count
    )

    def _test():
        pixels._set_channel_16bit_value(0, 10000)
        pixels._set_channel_16bit_value(1, 10000)
        pixels._set_channel_16bit_value(2, 10000)
    time_measurement_call(
        "'_set_channel_16bit_value(0..2, 10000)'",
        _test,
        loop_count
    )

    def _test():
        for i in range(pixel_count * 3):
            pixels._set_channel_16bit_value(i, 500)
    time_measurement_call(
        "'_set_channel_16bit_value(for 0..{}, 10000)'"
        "".format(pixel_count * 3),
        _test,
        10
    )


def time_measurement_pixels_get():
    """Measure timing pixels get."""
    print("*** pixels get:")

    pixels.set_pixel_all((1, 11, 111))

    def _test():
        print("[", end="")
        for i in range(pixel_count):
            print("{}:{}, ".format(i, pixels[i]), end="")
        print("]", end="")
    time_measurement_call(
        "'print('{}:{}, '.format(i, pixels[i]), end='')'",
        _test,
        1
    )


def time_measurement():
    """Measure timing."""
    print("meassure timing:")
    time_measurement_pixels_show()
    time_measurement_pixels_set_single()
    time_measurement_pixels_set_loop()
    time_measurement_pixels_set_all()
    time_measurement_pixels_set_16bit()
    time_measurement_pixels_set_float()
    time_measurement_channel_set()
    time_measurement_channel_set_internal()
    time_measurement_pixels_get()
    pixels.set_pixel_all((0, 1, 1))

##########################################


def test_BCData():
    """Test BC-Data setting."""
    print("test BC-Data setting:")
    print("set pixel all to 100, 100, 100")
    pixels.set_pixel_all((100, 100, 100))
    pixels.show()
    time.sleep(2)
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
    # calculate bc values
    Ioclmax = TLC59711Multi.calculate_Ioclmax(Riref=2.7)
    print("Ioclmax = {}".format(Ioclmax))
    Riref = TLC59711Multi.calculate_Riref(Ioclmax=Ioclmax)
    print("Riref = {}".format(Riref))
    BCValues = TLC59711Multi.calculate_BCData(Ioclmax=Ioclmax)
    # (62, 103, 117)
    print("BCValues = {}".format(BCValues))

    print("set bcX to {}".format(BCValues))
    pixels.bcr = BCValues[0]
    pixels.bcg = BCValues[1]
    pixels.bcb = BCValues[2]
    pixels.update_BCData()
    pixels.show()
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
    time.sleep(2)

##########################################


def test_main():
    """Test Main."""
    print(42 * '*', end="")
    print(__doc__, end="")
    print(42 * '*')
    # print()
    # time.sleep(0.5)
    # print(42 * '*')

    pixels.set_pixel_all_16bit_value(1, 10, 100)
    pixels.show()
    time.sleep(0.5)

    test_BCData()
    time.sleep(0.5)
    print(42 * '*')

    time_measurement()
    time.sleep(0.5)
    print(42 * '*')

    print("loop:")
    while True:
        channelcheck_update()
        time.sleep(0.5)


##########################################
# main loop

if __name__ == '__main__':

    test_main()
