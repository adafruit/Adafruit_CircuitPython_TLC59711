
# The MIT License (MIT).
#
# Copyright (c) 2017 Tony DiCola for Adafruit Industries
# Copyright (c) 2018 Stefan Krüger s-light.eu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
`adafruit_tlc59711_multi`.

====================================================

CircuitPython module for the
TLC59711 or TLC5971 16-bit 12 channel LED PWM driver.
See examples/simpletest_multi.py for a demo of the usage.

* Author(s): Tony DiCola, Stefan Kruger

Implementation Notes
--------------------

**Hardware:**

* Adafruit `12-Channel 16-bit PWM LED Driver - SPI Interface - TLC59711
  <https://www.adafruit.com/product/1455>`_ (Product ID: 1455)
  or TLC5971

**Software and Dependencies:**

* this is a variant with multi-chip support.
    The API is mostly compatible to the DotStar / NeoPixel Libraries
    and is therefore also compatible with FancyLED.
    for this see examples/fancy_multi.py

* Adafruit CircuitPython firmware for the ESP8622, M0 or M4-based boards:
  https://github.com/adafruit/circuitpython/releases
"""
__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TLC59711.git"


# Globally disable invalid-name check as this chip by design has short channel
# and register names.  It is confusing to rename these from what the datasheet
# refers to them as.
# pylint: disable=invalid-name

# Globally disable too many instance attributes check.  Again this is a case
# where pylint doesn't have the right context to make this call.  The chip by
# design has many channels which must be exposed.
# pylint: disable=too-many-instance-attributes

# Globally disable protected access.  Once again pylint can't figure out the
# context for using internal decorate classes below.
# In these cases protectected access is by design for the internal class.
# pylint: disable=protected-access

# Yet another pylint issue, it fails to recognize a decorator class by
# definition has no public methods.  Disable the check.
# pylint: disable=too-few-public-methods


# this is not available in CircuitPython
# import uctypes as ctypes

class TLC59711Multi:
    """Multi TLC59711 16-bit 12 channel LED PWM driver.

    This chip is designed to drive 4 RGB LEDs with 16-bit PWM per Color.
    The class has an interface compatible with the FancyLED.
    and with this is similar to the NeoPixel and DotStar Interfaces.

    :param ~busio.SPI spi: An instance of the SPI bus connected to the chip.
        The clock and MOSI/outout must be set, the MISO/input is unused.
        Maximal data clock frequence is:
            - TLC59711: 10MHz
            - TLC5971: 20MHz
    :param bool pixel_count: Number of RGB-LEDs (=Pixels) are connected.
    """

    """
    TLC5971 data / register structure

    some detailed information on the protocol based on
    http://www.ti.com/lit/ds/symlink/tlc5971.pdf
    8.5.4 Register and Data Latch Configuration (page 23ff)
    9.2.2.3 How to Control the TLC5971 (page27)

    How to send:
    the first data we send are received by the last device in chain.
    Device Nth (244Bit = 28Byte)
    Write Command (6Bit)
        WRCMD (fixed: 25h)
    Function Control Data (5 x 1Bit = 5Bit)
        OUTTMG 1bit
            GS clock edge select
            1=rising edge, 0= falling edge
        EXTGCK 1bit
            GS reference clock select
            1=SCKI clock, 0=internal oscillator
        TMGRST 1bit
            display timing reset mode
            1=OUT forced of on latchpulse, 0=no forced reset
        DSPRPT 1bit
            display repeat mode
            1=auto repeate
            0=Out only turned on after Blank or internal latchpulse
        BLANK 1bit;
            1=blank (outputs off)
            0=Out on - controlled by GS-Data
            ic power on sets this to 1
    BC-Data (3 x 7Bits = 21Bit)
        BCB 7bit;
        BCG 7bit;
        BCR 7bit;
    GS-Data (12 x 16Bits = 192Bit)
        GSB3 16bit;
        GSG3 16bit;
        GSR3 16bit;
        GSB2 16bit;
        GSG2 16bit;
        GSR2 16bit;
        GSB1 16bit;
        GSG1 16bit;
        GSR1 16bit;
        GSB0 16bit;
        GSG0 16bit;
        GSR0 16bit;
    Device Nth-1 (244Bit = 28Byte)
        Device ..
        Device 2
        Device 1
        short break of 8x period of clock (666ns .. 2.74ms)
        to generate latchpulse
        + 1.34uS
        than next update.
    """

    ##########################################
    # helper
    ##########################################

    CHIP_BUFFER_LENGTH = 28

    COLORS_PER_PIXEL = 3
    PIXEL_PER_CHIP = 4
    CHANNEL_PER_CHIP = COLORS_PER_PIXEL * PIXEL_PER_CHIP
    BUFFER_BYTES_PER_COLOR = 2
    BUFFER_BYTES_PER_PIXEL = BUFFER_BYTES_PER_COLOR * COLORS_PER_PIXEL

    @staticmethod
    def set_bit_with_mask(v, mask, x):
        """Set bit with help of mask."""
        # clear
        v &= ~mask
        if x:
            # set
            v |= mask
        return v

    @staticmethod
    def set_bit(v, index, x):
        """Set bit - return new value.

        Set the index:th bit of v to 1 if x is truthy,
        else to 0, and return the new value.
        https://stackoverflow.com/a/12174051/574981
        """
        # Compute mask, an integer with just bit 'index' set.
        mask = 1 << index
        # Clear the bit indicated by the mask (if x is False)
        v &= ~mask
        if x:
            # If x was True, set the bit indicated by the mask.
            v |= mask
        # Return the result, we're done.
        return v

    class _GS_Value:
        # Internal decorator to simplify exposing each 16-bit LED PWM channel.
        # These will get/set the appropriate bytes in the shift register with
        # the specified values.

        def __init__(self, byte_offset):
            # Keep track of the byte within the shift register where this
            # 16-bit value starts.  Luckily these are all aligned on byte
            # boundaries.  Note the byte order is big endian (MSB first).
            self._byte_offset = byte_offset

        def __get__(self, obj, obj_type):
            # Grab the 16-bit value at the offset for this channel.
            return (obj._buffer[self._byte_offset] << 8) | \
                obj._buffer[self._byte_offset + 1]

        def __set__(self, obj, val):
            # Set the 16-bit value at the offset for this channel.
            assert 0 <= val <= 65535
            obj._buffer[self._byte_offset] = (val >> 8) & 0xFF
            obj._buffer[self._byte_offset + 1] = val & 0xFF

    ##########################################
    # class _BC():
    """
    BC-Data (3 x 7Bits = 21Bit).

    BCB 7bit;
    BCG 7bit;
    BCR 7bit;
    """
    _BC_CHIP_BUFFER_BIT_OFFSET = 0
    _BC_BIT_COUNT = 3 * 7
    # this holds the chip offset and
    _BC_FIELDS = {
        "BCB": {
            "offset": 0,
            "length": 7,
            "mask": 0b01111111,
        },
        "BCG": {
            "offset": 7,
            "length": 7,
            "mask": 0b01111111,
        },
        "BCR": {
            "offset": 14,
            "length": 7,
            "mask": 0b01111111,
        },
    }

    ##########################################
    # class _FC():
    """
    Function Control Data (5 x 1Bit = 5Bit).

    OUTTMG 1bit
        GS clock edge select
        1 = rising edge
        0 = falling edge
    EXTGCK 1bit
        GS reference clock select
        1 = SCKI clock
        0 = internal oscillator
    TMGRST 1bit
        display timing reset mode
        1 = OUT forced of on latchpulse
        0 = no forced reset
    DSPRPT 1bit
        display repeat mode
        1 = auto repeate
        0 = Out only turned on after Blank or internal latchpulse
    BLANK 1bit;
        ic power on sets this to 1
        1 = blank (outputs off)
        0 = Out on - controlled by GS-Data
    """

    _FC_CHIP_BUFFER_BIT_OFFSET = _BC_BIT_COUNT
    _FC_BIT_COUNT = 5
    _FC_FIELDS = {
        "BLANK": {
            "offset": 0,
            "length": 1,
            "mask": 0b1,
        },
        "DSPRPT": {
            "offset": 1,
            "length": 1,
            "mask": 0b1,
        },
        "TMGRST": {
            "offset": 2,
            "length": 1,
            "mask": 0b1,
        },
        "EXTGCK": {
            "offset": 3,
            "length": 1,
            "mask": 0b1,
        },
        "OUTTMG": {
            "offset": 4,
            "length": 1,
            "mask": 0b1,
        },
    }

    ##########################################
    # class _WRITE_COMMAND():
    """WRITE_COMMAND."""

    _WC_CHIP_BUFFER_BIT_OFFSET = _FC_BIT_COUNT + _BC_BIT_COUNT
    _WC_BIT_COUNT = 6
    _WC_FIELDS = {
        "WRITE_COMMAND": {
            "offset": 0,
            "length": 6,
            "mask": 0b111111,
        },
    }
    WRITE_COMMAND = 0b100101
    ##########################################

    ########
    CHIP_BUFFER_HEADER_BIT_COUNT = \
        _WC_BIT_COUNT + _FC_BIT_COUNT + _BC_BIT_COUNT
    CHIP_BUFFER_HEADER_BYTE_COUNT = CHIP_BUFFER_HEADER_BIT_COUNT // 8

    ##########################################

    def set_chipheader_bits_in_buffer(
            self,
            *, # noqa
            chip_index=0,
            part_bit_offset=0,
            field={"mask": 0, "length": 0, "offset": 0},
            value=0
    ):
        """Set chip header bits in buffer."""
        # print(
        #     "chip_index={} "
        #     "part_bit_offset={} "
        #     "field={} "
        #     "value={} "
        #     "".format(
        #         chip_index,
        #         part_bit_offset,
        #         field,
        #         value
        #     )
        # )
        offset = part_bit_offset + field["offset"]
        # restrict value
        value &= field["mask"]
        # move value to position
        value = value << offset
        # calculate header start
        header_start = chip_index * self.CHIP_BUFFER_LENGTH
        # get chip header
        header = self._get_32bit_value_from_buffer(header_start)
        # print("{:032b}".format(header))
        # 0xFFFFFFFF == 0b11111111111111111111111111111111
        # create/move mask
        mask = field["mask"] << offset
        # clear
        header &= ~mask
        # set
        header |= value
        # write header back
        self._set_32bit_value_in_buffer(header_start, header)

    ##########################################

    def __init__(self, spi, pixel_count=1):
        """Init."""
        self._spi = spi
        # how many pixels are there?
        self.pixel_count = pixel_count
        self.channel_count = self.pixel_count * self.COLORS_PER_PIXEL
        # calculate how many chips are connected
        self.chip_count = self.pixel_count // 4

        # THe chips are just a big 28 byte long shift register without any
        # fancy update protocol.  Blast out all the bits to update, that's it!
        # create raw output data
        self._buffer = bytearray(self.CHIP_BUFFER_LENGTH * self.chip_count)

        # Initialize the brightness channel values to max (these are 7-bit
        # values).
        self._bcr = 127
        self._bcg = 127
        self._bcb = 127

        # Initialize external user-facing state for the function control
        # bits of the chip.  These aren't commonly used but available and
        # match the nomenclature from the datasheet.
        # you must manually call update_fc() after changing them
        # (reduces the need to make frivolous memory-hogging properties).
        # Default set
        # OUTTMG, TMGRST, and DSPRPT to on
        # like in Arduino library.
        # these are set for all chips
        self.outtmg = True
        self.extgck = False
        self.tmgrst = True
        self.dsprpt = True
        self.blank = False

        # preparation done
        # now initialize buffer to default values

        # self._debug_print_buffer()
        print("init buffer..")
        self._init_buffer()
        # self._debug_print_buffer()
        print("-> done")

        self._buffer_index_lookuptable = []
        self._init_lookuptable()

    def _init_buffer(self):
        for chip_index in range(self.chip_count):
            # set Write Command (6Bit) WRCMD (fixed: 25h)
            # buffer_start = chip_index * self.CHIP_BUFFER_LENGTH
            # self._buffer[buffer_start] = 0x25 << 2

            # self._debug_print_buffer()
            self.chip_set_BCData(
                chip_index, bcr=self._bcr, bcg=self._bcg, bcb=self._bcb)
            # self._debug_print_buffer()
            self._chip_set_FunctionControl(chip_index)
            # self._debug_print_buffer()
            self._chip_set_WriteCommand(chip_index)
            # self._debug_print_buffer()
        # loop end

    def chip_set_BCData(self, chip_index, bcr=127, bcg=127, bcb=127):
        """
        Set BC-Data.

        :param int chip_index: Index of Chip to set.
        :param int bcr: 7-bit value from 0-127 (default=127)
        :param int bcg: 7-bit value from 0-127 (default=127)
        :param int bcb: 7-bit value from 0-127 (default=127)
        """
        # set all bits
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._BC_CHIP_BUFFER_BIT_OFFSET,
            field=self._BC_FIELDS["BCR"],
            value=bcr)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._BC_CHIP_BUFFER_BIT_OFFSET,
            field=self._BC_FIELDS["BCG"],
            value=bcg)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._BC_CHIP_BUFFER_BIT_OFFSET,
            field=self._BC_FIELDS["BCB"],
            value=bcb)

    def _chip_set_FunctionControl(self, chip_index):
        """
        Set Function Control Bits in Buffer.

        values from object global parameters are used.

        :param int chip_index: Index of Chip to set.
        """
        # set all bits
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["OUTTMG"],
            value=self.outtmg)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["EXTGCK"],
            value=self.extgck)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["TMGRST"],
            value=self.tmgrst)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["DSPRPT"],
            value=self.dsprpt)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["BLANK"],
            value=self.blank)

    def update_fc(self):
        """
        Update Function Control Bits for all Chips in Buffer.

        need to be called after you changed on of the
        Function Control Bit Parameters.
        (outtmg, extgck, tmgrst, dsprpt, blank)
        """
        for chip_index in range(self.chip_count):
            self._chip_set_FunctionControl(chip_index)

    def _chip_set_WriteCommand(self, chip_index):
        """Set WRITE_COMMAND."""
        # set all bits
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=self._WC_CHIP_BUFFER_BIT_OFFSET,
            field=self._WC_FIELDS["WRITE_COMMAND"],
            value=self.WRITE_COMMAND)

    def _init_lookuptable(self):
        for channel_index in range(self.channel_count):
            buffer_index = (
                (self.CHIP_BUFFER_LENGTH // self.BUFFER_BYTES_PER_COLOR)
                * (channel_index // self.CHANNEL_PER_CHIP)
                + channel_index % self.CHANNEL_PER_CHIP
            )
            buffer_index *= self.BUFFER_BYTES_PER_COLOR
            buffer_index += self.CHIP_BUFFER_HEADER_BYTE_COUNT
            self._buffer_index_lookuptable.append(buffer_index)

    ##########################################

    def _write(self):
        # Write out the current state to the shift register.
        # self._debug_print_buffer()
        try:
            # Lock the SPI bus and configure it for the shift register.
            while not self._spi.try_lock():
                pass
            self._spi.configure(baudrate=10000000, polarity=0, phase=0)
            self._spi.write(self._buffer)
        finally:
            # Ensure the SPI bus is unlocked.
            self._spi.unlock()

    def show(self):
        """Write out the current LED PWM state to the chip."""
        self._write()

    def _debug_print_buffer(self):
        indent = "  "
        if self.chip_count == 1:
            print("buffer: [", end="")
            indent = ""
        else:
            print("buffer: [")
        for index in range(self.chip_count):
            print("{}{}: ".format(indent, index), end="")
            # print()
            # self._debug_print_tlc59711_bin()
            # print()
            # self._debug_print_tlc59711_hex()
            self._debug_print_tlc59711()
            if self.chip_count > 1:
                print()
        print("]")

    def _debug_print_tlc59711_bin(self):
        for index in range(len(self._buffer)):
            print("{:08b}".format(self._buffer[index]), end="")

    def _debug_print_tlc59711_hex(self):
        for index in range(len(self._buffer)):
            print("x{:02X}, ".format(self._buffer[index]), end="")

    def _debug_print_tlc59711(self):
        self._debug_print_tlc59711_header()
        self._debug_print_tlc59711_ch()

    def _debug_print_tlc59711_header(self):
        # print(
        #     "self.CHIP_BUFFER_HEADER_BYTE_COUNT",
        #     self.CHIP_BUFFER_HEADER_BYTE_COUNT)
        print("header: '", end="")
        for index in range(self.CHIP_BUFFER_HEADER_BYTE_COUNT):
            print("{:08b} ".format(self._buffer[index]), end="")
        print("' ", end="")

    def _debug_print_tlc59711_ch(self):
        print("ch: [", end="")
        for index in range(
                self.CHIP_BUFFER_HEADER_BYTE_COUNT, len(self._buffer)
        ):
            print("x{:02X}, ".format(self._buffer[index]), end="")
        print("]", end="")

    # Define properties for global brightness control channels.
    # @property
    # def red_brightness(self):
    #     """
    #     Red brightness for all channels on all chips.
    #
    #     This is a 7-bit value from 0-127.
    #     """
    #     return self._bcr
    #
    # @red_brightness.setter
    # def red_brightness(self, val):
    #     assert 0 <= val <= 127
    #     self._bcr = val
    #     if self.auto_show:
    #         self._write()
    #
    # @property
    # def green_brightness(self):
    #     """
    #     Green brightness for all channels on all chips.
    #
    #     This is a 7-bit value from 0-127.
    #     """
    #     return self._bcg
    #
    # @green_brightness.setter
    # def green_brightness(self, val):
    #     assert 0 <= val <= 127
    #     self._bcg = val
    #     if self.auto_show:
    #         self._write()
    #
    # @property
    # def blue_brightness(self):
    #     """
    #     Blue brightness for all channels on all chips.
    #
    #     This is a 7-bit value from 0-127.
    #     """
    #     return self._bcb
    #
    # @blue_brightness.setter
    # def blue_brightness(self, val):
    #     assert 0 <= val <= 127
    #     self._bcb = val
    #     if self.auto_show:
    #         self._write()

    def _get_32bit_value_from_buffer(self, buffer_start):
        return (
            (self._buffer[buffer_start + 0] << 24) |
            (self._buffer[buffer_start + 1] << 16) |
            (self._buffer[buffer_start + 2] << 8) |
            self._buffer[buffer_start + 3]
        )

    def _set_32bit_value_in_buffer(self, buffer_start, value):
        assert 0 <= value <= 0xFFFFFFFF
        # print("buffer_start", buffer_start, "value", value)
        # self._debug_print_buffer()
        self._buffer[buffer_start + 0] = (value >> 24) & 0xFF
        self._buffer[buffer_start + 1] = (value >> 16) & 0xFF
        self._buffer[buffer_start + 2] = (value >> 8) & 0xFF
        self._buffer[buffer_start + 3] = value & 0xFF

    def _get_16bit_value_from_buffer(self, buffer_start):
        return (
            (self._buffer[buffer_start + 0] << 8) |
            self._buffer[buffer_start + 1]
        )

    def _set_16bit_value_in_buffer(self, buffer_start, value):
        # assert 0 <= value <= 65535
        # print("buffer_start", buffer_start, "value", value)
        # self._debug_print_buffer()
        self._buffer[buffer_start + 0] = (value >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value & 0xFF

    @staticmethod
    def _convert_01_float_to_16bit_integer(value):
        """Convert 0..1 Float Value to 16bit (0..65535) Range."""
        # check if value is in range
        assert 0 <= value <= 1
        # convert to 16bit value
        return int(value * 65535)

    @classmethod
    def _convert_if_float(cls, value):
        """Convert if value is Float."""
        if isinstance(value, float):
            value = cls._convert_01_float_to_16bit_integer(value)
        return value

    def _get_channel_16bit_value(self, channel_index):
        return self._get_16bit_value_from_buffer(
            self._buffer_index_lookuptable[channel_index],
        )

    def _set_channel_16bit_value(self, channel_index, value):
        # self._set_16bit_value_in_buffer(
        #     self._buffer_index_lookuptable[channel_index],
        #     value
        # )
        # optimized:
        buffer_start = self._buffer_index_lookuptable[channel_index]
        self._buffer[buffer_start + 0] = (value >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value & 0xFF

    def set_pixel_16bit_value(self, pixel_index, value_r, value_g, value_b):
        """
        Set the value for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.

        :param int pixel_index: 0..(pixel_count)
        :param int value_r: 0..65535
        :param int value_g: 0..65535
        :param int value_b: 0..65535
        """
        pixel_start = pixel_index * self.COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_b & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_g & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_r & 0xFF

    def set_pixel_float_value(self, pixel_index, value_r, value_g, value_b):
        """
        Set the value for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.

        :param int pixel_index: 0..(pixel_count)
        :param int value_r: 0..1
        :param int value_g: 0..1
        :param int value_b: 0..1
        """
        value_r = int(value_r * 65535)
        value_g = int(value_g * 65535)
        value_b = int(value_b * 65535)
        pixel_start = pixel_index * self.COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_b & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_g & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_r & 0xFF

    def set_pixel_16bit_color(self, pixel_index, value):
        """
        Set color for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.
        its a little bit slower as `set_pixel_16bit_value`

        :param int pixel_index: 0..(pixel_count)
        :param int 3-tuple of R, G, B;  0..65535
        """
        pixel_start = pixel_index * self.COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (value[2] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value[2] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (value[1] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value[1] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (value[0] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value[0] & 0xFF

    def set_pixel_float_color(self, pixel_index, value):
        """
        Set color for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.
        its a little bit slower as `set_pixel_16bit_value`

        :param int pixel_index: 0..(pixel_count)
        :param tuple/float 3-tuple of R, G, B;  0..1
        """
        # convert tuple to list
        # this way we can assign values.
        # this seems faster than creating tree new variables
        value = list(value)
        # convert to 16bit int
        value[0] = int(value[0] * 65535)
        value[1] = int(value[1] * 65535)
        value[2] = int(value[2] * 65535)
        # calculate pixel_start
        pixel_start = pixel_index * self.COLORS_PER_PIXEL
        # set values
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (value[2] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value[2] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (value[1] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value[1] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (value[0] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value[0] & 0xFF

        # value_r = int(value[0] * 65535)
        # value_g = int(value[1] * 65535)
        # value_b = int(value[2] * 65535)
        # pixel_start = pixel_index * self.COLORS_PER_PIXEL
        # buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        # self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        # self._buffer[buffer_start + 1] = value_b & 0xFF
        # buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        # self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        # self._buffer[buffer_start + 1] = value_g & 0xFF
        # buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        # self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        # self._buffer[buffer_start + 1] = value_r & 0xFF

    def set_pixel(self, pixel_index, value):
        """
        Set the R, G, B values for the pixel.

        this funciton hase some advanced error checking.
        it is much slower than the other provided 'bare' variants..
        but therefor gives clues to what is going wrong.. ;-)

        :param int pixel_index: 0..(pixel_count)
        :param tuple 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        if 0 <= pixel_index < self.pixel_count:
            # print("value", value)
            # convert to list
            value = list(value)
            # print("value", value)
            # print("rep:")
            # repr(value)
            # print("check length..")
            if len(value) != self.COLORS_PER_PIXEL:
                raise IndexError(
                    "length of value {} does not match COLORS_PER_PIXEL (= {})"
                    "".format(len(value), self.COLORS_PER_PIXEL)
                )
            # tested:
            # splitting up into variables to not need the list..
            # this is about 0.25ms slower!
            # value_r = value[0]
            # value_g = value[1]
            # value_b = value[2]

            # check if we have float values
            # value[0] = self._convert_if_float(value[0])
            # value[1] = self._convert_if_float(value[1])
            # value[2] = self._convert_if_float(value[2])

            # check if values are in range
            # assert 0 <= value[0] <= 65535
            # assert 0 <= value[1] <= 65535
            # assert 0 <= value[2] <= 65535

            # optimize:
            # check if we have float values
            if isinstance(value[0], float):
                # check if value is in range
                assert 0 <= value[0] <= 1
                # convert to 16bit value
                value[0] = int(value[0] * 65535)
            else:
                assert 0 <= value[0] <= 65535
            if isinstance(value[1], float):
                # check if value is in range
                assert 0 <= value[1] <= 1
                # convert to 16bit value
                value[1] = int(value[1] * 65535)
            else:
                assert 0 <= value[1] <= 65535
            if isinstance(value[2], float):
                # check if value is in range
                assert 0 <= value[2] <= 1
                # convert to 16bit value
                value[2] = int(value[2] * 65535)
            else:
                assert 0 <= value[2] <= 65535

            # print("value", value)

            # update buffer
            # print("pixel_index", pixel_index, "value", value)
            # we change channel order here:
            # buffer channel order is blue, green, red
            pixel_start = pixel_index * self.COLORS_PER_PIXEL
            # self._set_channel_16bit_value(
            #     pixel_start + 0,
            #     value[2])
            # self._set_channel_16bit_value(
            #     pixel_start + 1,
            #     value[1])
            # self._set_channel_16bit_value(
            #     pixel_start + 2,
            #     value[0])
            # optimize:
            # self._set_16bit_value_in_buffer(
            #     self._buffer_index_lookuptable[pixel_start + 0],
            #     value[2])
            # self._set_16bit_value_in_buffer(
            #     self._buffer_index_lookuptable[pixel_start + 1],
            #     value[1])
            # self._set_16bit_value_in_buffer(
            #     self._buffer_index_lookuptable[pixel_start + 2],
            #     value[0])
            # optimize2
            buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
            self._buffer[buffer_start + 0] = (value[2] >> 8) & 0xFF
            self._buffer[buffer_start + 1] = value[2] & 0xFF
            buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
            self._buffer[buffer_start + 0] = (value[1] >> 8) & 0xFF
            self._buffer[buffer_start + 1] = value[1] & 0xFF
            buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
            self._buffer[buffer_start + 0] = (value[0] >> 8) & 0xFF
            self._buffer[buffer_start + 1] = value[0] & 0xFF
        else:
            raise IndexError(
                "index {} out of range [0..{}]"
                "".format(pixel_index, self.pixel_count)
            )

    # channel access
    def set_channel(self, channel_index, value):
        """
        Set the value for the provided channel.

        :param int channel_index: 0..(channel_count)
        :param int value: 0..65535
        """
        if 0 <= channel_index < (self.channel_count):
            # check if values are in range
            assert 0 <= value <= 65535
            # temp = channel_index
            # we change channel order here:
            # buffer channel order is blue, green, red
            pixel_index_offset = channel_index % self.COLORS_PER_PIXEL
            if pixel_index_offset == 0:
                channel_index += 2
            elif pixel_index_offset == 2:
                channel_index -= 2
            # print("{:>2} → {:>2}".format(temp, channel_index))
            self._set_16bit_value_in_buffer(
                self._buffer_index_lookuptable[channel_index],
                value
            )
        else:
            raise IndexError(
                "channel_index {} out of range (0..{})".format(
                    channel_index,
                    self.channel_count
                )
            )

    # Define index and length properties to set and get each channel as
    # atomic RGB tuples.  This provides a similar feel as using neopixels.
    def __len__(self):
        """Retrieve TLC5975 the total number of Pixels available."""
        return self.pixel_count

    def __getitem__(self, key):
        """
        Retrieve the R, G, B values for the provided channel as a 3-tuple.

        Each value is a 16-bit number from 0-65535.
        """
        if 0 <= key < self.pixel_count:
            pixel_start = key * self.COLORS_PER_PIXEL
            return (
                self._get_channel_16bit_value(pixel_start + 0),
                self._get_channel_16bit_value(pixel_start + 1),
                self._get_channel_16bit_value(pixel_start + 2)
            )
        else:
            raise IndexError(
                "index {} out of range [0..{}]"
                "".format(key, self.pixel_count)
            )

    def __setitem__(self, key, value):
        """
        Set the R, G, B values for the pixel.

        this funciton hase some advanced error checking.
        it is much slower than the other provided 'bare' variants..
        but therefor gives clues to what is going wrong.. ;-)
        this shortcut is identicall to `set_pixel`

        :param int key: 0..(pixel_count)
        :param tuple 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        if 0 <= key < self.pixel_count:
            # print("value", value)
            # convert to list
            value = list(value)
            # print("value", value)
            # print("rep:")
            # repr(value)
            # print("check length..")
            if len(value) != self.COLORS_PER_PIXEL:
                raise IndexError(
                    "length of value {} does not match COLORS_PER_PIXEL (= {})"
                    "".format(len(value), self.COLORS_PER_PIXEL)
                )
            # tested:
            # splitting up into variables to not need the list..
            # this is about 0.25ms slower!
            # value_r = value[0]
            # value_g = value[1]
            # value_b = value[2]

            # check if we have float values
            # value[0] = self._convert_if_float(value[0])
            # value[1] = self._convert_if_float(value[1])
            # value[2] = self._convert_if_float(value[2])

            # check if values are in range
            # assert 0 <= value[0] <= 65535
            # assert 0 <= value[1] <= 65535
            # assert 0 <= value[2] <= 65535

            # optimize:
            # check if we have float values
            if isinstance(value[0], float):
                # check if value is in range
                assert 0 <= value[0] <= 1
                # convert to 16bit value
                value[0] = int(value[0] * 65535)
            else:
                assert 0 <= value[0] <= 65535
            if isinstance(value[1], float):
                # check if value is in range
                assert 0 <= value[1] <= 1
                # convert to 16bit value
                value[1] = int(value[1] * 65535)
            else:
                assert 0 <= value[1] <= 65535
            if isinstance(value[2], float):
                # check if value is in range
                assert 0 <= value[2] <= 1
                # convert to 16bit value
                value[2] = int(value[2] * 65535)
            else:
                assert 0 <= value[2] <= 65535

            # print("value", value)

            # update buffer
            # print("key", key, "value", value)
            # we change channel order here:
            # buffer channel order is blue, green, red
            pixel_start = key * self.COLORS_PER_PIXEL
            # self._set_channel_16bit_value(
            #     pixel_start + 0,
            #     value[2])
            # self._set_channel_16bit_value(
            #     pixel_start + 1,
            #     value[1])
            # self._set_channel_16bit_value(
            #     pixel_start + 2,
            #     value[0])
            # optimize:
            # self._set_16bit_value_in_buffer(
            #     self._buffer_index_lookuptable[pixel_start + 0],
            #     value[2])
            # self._set_16bit_value_in_buffer(
            #     self._buffer_index_lookuptable[pixel_start + 1],
            #     value[1])
            # self._set_16bit_value_in_buffer(
            #     self._buffer_index_lookuptable[pixel_start + 2],
            #     value[0])
            # optimize2
            buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
            self._buffer[buffer_start + 0] = (value[2] >> 8) & 0xFF
            self._buffer[buffer_start + 1] = value[2] & 0xFF
            buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
            self._buffer[buffer_start + 0] = (value[1] >> 8) & 0xFF
            self._buffer[buffer_start + 1] = value[1] & 0xFF
            buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
            self._buffer[buffer_start + 0] = (value[0] >> 8) & 0xFF
            self._buffer[buffer_start + 1] = value[0] & 0xFF
        else:
            raise IndexError(
                "index {} out of range [0..{}]"
                "".format(key, self.pixel_count)
            )

##########################################
