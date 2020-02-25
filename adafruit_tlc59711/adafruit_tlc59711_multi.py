
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

import struct

from micropython import const


class TLC59711Multi:
    """Multi TLC59711 16-bit 12 channel LED PWM driver.

    This chip is designed to drive 4 RGB LEDs with 16-bit PWM per Color.
    The class has an interface compatible with the FancyLED library.
    and with this is similar to the NeoPixel and DotStar Interfaces.

    :param ~busio.SPI spi: An instance of the SPI bus connected to the chip.
        The clock and MOSI/outout must be set, the MISO/input is unused.
        Maximal data clock frequence is:
        - TLC59711: 10MHz
        - TLC5971: 20MHz
    :param bool pixel_count: Number of RGB-LEDs (=Pixels) that are connected.
    """

    # pylint: disable=too-many-instance-attributes
    # it just does make senes to have the chip params as attributes.

    # """
    # TLC5971 data / register structure
    #
    # some detailed information on the protocol based on
    # http://www.ti.com/lit/ds/symlink/tlc5971.pdf
    # 8.5.4 Register and Data Latch Configuration (page 23ff)
    # 9.2.2.3 How to Control the TLC5971 (page27)
    #
    # How to send:
    # the first data we send are received by the last device in chain.
    # Device Nth (244Bit = 28Byte)
    # Write Command (6Bit)
    #     WRCMD (fixed: 25h)
    # Function Control Data (5 x 1Bit = 5Bit)
    #     OUTTMG 1bit
    #         GS clock edge select
    #         1=rising edge, 0= falling edge
    #     EXTGCK 1bit
    #         GS reference clock select
    #         1=SCKI clock, 0=internal oscillator
    #     TMGRST 1bit
    #         display timing reset mode
    #         1=OUT forced of on latchpulse, 0=no forced reset
    #     DSPRPT 1bit
    #         display repeat mode
    #         1=auto repeate
    #         0=Out only turned on after Blank or internal latchpulse
    #     BLANK 1bit;
    #         1=blank (outputs off)
    #         0=Out on - controlled by GS-Data
    #         ic power on sets this to 1
    # BC-Data (3 x 7Bits = 21Bit)
    #     BCB 7bit;
    #     BCG 7bit;
    #     BCR 7bit;
    # GS-Data (12 x 16Bits = 192Bit)
    #     GSB3 16bit;
    #     GSG3 16bit;
    #     GSR3 16bit;
    #     GSB2 16bit;
    #     GSG2 16bit;
    #     GSR2 16bit;
    #     GSB1 16bit;
    #     GSG1 16bit;
    #     GSR1 16bit;
    #     GSB0 16bit;
    #     GSG0 16bit;
    #     GSR0 16bit;
    # Device Nth-1 (244Bit = 28Byte)
    #     Device ..
    #     Device 2
    #     Device 1
    #     short break of 8x period of clock (666ns .. 2.74ms)
    #     to generate latchpulse
    #     + 1.34uS
    #     than next update.
    # """

    ##########################################
    # helper
    ##########################################

    # pylama:ignore=E0602
    # ugly workaround for pylama not knowing of micropython const thing
    _CHIP_BUFFER_BYTE_COUNT = const(28)

    COLORS_PER_PIXEL = const(3)
    PIXEL_PER_CHIP = const(4)
    CHANNEL_PER_CHIP = const(COLORS_PER_PIXEL * PIXEL_PER_CHIP)

    _BUFFER_BYTES_PER_COLOR = const(2)
    _BUFFER_BYTES_PER_PIXEL = const(_BUFFER_BYTES_PER_COLOR * COLORS_PER_PIXEL)

    # @staticmethod
    # def set_bit_with_mask(v, mask, x):
    #     """Set bit with help of mask."""
    #     # clear
    #     v &= ~mask
    #     if x:
    #         # set
    #         v |= mask
    #     return v
    #
    # @staticmethod
    # def set_bit(v, index, x):
    #     """Set bit - return new value.
    #
    #     Set the index:th bit of v to 1 if x is truthy,
    #     else to 0, and return the new value.
    #     https://stackoverflow.com/a/12174051/574981
    #     """
    #     # Compute mask, an integer with just bit 'index' set.
    #     mask = 1 << index
    #     # Clear the bit indicated by the mask (if x is False)
    #     v &= ~mask
    #     if x:
    #         # If x was True, set the bit indicated by the mask.
    #         v |= mask
    #     # Return the result, we're done.
    #     return v

    ##########################################
    # class _BC():
    # BC-Data (3 x 7Bits = 21Bit).
    #
    # BCB 7bit;
    # BCG 7bit;
    # BCR 7bit;
    _BC_CHIP_BUFFER_BIT_OFFSET = const(0)
    _BC_BIT_COUNT = const(3 * 7)
    # this holds the chip offset and
    _BC_FIELDS = {
        "BCR": {
            "offset": 0,
            "length": 7,
            "mask": 0b01111111,
        },
        "BCG": {
            "offset": 7,
            "length": 7,
            "mask": 0b01111111,
        },
        "BCB": {
            "offset": 14,
            "length": 7,
            "mask": 0b01111111,
        },
    }

    ##########################################
    # class _FC():
    # """
    # Function Control Data (5 x 1Bit = 5Bit).
    #
    # OUTTMG 1bit
    #     GS clock edge select
    #     1 = rising edge
    #     0 = falling edge
    # EXTGCK 1bit
    #     GS reference clock select
    #     1 = SCKI clock
    #     0 = internal oscillator
    # TMGRST 1bit
    #     display timing reset mode
    #     1 = OUT forced of on latchpulse
    #     0 = no forced reset
    # DSPRPT 1bit
    #     display repeat mode
    #     1 = auto repeate
    #     0 = Out only turned on after Blank or internal latchpulse
    # BLANK 1bit;
    #     ic power on sets this to 1
    #     1 = blank (outputs off)
    #     0 = Out on - controlled by GS-Data
    # """

    _FC_CHIP_BUFFER_BIT_OFFSET = const(_BC_BIT_COUNT)
    _FC_BIT_COUNT = const(5)
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
    # """WRITE_COMMAND."""

    _WC_CHIP_BUFFER_BIT_OFFSET = const(_FC_BIT_COUNT + _BC_BIT_COUNT)
    _WC_BIT_COUNT = const(6)
    _WC_FIELDS = {
        "WRITE_COMMAND": {
            "offset": 0,
            "length": 6,
            "mask": 0b111111,
        },
    }
    WRITE_COMMAND = const(0b100101)
    ##########################################

    ########
    _CHIP_BUFFER_HEADER_BIT_COUNT = const(
        _WC_BIT_COUNT + _FC_BIT_COUNT + _BC_BIT_COUNT)
    _CHIP_BUFFER_HEADER_BYTE_COUNT = const(_CHIP_BUFFER_HEADER_BIT_COUNT // 8)

    ##########################################

    def __init__(self, spi, pixel_count=1):
        """Init."""
        self._spi = spi
        # how many pixels are there?
        self.pixel_count = pixel_count
        self.channel_count = self.pixel_count * self.COLORS_PER_PIXEL
        # calculate how many chips are connected
        self.chip_count = self.pixel_count // 4

        # The chips are just a big 28 byte long shift register without any
        # fancy update protocol.  Blast out all the bits to update, that's it!
        # create raw output data
        self._buffer = bytearray(_CHIP_BUFFER_BYTE_COUNT * self.chip_count)

        # Initialize the brightness channel values to max
        # (these are 7-bit values).
        self.bcr = 127
        self.bcg = 127
        self.bcb = 127

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

        # self._buf32_format = struct.Struct('>I')
        # self._buf16_format = struct.Struct('>H')

        # preparation done
        # now initialize buffer to default values
        self._init_buffer()

        self._buffer_index_lookuptable = []
        self._init_lookuptable()

    def _init_buffer(self):
        for chip_index in range(self.chip_count):
            # set Write Command (6Bit) WRCMD (fixed: 25h)
            self.chip_set_BCData(
                chip_index, bcr=self.bcr, bcg=self.bcg, bcb=self.bcb)
            self._chip_set_FunctionControl(chip_index)
            self._chip_set_WriteCommand(chip_index)
        # loop end

    def set_chipheader_bits_in_buffer(
            self,
            *,
            chip_index=0,
            part_bit_offset=0,
            field={"mask": 0, "length": 0, "offset": 0},
            value=0
    ):
        """Set chip header bits in buffer."""
        offset = part_bit_offset + field["offset"]
        # restrict value
        value &= field["mask"]
        # move value to position
        value = value << offset
        # calculate header start
        header_start = chip_index * _CHIP_BUFFER_BYTE_COUNT
        # get chip header
        header = struct.unpack_from('>I', self._buffer, header_start)[0]
        # 0xFFFFFFFF == 0b11111111111111111111111111111111
        # create/move mask
        mask = field["mask"] << offset
        # clear
        header &= (~mask)
        # set
        header |= value
        # write header back
        struct.pack_into('>I', self._buffer, header_start, value)

    ##########################################

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
            part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET,
            field=self._BC_FIELDS["BCR"],
            value=bcr)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET,
            field=self._BC_FIELDS["BCG"],
            value=bcg)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET,
            field=self._BC_FIELDS["BCB"],
            value=bcb)

    def update_BCData(self):
        """
        Update BC-Data for all Chips in Buffer.

        need to be called after you changed on of the
        BC-Data Parameters. (bcr, bcg, bcb)
        """
        for chip_index in range(self.chip_count):
            self.chip_set_BCData(
                chip_index, bcr=self.bcr, bcg=self.bcg, bcb=self.bcb)

    def _chip_set_FunctionControl(self, chip_index):
        """
        Set Function Control Bits in Buffer.

        values from object global parameters are used.

        :param int chip_index: Index of Chip to set.
        """
        # set all bits
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["OUTTMG"],
            value=self.outtmg)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["EXTGCK"],
            value=self.extgck)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["TMGRST"],
            value=self.tmgrst)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=self._FC_FIELDS["DSPRPT"],
            value=self.dsprpt)
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
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
            part_bit_offset=_WC_CHIP_BUFFER_BIT_OFFSET,
            field=self._WC_FIELDS["WRITE_COMMAND"],
            value=self.WRITE_COMMAND)

    def _init_lookuptable(self):
        for channel_index in range(self.channel_count):
            buffer_index = (
                (_CHIP_BUFFER_BYTE_COUNT // _BUFFER_BYTES_PER_COLOR)
                * (channel_index // self.CHANNEL_PER_CHIP)
                + channel_index % self.CHANNEL_PER_CHIP)
            buffer_index *= _BUFFER_BYTES_PER_COLOR
            buffer_index += _CHIP_BUFFER_HEADER_BYTE_COUNT
            self._buffer_index_lookuptable.append(buffer_index)

    ##########################################

    def _write(self):
        # Write out the current state to the shift register.
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

    ##########################################

    @staticmethod
    def calculate_Ioclmax(*, Riref=2.48):
        """
        Calculate Maximum Constant Sink Current Value.

        see:
        8.4.1 Maximum Constant Sink Current Setting
        http://www.ti.com/lit/ds/symlink/tlc5971.pdf#page=18&zoom=160,0,524

        Riref = (Viref / Ioclmax) * 41
        Ioclmax = (41 / Riref) * Viref

        :param float Riref: resistor value (kΩ) (default=20)
        :return tuple: Ioclmax (mA)
        """
        # Riref                 = (Viref / Ioclmax) * 41    | / 41
        # Riref / 41            = Viref / Ioclmax           | switch
        # 41 / Riref            = Ioclmax / Viref           | * Viref
        # (41 / Riref) * Viref  = Ioclmax
        if not 0.8 <= Riref <= 24.8:
            raise ValueError(
                "Riref {} not in range: 0.8kΩ..25kΩ".format(Riref))
        Viref = 1.21
        Ioclmax = (41 / Riref) * Viref
        if not 2.0 <= Ioclmax <= 60.0:
            raise ValueError(
                "Ioclmax {} not in range: 2mA..60mA".format(Ioclmax))
        return Ioclmax

    @staticmethod
    def calculate_Riref(*, Ioclmax=20):
        """
        Calculate Maximum Constant Sink Current Value.

        see:
        8.4.1 Maximum Constant Sink Current Setting
        http://www.ti.com/lit/ds/symlink/tlc5971.pdf#page=19&zoom=200,0,697

        Riref = (Viref / Ioclmax) * 41

        :param float Ioclmax: target max output current (mA) (default=20)
        :return tuple: Riref (kΩ)
        """
        if not 2.0 <= Ioclmax <= 60.0:
            raise ValueError(
                "Ioclmax {} not in range: 2mA..60mA".format(Ioclmax))
        Viref = 1.21
        Riref = (Viref / Ioclmax) * 41
        if not 0.8 <= Riref <= 24.8:
            raise ValueError(
                "Riref {} not in range: 0.8kΩ..25kΩ".format(Riref))
        return Riref

    @staticmethod
    def calculate_BCData(*, Ioclmax=18, IoutR=17, IoutG=15, IoutB=9):
        """
        Calculate Global Brightness Control Values.

        see:
        8.5.1 Global Brightness Control (BC) Function (Sink Current Control)
        http://www.ti.com/lit/ds/symlink/tlc5971.pdf#page=19&zoom=200,0,697

        Iout = Ioclmax * (BCX / 127)
        BCX = Iout / Ioclmax * 127

        :param float Ioclmax: max output current set by Riref (mA) (default=20)
        :param float IoutR: max output current for red color group (mA)
            (default=9)
        :param float IoutG: max output current for green color (mA)
            (default=15)
        :param float IoutB: max output current for blue color (mA)
            (default=17)
        :return tuple: (bcr, bcg, bcb)
        """
        # Iout                 =  Ioclmax * (BCX / 127)  | / Ioclmax
        # Iout / Ioclmax       =  BCX / 127              | * 127
        # Iout / Ioclmax * 127 =  BCX
        if not 2.0 <= Ioclmax <= 60.0:
            raise ValueError("Ioclmax {} not in range: 2mA..60mA"
                             "".format(Ioclmax))
        if not 0.0 <= IoutR <= Ioclmax:
            raise ValueError(
                "IoutR {} not in range: 2mA..{}mA".format(IoutR, Ioclmax))
        if not 0.0 <= IoutG <= Ioclmax:
            raise ValueError(
                "IoutG {} not in range: 2mA..{}mA".format(IoutG, Ioclmax))
        if not 0.0 <= IoutB <= Ioclmax:
            raise ValueError(
                "IoutB {} not in range: 2mA..{}mA".format(IoutB, Ioclmax))
        bcr = int((IoutR / Ioclmax) * 127)
        bcg = int((IoutG / Ioclmax) * 127)
        bcb = int((IoutB / Ioclmax) * 127)
        if not 0 <= bcr <= 127:
            raise ValueError("bcr {} not in range: 0..127".format(bcr))
        if not 0 <= bcg <= 127:
            raise ValueError("bcg {} not in range: 0..127".format(bcg))
        if not 0 <= bcb <= 127:
            raise ValueError("bcb {} not in range: 0..127".format(bcb))
        return (bcr, bcg, bcb)

    ##########################################

    @staticmethod
    def _convert_01_float_to_16bit_integer(value):
        """Convert 0..1 Float Value to 16bit (0..65535) Range."""
        # check if value is in range
        if not 0.0 <= value[0] <= 1.0:
            raise ValueError("value[0] {} not in range: 0..1".format(value[0]))
        # convert to 16bit value
        return int(value * 65535)

    @classmethod
    def _convert_if_float(cls, value):
        """Convert if value is Float."""
        if isinstance(value, float):
            value = cls._convert_01_float_to_16bit_integer(value)
        return value

    @staticmethod
    def _check_and_convert(value):
        # loop
        # error_message = "values[{}] {} not in range: 0..{}"
        # for i, value in enumerate(values):
        #     # check if we have float values
        #     if isinstance(value, float):
        #         # check if value is in range
        #         if not 0.0 <= value <= 1.0:
        #             raise ValueError(error_message.format(i, value, '1'))
        #         # convert to 16bit value
        #         values[i] = int(value * 65535)
        #     else:
        #         if not 0 <= value <= 65535:
        #             raise ValueError(error_message.format(i, value, '65535'))
        # discreet
        # check if we have float values
        if isinstance(value[0], float):
            # check if value is in range
            if not 0.0 <= value[0] <= 1.0:
                raise ValueError(
                    "value[0] {} not in range: 0..1".format(value[0]))
            # convert to 16bit value
            value[0] = int(value[0] * 65535)
        else:
            if not 0 <= value[0] <= 65535:
                raise ValueError(
                    "value[0] {} not in range: 0..65535".format(value[0]))
        if isinstance(value[1], float):
            if not 0.0 <= value[1] <= 1.0:
                raise ValueError(
                    "value[1] {} not in range: 0..1".format(value[1]))
            value[1] = int(value[1] * 65535)
        else:
            if not 0 <= value[1] <= 65535:
                raise ValueError(
                    "value[1] {} not in range: 0..65535".format(value[1]))
        if isinstance(value[2], float):
            if not 0.0 <= value[2] <= 1.0:
                raise ValueError(
                    "value[2] {} not in range: 0..1".format(value[2]))
            value[2] = int(value[2] * 65535)
        else:
            if not 0 <= value[2] <= 65535:
                raise ValueError(
                    "value[2] {} not in range: 0..65535".format(value[2]))

    ##########################################

    def _get_channel_16bit_value(self, channel_index):
        buffer_start = self._buffer_index_lookuptable[channel_index]
        return struct.unpack_from('>H', self._buffer, buffer_start)[0]

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
        # self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        # self._buffer[buffer_start + 1] = value_b & 0xFF
        struct.pack_into('>H', self._buffer, buffer_start, value_b)
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        # self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        # self._buffer[buffer_start + 1] = value_g & 0xFF
        struct.pack_into('>H', self._buffer, buffer_start, value_g)
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        # self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        # self._buffer[buffer_start + 1] = value_r & 0xFF
        struct.pack_into('>H', self._buffer, buffer_start, value_r)

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
        self.set_pixel_16bit_value(
            pixel_index,
            int(value_r * 65535),
            int(value_g * 65535),
            int(value_b * 65535)
        )

    def set_pixel_16bit_color(self, pixel_index, color):
        """
        Set color for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.
        its a little bit slower as `set_pixel_16bit_value`

        :param int pixel_index: 0..(pixel_count)
        :param int color: 3-tuple of R, G, B;  0..65535
        """
        self.set_pixel_16bit_value(
            pixel_index,
            color[0],
            color[1],
            color[2]
        )

    def set_pixel_float_color(self, pixel_index, color):
        """
        Set color for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.
        its a little bit slower as `set_pixel_16bit_value`

        :param int pixel_index: 0..(pixel_count)
        :param tuple/float color: 3-tuple of R, G, B;  0..1
        """
        self.set_pixel_16bit_value(
            pixel_index,
            int(color[0] * 65535),
            int(color[1] * 65535),
            int(color[2] * 65535)
        )

    def set_pixel(self, pixel_index, value):
        """
        Set the R, G, B values for the pixel.

        this funciton hase some advanced error checking.
        it is much slower than the other provided 'bare' variants..
        but therefor gives clues to what is going wrong.. ;-)

        :param int pixel_index: 0..(pixel_count)
        :param tuple value: 3-tuple of R, G, B;
            each int 0..65535 or float 0..1
        """
        if 0 <= pixel_index < self.pixel_count:
            # convert to list
            value = list(value)
            # repr(value)
            if len(value) != self.COLORS_PER_PIXEL:
                raise IndexError(
                    "length of value {} does not match COLORS_PER_PIXEL (= {})"
                    "".format(len(value), self.COLORS_PER_PIXEL)
                )
            # tested:
            # splitting up into variables to not need the list..
            # this is about 0.25ms slower..
            # value_r = value[0]
            # value_g = value[1]
            # value_b = value[2]

            # check if we have float values
            # this modifies 'value' in place..
            self._check_and_convert(value)

            # update buffer
            # we change channel order here:
            # buffer channel order is blue, green, red
            # pixel_start = pixel_index * self.COLORS_PER_PIXEL
            # self._set_channel_16bit_value(
            #     pixel_start + 0,
            #     value[2])
            # self._set_channel_16bit_value(
            #     pixel_start + 1,
            #     value[1])
            # self._set_channel_16bit_value(
            #     pixel_start + 2,
            #     value[0])
            # optimize2
            # buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
            # self._buffer[buffer_start + 0] = (value[2] >> 8) & 0xFF
            # self._buffer[buffer_start + 1] = value[2] & 0xFF
            # buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
            # self._buffer[buffer_start + 0] = (value[1] >> 8) & 0xFF
            # self._buffer[buffer_start + 1] = value[1] & 0xFF
            # buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
            # self._buffer[buffer_start + 0] = (value[0] >> 8) & 0xFF
            # self._buffer[buffer_start + 1] = value[0] & 0xFF
            # simpliefy code
            self.set_pixel_16bit_value(
                pixel_index, value[0], value[1], value[2])
        else:
            raise IndexError(
                "index {} out of range [0..{}]"
                "".format(pixel_index, self.pixel_count))

    def set_pixel_all_16bit_value(self, value_r, value_g, value_b):
        """
        Set the R, G, B values for all pixels.

        fast. without error checking.

        :param int value_r: 0..65535
        :param int value_g: 0..65535
        :param int value_b: 0..65535
        """
        for i in range(self.pixel_count):
            self.set_pixel_16bit_value(i, value_r, value_g, value_b)

    def set_pixel_all(self, color):
        """
        Set the R, G, B values for all pixels.

        :param tuple 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        for i in range(self.pixel_count):
            self.set_pixel(i, color)

    def set_all_black(self):
        """Set all pixels to black."""
        for i in range(self.pixel_count):
            self.set_pixel_16bit_value(i, 0, 0, 0)

    # channel access
    def set_channel(self, channel_index, value):
        """
        Set the value for the provided channel.

        :param int channel_index: 0..channel_count
        :param int value: 0..65535
        """
        if 0 <= channel_index < (self.channel_count):
            # check if values are in range
            if not 0 <= value <= 65535:
                raise ValueError("value {} not in range: 0..65535")
            # temp = channel_index
            # we change channel order here:
            # buffer channel order is blue, green, red
            pixel_index_offset = channel_index % self.COLORS_PER_PIXEL
            if pixel_index_offset == 0:
                channel_index += 2
            elif pixel_index_offset == 2:
                channel_index -= 2
            # set value in buffer
            buffer_start = self._buffer_index_lookuptable[channel_index]
            struct.pack_into('>H', self._buffer, buffer_start, value)
        else:
            raise IndexError(
                "channel_index {} out of range (0..{})"
                .format(channel_index, self.channel_count))

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
        # else:
        raise IndexError(
            "index {} out of range [0..{}]".format(key, self.pixel_count))

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
        # for a more detailed version with all the debugging code and
        # comments look at set_pixel
        if 0 <= key < self.pixel_count:
            value = list(value)
            if len(value) != self.COLORS_PER_PIXEL:
                raise IndexError(
                    "length of value {} does not match COLORS_PER_PIXEL (= {})"
                    "".format(len(value), self.COLORS_PER_PIXEL)
                )
            # _check_and_convert modifies value in place..
            self._check_and_convert(value)
            self.set_pixel_16bit_value(key, value[0], value[1], value[2])
        else:
            raise IndexError(
                "index {} out of range [0..{}]"
                "".format(key, self.pixel_count)
            )

##########################################
