# SPDX-FileCopyrightText: 2017 Tony DiCola for Adafruit Industries
# SPDX-FileCopyrightText: 2018 Stefan Krüger s-light.eu
#
# SPDX-License-Identifier: MIT

"""
`adafruit_tlc59711`
====================================================

CircuitPython module for the
TLC59711 or TLC5971 16-bit 12 channel LED PWM driver.
See examples/tlc59711_simpletest.py for a demo of the usage.

* Author(s): Tony DiCola, Stefan Kruger

Implementation Notes
--------------------

**Hardware:**

* Adafruit `12-Channel 16-bit PWM LED Driver - SPI Interface - TLC59711
  <https://www.adafruit.com/product/1455>`_ (Product ID: 1455)
  or TLC5971

**Software and Dependencies:**

* The API is mostly compatible to the DotStar / NeoPixel Libraries
    and is therefore also compatible with FancyLED.

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads
"""
__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TLC59711.git"


# pylint - globally disable
# 'invalid-name' check to allow for datasheet conform short channel and register names.
# pylint: disable=invalid-name
# 'too-many-lines' with the extra class and the api backwards compatibel things i have ~1100 lines..
# and as it was wished to not alter the .pylint file to limit to 1100 i disable it here..
# and yes - this code is very detailed commented. but i think that this is a good thing -
# as hopefully this way it is easier to understand desicions &
# what is going on and learn the backgrounds..
# pylint: disable=too-many-lines


import struct

from micropython import const

try:
    from typing import Dict, List, Optional, Tuple
    from busio import SPI
except ImportError:
    pass

_CHIP_BUFFER_BYTE_COUNT = const(28)

COLORS_PER_PIXEL = const(3)
PIXEL_PER_CHIP = const(4)
CHANNEL_PER_CHIP = const(COLORS_PER_PIXEL * PIXEL_PER_CHIP)

_BUFFER_BYTES_PER_COLOR = const(2)
_BUFFER_BYTES_PER_PIXEL = const(_BUFFER_BYTES_PER_COLOR * COLORS_PER_PIXEL)

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
_CHIP_BUFFER_HEADER_BIT_COUNT = const(_WC_BIT_COUNT + _FC_BIT_COUNT + _BC_BIT_COUNT)
_CHIP_BUFFER_HEADER_BYTE_COUNT = const(_CHIP_BUFFER_HEADER_BIT_COUNT // 8)


class TLC59711:
    """TLC5971 & TLC59711 16-bit 12 channel LED PWM driver.

    The TLC59711 & TLC5971 chip is designed to drive 4 RGB LEDs with 16-bit PWM per Color.
    This Library can control 1..many chips.
    The class has an interface compatible with the FancyLED library -
    and the API is similar to the NeoPixel and DotStar Interfaces.

    :param ~busio.SPI spi: An instance of the SPI bus connected to the chip.
        The clock and MOSI/output must be set, the MISO/input is unused.
        Maximal data clock frequency is:
        - TLC59711: 10MHz
        - TLC5971: 20MHz
    :param int pixel_count: Number of RGB-LEDs (=Pixels) that are connected. (default=4)
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

    def __init__(self, spi: SPI, *, pixel_count: int = 4) -> None:
        """Init."""
        self._spi = spi
        self.pixel_count = pixel_count
        self.channel_count = self.pixel_count * COLORS_PER_PIXEL
        # calculate how many chips are connected
        self.chip_count = self.pixel_count // 4

        # The chips are just a big 28 byte long shift register without any fancy update protocol.
        # Blast out all the bits to update, that's it! → create raw output data
        self._buffer = bytearray(_CHIP_BUFFER_BYTE_COUNT * self.chip_count)

        # Initialize the brightness channel values to max (these are 7-bit values).
        self.bcr = 127
        self.bcg = 127
        self.bcb = 127

        # Initialize external user-facing state for the function control bits of the chip.
        # These aren't commonly used but available and match the nomenclature from the datasheet.
        # you must manually call update_fc() after changing them
        # (reduces the need to make frivolous memory-hogging properties).
        # Default set: OUTTMG, TMGRST, and DSPRPT to on; like in Arduino library.
        # these are set for all chips the same
        self.outtmg = True
        self.extgck = False
        self.tmgrst = True
        self.dsprpt = True
        self.blank = False

        # preparation done → now initialize buffer to default values
        self._init_buffer()
        self._buffer_index_lookuptable = []
        self._init_lookuptable()

    def _init_buffer(self) -> None:
        for chip_index in range(self.chip_count):
            # set Write Command (6Bit) WRCMD (fixed: 25h)
            self.chip_set_BCData(chip_index, bcr=self.bcr, bcg=self.bcg, bcb=self.bcb)
            self._chip_set_FunctionControl(chip_index)
            self._chip_set_WriteCommand(chip_index)

    def set_chipheader_bits_in_buffer(
        self,
        *,
        chip_index: int = 0,
        part_bit_offset: int = 0,
        field: Optional[Dict[str, int]] = None,
        value: int = 0,
    ) -> None:
        """Set chip header bits in buffer."""
        if field is None:
            field = {"mask": 0, "length": 0, "offset": 0}
        offset = part_bit_offset + field["offset"]
        # restrict value
        value &= field["mask"]
        # move value to position
        value = value << offset
        # calculate header start
        header_start = chip_index * _CHIP_BUFFER_BYTE_COUNT
        # get chip header
        header = struct.unpack_from(">I", self._buffer, header_start)[0]
        # 0xFFFFFFFF == 0b11111111111111111111111111111111
        # create/move mask
        mask = field["mask"] << offset
        # clear
        header &= ~mask
        # set
        header |= value
        # write header back
        struct.pack_into(">I", self._buffer, header_start, header)

    ##########################################

    def chip_set_BCData(
        self, chip_index: int, bcr: int = 127, bcg: int = 127, bcb: int = 127
    ) -> None:
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
            field=_BC_FIELDS["BCR"],
            value=bcr,
        )
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET,
            field=_BC_FIELDS["BCG"],
            value=bcg,
        )
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_BC_CHIP_BUFFER_BIT_OFFSET,
            field=_BC_FIELDS["BCB"],
            value=bcb,
        )

    def update_BCData(self) -> None:
        """
        Update BC-Data for all Chips in Buffer.

        need to be called after you changed on of the BC-Data Parameters. (bcr, bcg, bcb)
        """
        for chip_index in range(self.chip_count):
            self.chip_set_BCData(chip_index, bcr=self.bcr, bcg=self.bcg, bcb=self.bcb)

    def _chip_set_FunctionControl(self, chip_index: int) -> None:
        """
        Set Function Control Bits in Buffer.

        values from object global parameters are used.

        :param int chip_index: Index of Chip to set.
        """
        # set all bits
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=_FC_FIELDS["OUTTMG"],
            value=self.outtmg,
        )
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=_FC_FIELDS["EXTGCK"],
            value=self.extgck,
        )
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=_FC_FIELDS["TMGRST"],
            value=self.tmgrst,
        )
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=_FC_FIELDS["DSPRPT"],
            value=self.dsprpt,
        )
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_FC_CHIP_BUFFER_BIT_OFFSET,
            field=_FC_FIELDS["BLANK"],
            value=self.blank,
        )

    def update_fc(self) -> None:
        """
        Update Function Control Bits for all Chips in Buffer.

        need to be called after you changed on of the Function Control Bit Parameters.
        (outtmg, extgck, tmgrst, dsprpt, blank)
        """
        for chip_index in range(self.chip_count):
            self._chip_set_FunctionControl(chip_index)

    def _chip_set_WriteCommand(self, chip_index: int) -> None:
        """Set WRITE_COMMAND."""
        # set all bits
        self.set_chipheader_bits_in_buffer(
            chip_index=chip_index,
            part_bit_offset=_WC_CHIP_BUFFER_BIT_OFFSET,
            field=_WC_FIELDS["WRITE_COMMAND"],
            value=WRITE_COMMAND,
        )

    def _init_lookuptable(self) -> None:
        for channel_index in range(self.channel_count):
            buffer_index = (_CHIP_BUFFER_BYTE_COUNT // _BUFFER_BYTES_PER_COLOR) * (
                channel_index // CHANNEL_PER_CHIP
            ) + channel_index % CHANNEL_PER_CHIP
            buffer_index *= _BUFFER_BYTES_PER_COLOR
            buffer_index += _CHIP_BUFFER_HEADER_BYTE_COUNT
            self._buffer_index_lookuptable.append(buffer_index)

    ##########################################

    def _write(self) -> None:
        # Write out the current state to the shift register.
        try:
            # Lock the SPI bus and configure it for the shift register.
            while not self._spi.try_lock():
                pass
            self._spi.configure(baudrate=self._spi.frequency, polarity=0, phase=0)
            self._spi.write(self._buffer)
        finally:
            # Ensure the SPI bus is unlocked.
            self._spi.unlock()

    def show(self) -> None:
        """Write out the current LED PWM state to the chip."""
        self._write()

    ##########################################

    @staticmethod
    def calculate_Ioclmax(*, Riref: float = 2.48) -> float:
        """
        Calculate Maximum Constant Sink Current Value.

        see:
        8.4.1 Maximum Constant Sink Current Setting
        http://www.ti.com/lit/ds/symlink/tlc5971.pdf#page=18&zoom=160,0,524

        Riref = (Viref / Ioclmax) * 41
        Ioclmax = (41 / Riref) * Viref

        :param float Riref: resistor value (kΩ) (default=20)
        :return float: Ioclmax (mA)
        """
        # Riref                 = (Viref / Ioclmax) * 41    | / 41
        # Riref / 41            = Viref / Ioclmax           | switch
        # 41 / Riref            = Ioclmax / Viref           | * Viref
        # (41 / Riref) * Viref  = Ioclmax
        if not 0.8 <= Riref <= 24.8:
            raise ValueError(f"Riref {Riref} not in range: 0.8kΩ..25kΩ")
        Viref = 1.21
        Ioclmax = (41 / Riref) * Viref
        if not 2.0 <= Ioclmax <= 60.0:
            raise ValueError(f"Ioclmax {Ioclmax} not in range: 2mA..60mA")
        return Ioclmax

    @staticmethod
    def calculate_Riref(*, Ioclmax: float = 20) -> float:
        """
        Calculate Maximum Constant Sink Current Value.

        see:
        8.4.1 Maximum Constant Sink Current Setting
        http://www.ti.com/lit/ds/symlink/tlc5971.pdf#page=19&zoom=200,0,697

        Riref = (Viref / Ioclmax) * 41

        :param float Ioclmax: target max output current (mA) (default=20)
        :return float: Riref (kΩ)
        """
        if not 2.0 <= Ioclmax <= 60.0:
            raise ValueError(f"Ioclmax {Ioclmax} not in range: 2mA..60mA")
        Viref = 1.21
        Riref = (Viref / Ioclmax) * 41
        if not 0.8 <= Riref <= 24.8:
            raise ValueError(f"Riref {Riref} not in range: 0.8kΩ..25kΩ")
        return Riref

    @staticmethod
    def calculate_BCData(
        *, Ioclmax: float = 18, IoutR: float = 17, IoutG: float = 15, IoutB: float = 9
    ) -> Tuple[float, float, float]:
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
            raise ValueError(f"Ioclmax {Ioclmax} not in range: 2mA..60mA")
        if not 0.0 <= IoutR <= Ioclmax:
            raise ValueError(f"IoutR {IoutR} not in range: 2mA..{Ioclmax}mA")
        if not 0.0 <= IoutG <= Ioclmax:
            raise ValueError(f"IoutG {IoutG} not in range: 2mA..{Ioclmax}mA")
        if not 0.0 <= IoutB <= Ioclmax:
            raise ValueError(f"IoutB {IoutB} not in range: 2mA..{Ioclmax}mA")
        bcr = int((IoutR / Ioclmax) * 127)
        bcg = int((IoutG / Ioclmax) * 127)
        bcb = int((IoutB / Ioclmax) * 127)
        if not 0 <= bcr <= 127:
            raise ValueError(f"bcr {bcr} not in range: 0..127")
        if not 0 <= bcg <= 127:
            raise ValueError(f"bcg {bcg} not in range: 0..127")
        if not 0 <= bcb <= 127:
            raise ValueError(f"bcb {bcb} not in range: 0..127")
        return (bcr, bcg, bcb)

    ##########################################

    @staticmethod
    def _convert_01_float_to_16bit_integer(value: float) -> int:
        """Convert 0..1 Float Value to 16bit (0..65535) Range."""
        # check if value is in range
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"value[0] {value} not in range: 0..1")
        # convert to 16bit value
        return int(value * 65535)

    @classmethod
    def _convert_if_float(cls, value: float) -> int:
        """Convert if value is Float."""
        if isinstance(value, float):
            value = cls._convert_01_float_to_16bit_integer(value)
        return value

    @staticmethod
    def _check_and_convert(value: List[int]):
        # loop
        # mega slow
        # for i, val in enumerate(value):
        #     # check if we have float values
        #     if isinstance(val, float):
        #         # check if val is in range
        #         if not 0.0 <= val <= 1.0:
        #             raise ValueError(
        #                 "value[{}] {} not in range: 0..1".format(i, val))
        #         # convert to 16bit val
        #         value[i] = int(val * 65535)
        #     else:
        #         if not 0 <= val <= 65535:
        #             raise ValueError(
        #                 "value[{}] {} not in range: 0..65535".format(i, val))
        # discreet is way faster
        # (compared with tlc59711_multi_dev.py: pixels.set_all_black())
        # check if we have float values
        if isinstance(value[0], float):
            # check if value is in range
            if not 0.0 <= value[0] <= 1.0:
                raise ValueError(f"value[0] {value[0]} not in range: 0..1")
            # convert to 16bit value
            value[0] = int(value[0] * 65535)
        else:
            if not 0 <= value[0] <= 65535:
                raise ValueError(f"value[0] {value[0]} not in range: 0..65535")
        if isinstance(value[1], float):
            if not 0.0 <= value[1] <= 1.0:
                raise ValueError(f"value[1] {value[1]} not in range: 0..1")
            value[1] = int(value[1] * 65535)
        else:
            if not 0 <= value[1] <= 65535:
                raise ValueError(f"value[1] {value[1]} not in range: 0..65535")
        if isinstance(value[2], float):
            if not 0.0 <= value[2] <= 1.0:
                raise ValueError(f"value[2] {value[2]} not in range: 0..1")
            value[2] = int(value[2] * 65535)
        else:
            if not 0 <= value[2] <= 65535:
                raise ValueError(f"value[2] {value[2]} not in range: 0..65535")

    ##########################################

    def _get_channel_16bit_value(self, channel_index):
        buffer_start = self._buffer_index_lookuptable[channel_index]
        return struct.unpack_from(">H", self._buffer, buffer_start)[0]

    def set_pixel_16bit_value(
        self, pixel_index: int, value_r: int, value_g: int, value_b: int
    ) -> None:
        """
        Set the value for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.

        :param int pixel_index: 0..(pixel_count)
        :param int value_r: 0..65535
        :param int value_g: 0..65535
        :param int value_b: 0..65535
        """
        # optimized for speed.
        # the struct version leads to very slow runtime behaivor if you set
        # lots of pixels. that is the reason the discreet version is used.
        # you can check with the tlc59711_multi_dev.py file.
        # most prominent this is visible at the set_pixel_all_16bit_value func:
        #  struct 157ms to 16ms (@144pixel on ItsyBitsy M4)
        pixel_start = pixel_index * COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        # struct.pack_into('>H', self._buffer, buffer_start, value_b)
        self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_b & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        # struct.pack_into('>H', self._buffer, buffer_start, value_g)
        self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_g & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        # struct.pack_into('>H', self._buffer, buffer_start, value_r)
        self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_r & 0xFF

    def set_pixel_float_value(
        self, pixel_index: int, value_r: int, value_g: int, value_b: int
    ) -> None:
        """
        Set the value for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.

        :param int pixel_index: 0..(pixel_count)
        :param int value_r: 0..1
        :param int value_g: 0..1
        :param int value_b: 0..1
        """
        # self.set_pixel_16bit_value(
        #     pixel_index,
        #     int(value_r * 65535),
        #     int(value_g * 65535),
        #     int(value_b * 65535)
        # )
        #  this is again a speed optimized version:
        #  this cuts down from 228ms to 22ms (@144pixel on ItsyBitsy M4)
        value_r = int(value_r * 65535)
        value_g = int(value_g * 65535)
        value_b = int(value_b * 65535)
        pixel_start = pixel_index * COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (value_b >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_b & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (value_g >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_g & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (value_r >> 8) & 0xFF
        self._buffer[buffer_start + 1] = value_r & 0xFF

    def set_pixel_16bit_color(
        self, pixel_index: int, color: Tuple[int, int, int]
    ) -> None:
        """
        Set color for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.
        its a little bit slower as `set_pixel_16bit_value`

        :param int pixel_index: 0..(pixel_count)
        :param int color: 3-tuple of R, G, B;  0..65535
        """
        # self.set_pixel_16bit_value(
        #     pixel_index,
        #     color[0],
        #     color[1],
        #     color[2]
        # )
        # speed optimization: 140ms to 24ms (@144pixel on ItsyBitsy M4)
        # the `color = list(color)` is the key here! (don't ask me why..)
        color = list(color)
        pixel_start = pixel_index * COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (color[2] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = color[2] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (color[1] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = color[1] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (color[0] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = color[0] & 0xFF

    def set_pixel_float_color(
        self, pixel_index: int, color: Tuple[float, float, float]
    ) -> None:
        """
        Set color for pixel.

        This is a Fast UNPROTECTED function:
        no error / range checking is done.
        it's a little bit slower as `set_pixel_16bit_value`

        :param int pixel_index: 0..(pixel_count)
        :param tuple/float color: 3-tuple of R, G, B;  0..1
        """
        # self.set_pixel_16bit_value(
        #     pixel_index,
        #     int(color[0] * 65535),
        #     int(color[1] * 65535),
        #     int(color[2] * 65535)
        # )
        # speed optimization: 140ms to 30ms (@144pixel on ItsyBitsy M4)
        color = list(color)
        color[0] = int(color[0] * 65535)
        color[1] = int(color[1] * 65535)
        color[2] = int(color[2] * 65535)
        pixel_start = pixel_index * COLORS_PER_PIXEL
        buffer_start = self._buffer_index_lookuptable[pixel_start + 0]
        self._buffer[buffer_start + 0] = (color[2] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = color[2] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 1]
        self._buffer[buffer_start + 0] = (color[1] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = color[1] & 0xFF
        buffer_start = self._buffer_index_lookuptable[pixel_start + 2]
        self._buffer[buffer_start + 0] = (color[0] >> 8) & 0xFF
        self._buffer[buffer_start + 1] = color[0] & 0xFF

    def set_pixel(
        self,
        pixel_index: int,
        value: Tuple[float, float, float],
    ) -> None:
        """
        Set the R, G, B values for the pixel.

        this function hase some advanced error checking.
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
            if len(value) != COLORS_PER_PIXEL:
                raise IndexError(
                    f"length of value {len(value)} does not match COLORS_PER_PIXEL (= {COLORS_PER_PIXEL})"
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
            # pixel_start = pixel_index * COLORS_PER_PIXEL
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
            self.set_pixel_16bit_value(pixel_index, value[0], value[1], value[2])
        else:
            raise IndexError(
                f"index {pixel_index} out of range [0..{self.pixel_count}]"
            )

    def set_pixel_all_16bit_value(
        self, value_r: int, value_g: int, value_b: int
    ) -> None:
        """
        Set the R, G, B values for all pixels.

        fast. without error checking.

        :param int value_r: 0..65535
        :param int value_g: 0..65535
        :param int value_b: 0..65535
        """
        for i in range(self.pixel_count):
            self.set_pixel_16bit_value(i, value_r, value_g, value_b)

    def set_pixel_all(self, color: Tuple[float, float, float]) -> None:
        """
        Set the R, G, B values for all pixels.

        :param tuple color: 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        for i in range(self.pixel_count):
            self.set_pixel(i, color)

    def set_all_black(self) -> None:
        """Set all pixels to black."""
        for i in range(self.pixel_count):
            self.set_pixel_16bit_value(i, 0, 0, 0)

    # channel access
    def set_channel(self, channel_index: int, value: int) -> None:
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
            pixel_index_offset = channel_index % COLORS_PER_PIXEL
            if pixel_index_offset == 0:
                channel_index += 2
            elif pixel_index_offset == 2:
                channel_index -= 2
            # set value in buffer
            buffer_start = self._buffer_index_lookuptable[channel_index]
            struct.pack_into(">H", self._buffer, buffer_start, value)
        else:
            raise IndexError(
                "channel_index {channel_index} out of range (0..{self.channel_count})"
            )

    # Define index and length properties to set and get each pixel as
    # atomic RGB tuples.  This provides a similar feel as using neopixels.
    def __len__(self) -> int:
        """Retrieve TLC5975 the total number of Pixels available."""
        return self.pixel_count

    def __getitem__(self, key: int) -> Tuple[int, int, int]:
        """
        Retrieve the R, G, B values for the provided channel as a 3-tuple.

        Each value is a 16-bit number from 0-65535.
        """
        if 0 <= key < self.pixel_count:
            pixel_start = key * COLORS_PER_PIXEL
            return (
                self._get_channel_16bit_value(pixel_start + 0),
                self._get_channel_16bit_value(pixel_start + 1),
                self._get_channel_16bit_value(pixel_start + 2),
            )
        # else:
        raise IndexError(f"index {key} out of range [0..{self.pixel_count}]")

    def __setitem__(self, key: int, value: Tuple[float, float, float]) -> None:
        """
        Set the R, G, B values for the pixel.

        this function has some advanced error checking.
        it is much slower than the other provided 'bare' variants..
        but therefor gives clues to what is going wrong.. ;-)
        this shortcut is identical to `set_pixel`

        :param int key: 0..(pixel_count)
        :param tuple value: 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        # for a more detailed version with all the debugging code and
        # comments look at set_pixel
        if 0 <= key < self.pixel_count:
            value = list(value)
            if len(value) != COLORS_PER_PIXEL:
                raise IndexError(
                    f"length of value {len(value)} does not match COLORS_PER_PIXEL (= {COLORS_PER_PIXEL})"
                )
            # _check_and_convert modifies value in place..
            self._check_and_convert(value)
            self.set_pixel_16bit_value(key, value[0], value[1], value[2])
        else:
            raise IndexError(f"index {key} out of range [0..{self.pixel_count}]")

    class _ChannelDirekt:
        # Internal decorator to simplify mapping.

        def __init__(self, channel) -> None:
            self._channel = channel

        def __get__(self, obj, obj_type):
            # Grab the 16-bit value at the offset for this channel.
            return obj._get_channel_16bit_value(self._channel)

        def __set__(self, obj, value) -> None:
            # Set the 16-bit value at the offset for this channel.
            assert 0 <= value <= 65535
            obj.set_channel(self._channel, value)
            # if obj.auto_show:
            #     obj._write()

    # Define explicit channels for first IC.
    # → this is only for api backwards compliance.
    # not recommended for new use - use *set_channel*
    b0 = _ChannelDirekt(0)
    g0 = _ChannelDirekt(1)
    r0 = _ChannelDirekt(2)
    b1 = _ChannelDirekt(3)
    g1 = _ChannelDirekt(4)
    r1 = _ChannelDirekt(5)
    b2 = _ChannelDirekt(6)
    g2 = _ChannelDirekt(7)
    r2 = _ChannelDirekt(8)
    b3 = _ChannelDirekt(9)
    g3 = _ChannelDirekt(10)
    r3 = _ChannelDirekt(11)


##########################################


class TLC59711AutoShow(TLC59711):
    """TLC59711 16-bit 12 channel LED PWM driver with Auto-Show.

    This chip is designed to drive 4 RGB LEDs with 16-bit PWM per Color.
    The class has an interface compatible with the FancyLED library.
    and with this is similar to the NeoPixel and DotStar Interfaces.

    this TLC59711AutoShow is a subclass of TLC59711 that adds automatically
    sending changed data to the chips.
    this creates very slows responses on big changes.
    It is mainly useful if you only have a very small number of pixels.

    :param ~busio.SPI spi: An instance of the SPI bus connected to the chip.
        The clock and MOSI/output must be set, the MISO/input is unused.
        Maximal data clock frequency is:
        - TLC59711: 10MHz
        - TLC5971: 20MHz
    :param int pixel_count: Number of RGB-LEDs (=Pixels) that are connected.
    """

    def __init__(self, spi: SPI, pixel_count: int = 4) -> None:
        """Init."""
        super().__init__(spi, pixel_count=pixel_count)

    ##########################################

    def set_pixel(self, pixel_index: int, value: Tuple[float, float, float]) -> None:
        """
        Set the R, G, B values for the pixel.

        this function hase some advanced error checking.
        it is much slower than the other provided 'bare' variants..
        but therefor gives clues to what is going wrong.. ;-)

        :param int pixel_index: 0..(pixel_count)
        :param tuple value: 3-tuple of R, G, B;
            each int 0..65535 or float 0..1
        """
        super().set_pixel(pixel_index, value)
        self._write()

    def set_pixel_all(self, color: Tuple[float, float, float]) -> None:
        """
        Set the R, G, B values for all pixels.

        :param tuple color: 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        super().set_pixel_all(color)
        self._write()

    def set_all_black(self) -> None:
        """Set all pixels to black."""
        super().set_all_black()
        self._write()

    # channel access
    def set_channel(self, channel_index: int, value: int) -> None:
        """
        Set the value for the provided channel.

        :param int channel_index: 0..channel_count
        :param int value: 0..65535
        """
        super().set_channel(channel_index, value)
        self._write()

    def __setitem__(self, key: int, value: Tuple[float, float, float]) -> None:
        """
        Set the R, G, B values for the pixel.

        this function hase some advanced error checking.
        it is much slower than the other provided 'bare' variants..
        but therefor gives clues to what is going wrong.. ;-)
        this shortcut is identical to `set_pixel`

        :param int key: 0..(pixel_count)
        :param tuple 3-tuple of R, G, B;  each int 0..65535 or float 0..1
        """
        super().__setitem__(key, value)
        self._write()

    class _ChannelDirektAutoShow:
        # Internal decorator to simplify mapping.

        def __init__(self, channel) -> None:
            self._channel = channel

        def __get__(self, obj, obj_type):
            # Grab the 16-bit value at the offset for this channel.
            return obj._get_channel_16bit_value(self._channel)

        def __set__(self, obj, value) -> None:
            # Set the 16-bit value at the offset for this channel.
            assert 0 <= value <= 65535
            obj.set_channel(self._channel, value)
            obj._write()

    # Define explicit channels for first IC.
    # → this is only for api backwards compliance.
    # not recommended for new use - use *set_channel*
    b0 = _ChannelDirektAutoShow(0)
    g0 = _ChannelDirektAutoShow(1)
    r0 = _ChannelDirektAutoShow(2)
    b1 = _ChannelDirektAutoShow(3)
    g1 = _ChannelDirektAutoShow(4)
    r1 = _ChannelDirektAutoShow(5)
    b2 = _ChannelDirektAutoShow(6)
    g2 = _ChannelDirektAutoShow(7)
    r2 = _ChannelDirektAutoShow(8)
    b3 = _ChannelDirektAutoShow(9)
    g3 = _ChannelDirektAutoShow(10)
    r3 = _ChannelDirektAutoShow(11)

