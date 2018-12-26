
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
    for thsi see examples/fancy_multi.py

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


import ctypes


def _shift_in(target_byte, val):
    # Shift in a new bit value to the provided target byte.  The byte will be
    # shift one position left and a new bit inserted that's a 1 if val is true,
    # of a 0 if false.
    target_byte <<= 1
    if val:
        target_byte |= 0x01
    return target_byte


def _get_7bit_uint(buffer, start):
    """Return 7bit interpreted as unsigned integer."""
    # 0b1111111
    pass


class TLC59711:
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
    CHIP_BUFFER_FC_OFFSET = 6
    CHIP_BUFFER_BC_OFFSET = CHIP_BUFFER_FC_OFFSET + 5

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

    class _FC_bits(ctypes.LittleEndianStructure):
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

        _fields_ = [
            ("OUTTMG", ctypes.c_uint8, 1),  # asByte & 1
            ("EXTGCK", ctypes.c_uint8, 1),  # asByte & 2
            ("TMGRST", ctypes.c_uint8, 1),  # asByte & 4
            ("DSPRPT", ctypes.c_uint8, 1),  # asByte & 8
            ("BLANK", ctypes.c_uint8, 1),  # asByte & 16
        ]

    # pylama:ignore=E0602

    class _FC(ctypes.Union):
        _anonymous_ = ("bit",)
        _fields_ = [
            ("bit", _FC_bits),
            ("asByte", ctypes.c_uint8)
        ]

    class _BC_bits(ctypes.LittleEndianStructure):
        _fields_ = [
            ("BCB", ctypes.c_uint8, 7),  # asByte & 1
            ("BCG", ctypes.c_uint8, 7),  # asByte & 2
            ("BCR", ctypes.c_uint8, 7),  # asByte & 4
        ]

    class _BC(ctypes.Union):
        _anonymous_ = ("bit",)
        _fields_ = [
            ("bit", _BC_bits),  # pylint: disable=undefined-name
            ("asByte", ctypes.c_uint32)
        ]

    ##########################################

    def __init__(self, spi, pixel_count=1):
        """Init."""
        self._spi = spi
        # how many pixels are there?
        self.pixel_count = pixel_count
        # calculate how many chips are connected
        self.chip_count = self.pixel_count // 4

        # This device is just a big 28 byte long shift register without any
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
        self.extgclk = False
        self.tmgrst = True
        self.dsprpt = True
        self.blank = False

        # preparation done
        # now initialize buffer to default values
        self._init_buffer()

    def _init_buffer(self):
        for chip_index in range(self.chip_count):
            # set Write Command (6Bit) WRCMD (fixed: 25h)
            buffer_start = chip_index * self.CHIP_BUFFER_LENGTH
            self._buffer[buffer_start] = 0x25

            self._chip_set_FunctionControl(chip_index)
            self.chip_set_BCData(
                chip_index, BCR=self._bcr, BCG=self._bcg, BCB=self._bcb)

    def _chip_set_FunctionControl(self, chip_index):
        """
        Set Function Control Bits in Buffer.

        values from object global parameters are used.

        :param int chip_index: Index of Chip to set.
        """
        buffer_start = (chip_index * self.CHIP_BUFFER_LENGTH) + \
            self.CHIP_BUFFER_FC_OFFSET
        fc = self._FC()
        fc.asByte = self._buffer[buffer_start]

    def update_fc(self):
        """
        Update Function Control Bits for all Chips in Buffer.

        need to be called after you changed on of the
        Function Control Bit Parameters.
        (outtmg, extgclk, tmgrst, dsprpt, blank)
        """
        for chip_index in range(self.chip_count):
            self._chip_set_FunctionControl(chip_index)

    def chip_set_BCData(self, chip_index, bcr=127, bcg=127, bcb=127):
        """
        Set BC-Data.

        :param int chip_index: Index of Chip to set.
        :param int bcr: 7-bit value from 0-127 (default=127)
        :param int bcg: 7-bit value from 0-127 (default=127)
        :param int bcb: 7-bit value from 0-127 (default=127)
        """
        buffer_start = (
            (chip_index * self.CHIP_BUFFER_LENGTH) + self.CHIP_BUFFER_BC_OFFSET
        )
        bc = self._BC()
        bc.asByte = self._buffer[buffer_start]

    def _write(self):
        # Write out the current state to the shift register.
        try:
            # Lock the SPI bus and configure it for the shift register.
            while not self._spi.try_lock():
                pass
            self._spi.configure(baudrate=10000000, polarity=0, phase=0)
            # Update the preamble of chip state in the first 4 bytes (32-bits)
            # with the write command, function control bits, and brightness
            # control register values.
            self._buffer[0] = 0x25  # 0x25 in top 6 bits initiates write.
            # Lower two bits control OUTTMG and EXTGCLK bits, set them
            # as appropriate.
            self._buffer[0] = _shift_in(self._buffer[0], self.outtmg)
            self._buffer[0] = _shift_in(self._buffer[0], self.extgclk)
            # Next byte contains remaining function control state and start of
            # brightness control bits.
            self._buffer[1] = 0x00
            self._buffer[1] = _shift_in(self._buffer[1], self.tmgrst)
            self._buffer[1] = _shift_in(self._buffer[1], self.dsprpt)
            self._buffer[1] = _shift_in(self._buffer[1], self.blank)
            # Top 5 bits from BC blue channel.
            self._buffer[1] <<= 5
            self._buffer[1] |= (self._bcb >> 2) & 0b11111
            # Next byte contains lower 2 bits from BC blue channel and upper 6
            # from BC green channel.
            self._buffer[2] = (self._bcb) & 0b11
            self._buffer[2] <<= 6
            self._buffer[2] |= (self._bcg >> 1) & 0b111111
            # Final byte contains lower 1 bit from BC green and 7 bits from BC
            # red channel.
            self._buffer[3] = self._bcg & 0b1
            self._buffer[3] <<= 7
            self._buffer[3] |= self._bcr & 0b1111111
            # The remaining bytes in the shift register are the channel PWM
            # values that have already been set by the user.
            # Now write out the the entire set of bytes.
            # Note there is no latch or other explicit line to tell the chip
            # when finished, it expects 28 bytes.
            self._spi.write(self._buffer)
        finally:
            # Ensure the SPI bus is unlocked.
            self._spi.unlock()

    def show(self):
        """Write out the current LED PWM state to the chip."""
        self._write()

    # Define properties for global brightness control channels.
    @property
    def red_brightness(self):
        """
        Red brightness for all channels on all chips.

        This is a 7-bit value from 0-127.
        """
        return self._bcr

    @red_brightness.setter
    def red_brightness(self, val):
        assert 0 <= val <= 127
        self._bcr = val
        if self.auto_show:
            self._write()

    @property
    def green_brightness(self):
        """
        Green brightness for all channels on all chips.

        This is a 7-bit value from 0-127.
        """
        return self._bcg

    @green_brightness.setter
    def green_brightness(self, val):
        assert 0 <= val <= 127
        self._bcg = val
        if self.auto_show:
            self._write()

    @property
    def blue_brightness(self):
        """
        Blue brightness for all channels on all chips.

        This is a 7-bit value from 0-127.
        """
        return self._bcb

    @blue_brightness.setter
    def blue_brightness(self, val):
        assert 0 <= val <= 127
        self._bcb = val
        if self.auto_show:
            self._write()

    # Define index and length properties to set and get each channel as
    # atomic RGB tuples.  This provides a similar feel as using neopixels.
    def __len__(self):
        """Retrieve the total number of Pixels available."""
        return self.pixel_count

    def __getitem__(self, key):
        # pylint: disable=no-else-return
        # Disable should be removed when refactor can be tested
        """
        Retrieve the R, G, B values for the provided channel as a 3-tuple.

        Each value is a 16-bit number from 0-65535.
        """
        if 0 < key > (self.pixel_count - 1):
            raw_data_start = 14 * (key / 12) + key % 12
            self._buffer[raw_data_start]
            return (self.r0, self.g0, self.b0)
        else:
            raise IndexError

    def __setitem__(self, key, val):
        """
        Set the R, G, B values for the provided channel.

        Specify a 3-tuple of R, G, B values that are each 16-bit numbers
        (0-65535).
        """
        # Do this check here instead of later to
        # prevent accidentally keeping auto_show
        # turned off when a bad key is provided.
        assert 0 <= key <= 3

        assert len(val) == 3
        assert 0 <= val[0] <= 65535
        assert 0 <= val[1] <= 65535
        assert 0 <= val[2] <= 65535
        # Temporarily halt auto write to perform an atomic update of all
        # the channel values.
        old_auto_show = self.auto_show
        self.auto_show = False
        # Update appropriate channel values.
        if key == 0:
            self.r0, self.g0, self.b0 = val
        elif key == 1:
            self.r1, self.g1, self.b1 = val
        elif key == 2:
            self.r2, self.g2, self.b2 = val
        elif key == 3:
            self.r3, self.g3, self.b3 = val
        # Restore auto_show state.
        self.auto_show = old_auto_show
        # Write out new values if in auto_show state.
        if self.auto_show:
            self._write()
