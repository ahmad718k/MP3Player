STAGE17 - REAL MPEG-1 LAYER III DECODER UPDATE
===============================================

This revision replaces the previous experimental Huffman subset with a real
MPEG-1 Layer III Huffman decoder covering:

  - Huffman tables 0..31
  - Count1 tables 32 and 33
  - shared trees for tables 16..23 and 24..31
  - table-specific linbits
  - sign bits
  - big_values up to 288 pairs
  - region 0 / region 1 / region 2 selection
  - switched-block Huffman region handling

The Huffman tree representation is the compact negative-offset form used by
JUCE's MP3 decoder.  Table values are derived from the standard Layer III
Huffman tables.  Attribution/license information for the JUCE source is
retained here because the tree representation was used as the implementation
reference.

REAL BIT RESERVOIR
------------------

The decoder now treats main_data_begin correctly:

  virtual main-data = last main_data_begin bytes of reservoir
                    + current frame main-data

The reservoir is the final 511 bytes of the COMPLETE accumulated main-data
stream, not merely the current frame.  This is important when the current
frame contains fewer than 511 main-data bytes.

part2_3 decoding starts at bit zero of the virtual stream.  The reservoir
prefix is NOT skipped.

REGIONS
-------

For MPEG-1 long blocks, region boundaries are derived from the 44.1 kHz
Layer III scalefactor-band table currently used by this project.

For switched block_type=2, region 0 is 18 Huffman pairs (36 spectral lines)
and region 1 continues to the end of the big_values area.

CURRENT PROJECT SCOPE
---------------------

This Stage17 path is the MPEG-1 Layer III path used by the supplied project
and test MP3 (44.1 kHz stereo).  The existing parser/playback path remains
MPEG-1 oriented; MPEG-2/2.5 side-information and scalefactor decoding have
not been silently claimed as complete.

HOST VALIDATION
---------------

The new MP3Huffman.cpp was compiled independently with MP3BitReader.cpp.
A host test using the local 44.1 kHz stereo MP3 test file completed:

  frames scanned : 8
  Huffman frames : 8
  Huffman fails  : 0
  reservoir      : 511 bytes

Tables encountered in that test included 0, 3, 6, 12, 15, 24, 25, 26, 27.

The actual Stage17 decoder must still be validated against the user's
original MP3 file on the ESP32/target hardware, because the target project's
later synthesis/PWM stages are independent of the Huffman table test.
