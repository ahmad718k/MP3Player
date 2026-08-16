MP3 Decoder Stage 2 - MPEG-1 Layer III Scalefactor Test

Files:
  Storage.ino
  MP3Decoder.cpp
  MP3Decoder.h
  MP3BitReader.cpp
  MP3BitReader.h

This stage keeps the USB/MSC/FAT code unchanged.

Added:
  - MPEG-1 Layer III long-block scalefactor tables
  - scalefactor decoding for sfb 0..20
  - SCFSI group handling for granule 1
  - exact part2_3 boundary tracking
  - exact Huffman-region start and length reporting

Not yet implemented:
  - short/mixed block scalefactors
  - bit reservoir across frames
  - Huffman tables/decoding
  - requantization
  - IMDCT
  - polyphase synthesis

Expected next step:
  Huffman pair decoding (big_values), then count1 quadruples.
