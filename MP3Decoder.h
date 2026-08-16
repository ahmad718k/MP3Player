#ifndef MP3_DECODER_H
#define MP3_DECODER_H

#include <stdio.h>
#include <stdint.h>

/*
 * Portable MPEG-1 Layer III decoder development interface.
 *
 * Current development stage: MPEG-1 Layer III frame analysis through
 * scalefactor, Huffman, requantization, reordering and the hybrid IMDCT, stereo and 32-subband synthesis test path.  The public test entry point accepts any FILE*; Storage.ino
 * enumerates all .mp3 files in /storage and calls it for every file.
 *
 * The implementation is still a development MPEG-1 Layer III decoder,
 * currently focused on the supplied 44.1 kHz stereo test path.
 *
 * The decoder core is kept separate from the FAT/MSC code where possible.
 * Storage.ino owns file discovery; this module owns MP3 frame decoding.
 */

#ifdef __cplusplus
extern "C" {
#endif

void mp3DecoderTest(FILE *file);

#ifdef __cplusplus
}
#endif

#endif
