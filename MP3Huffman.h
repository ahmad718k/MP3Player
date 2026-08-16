#ifndef MP3_HUFFMAN_H
#define MP3_HUFFMAN_H

#include <stdint.h>
#include <stddef.h>
#include "MP3BitReader.h"

struct MP3HuffmanPair {
    int x;
    int y;
};

struct MP3HuffmanQuad {
    int v;
    int w;
    int x;
    int y;
};

/*
 * MPEG-1 Layer III Huffman decoder.
 *
 * The core is deliberately independent of Arduino/ESP32/FILE.  It uses
 * only MP3BitReader and fixed-size integer types so it can be moved to STM32.
 *
 * The implementation contains MPEG-1 Layer III Huffman tables 0..31 and
 * Count1 tables 32/33.  Tables 4 and 14 are the standard zero-bit aliases.
 */
namespace MP3Huffman {

bool decodePair(MP3BitReader &br, unsigned table, MP3HuffmanPair &out);
bool decodeQuad(MP3BitReader &br, unsigned table, MP3HuffmanQuad &out);

/* Decode one MPEG-1 granule's big_values and count1 data. */
bool decodeGranule(MP3BitReader &br,
                   unsigned part23Bits,
                   unsigned bigValues,
                   unsigned table0,
                   unsigned table1,
                   unsigned table2,
                   unsigned region1Start,
                   unsigned region2Start,
                   unsigned count1Table,
                   int16_t spectral[576],
                   unsigned &pairsDecoded,
                   unsigned &quadsDecoded,
                   unsigned &bitsUsed);

}

#endif
