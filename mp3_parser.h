#ifndef MP3_PARSER_H
#define MP3_PARSER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MP3_OK = 0,
    MP3_NEED_DATA = 1,
    MP3_INVALID = -1
} MP3Result;


typedef struct
{
    uint8_t version;
    uint8_t layer;

    uint8_t protection_absent;

    uint32_t bitrate;
    uint32_t sample_rate;

    uint8_t padding;
    uint8_t private_bit;

    uint8_t channel_mode;
    uint8_t mode_extension;

    uint8_t copyright;
    uint8_t original;
    uint8_t emphasis;

    uint16_t frame_size;

    uint16_t samples_per_frame;

    uint8_t channels;

} MP3FrameHeader;


/*
 * Parse one MPEG Audio Layer III frame header.
 *
 * buffer must contain at least 4 bytes.
 */
MP3Result mp3_parse_header(
    const uint8_t *buffer,
    MP3FrameHeader *header
);


/*
 * Find the first valid MP3 frame.
 *
 * Returns byte offset from buffer start.
 */
int32_t mp3_find_sync(
    const uint8_t *buffer,
    uint32_t size
);

typedef struct
{
    uint16_t part2_3_length;
    uint16_t big_values;

    uint8_t global_gain;
    uint8_t scalefac_compress;

    uint8_t window_switching_flag;

    uint8_t table_select[3];

    uint8_t region0_count;
    uint8_t region1_count;

    uint8_t preflag;
    uint8_t scalefac_scale;
    uint8_t count1table_select;

    uint8_t block_type;
    uint8_t mixed_block_flag;

    uint8_t subblock_gain[3];

} MP3Granule;


typedef struct
{
    uint16_t main_data_begin;

    uint8_t private_bits;

    uint8_t scfsi[2][4];

    MP3Granule granule[2][2];

} MP3SideInfo;

MP3Result mp3_parse_side_info(
    const uint8_t *buffer,
    uint8_t channels,
    MP3SideInfo *side_info
);
#ifdef __cplusplus
}
#endif

#endif