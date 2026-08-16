#include "mp3_parser.h"


/*
 * MPEG-1 Layer III
 *
 * index:
 *
 * 1 = 32
 * 2 = 40
 * 3 = 48
 * 4 = 56
 * 5 = 64
 * 6 = 80
 * 7 = 96
 * 8 = 112
 * 9 = 128
 * 10 = 160
 * 11 = 192
 * 12 = 224
 * 13 = 256
 * 14 = 320
 *
 * kbps
 */

static const uint16_t bitrate_mpeg1_l3[] =
{
    0,
    32,
    40,
    48,
    56,
    64,
    80,
    96,
    112,
    128,
    160,
    192,
    224,
    256,
    320,
    0
};


/*
 * MPEG-2 / MPEG-2.5 Layer III
 */

static const uint16_t bitrate_mpeg2_l3[] =
{
    0,
    8,
    16,
    24,
    32,
    40,
    48,
    56,
    64,
    80,
    96,
    112,
    128,
    144,
    160,
    0
};


/*
 * MPEG-1 sample rates
 */

static const uint32_t samplerate_mpeg1[] =
{
    44100,
    48000,
    32000
};


/*
 * MPEG-2
 */

static const uint32_t samplerate_mpeg2[] =
{
    22050,
    24000,
    16000
};


/*
 * MPEG-2.5
 */

static const uint32_t samplerate_mpeg25[] =
{
    11025,
    12000,
    8000
};


MP3Result mp3_parse_header(
    const uint8_t *b,
    MP3FrameHeader *h
)
{

    if (b == 0 || h == 0)
        return MP3_INVALID;


    /*
     * Sync:
     *
     * 11 bits must be 1
     */

    if (b[0] != 0xFF)
        return MP3_INVALID;


    if ((b[1] & 0xE0) != 0xE0)
        return MP3_INVALID;


    /*
     * Version
     *
     * 00 = MPEG 2.5
     * 01 = reserved
     * 10 = MPEG 2
     * 11 = MPEG 1
     */

    uint8_t version_bits =
        (b[1] >> 3) & 0x03;


    if (version_bits == 1)
        return MP3_INVALID;


    if (version_bits == 3)
        h->version = 1;

    else if (version_bits == 2)
        h->version = 2;

    else
        h->version = 25;


    /*
     * Layer
     *
     * 01 = Layer III
     */

    uint8_t layer_bits =
        (b[1] >> 1) & 0x03;


    if (layer_bits != 1)
        return MP3_INVALID;


    h->layer = 3;


    /*
     * CRC
     */

    h->protection_absent =
        b[1] & 0x01;


    /*
     * Bitrate index
     */

    uint8_t bitrate_index =
        (b[2] >> 4) & 0x0F;


    if (
        bitrate_index == 0 ||
        bitrate_index == 15
    )
    {
        return MP3_INVALID;
    }


    /*
     * Sample rate index
     */

    uint8_t sample_index =
        (b[2] >> 2) & 0x03;


    if (sample_index == 3)
        return MP3_INVALID;


    /*
     * Bitrate
     */

    if (h->version == 1)
    {
        h->bitrate =
            bitrate_mpeg1_l3[
                bitrate_index
            ] * 1000UL;
    }
    else
    {
        h->bitrate =
            bitrate_mpeg2_l3[
                bitrate_index
            ] * 1000UL;
    }


    /*
     * Sample rate
     */

    if (h->version == 1)
    {
        h->sample_rate =
            samplerate_mpeg1[
                sample_index
            ];
    }

    else if (h->version == 2)
    {
        h->sample_rate =
            samplerate_mpeg2[
                sample_index
            ];
    }

    else
    {
        h->sample_rate =
            samplerate_mpeg25[
                sample_index
            ];
    }


    /*
     * Padding
     */

    h->padding =
        (b[2] >> 1) & 0x01;


    /*
     * Private bit
     */

    h->private_bit =
        b[2] & 0x01;


    /*
     * Channel mode
     */

    h->channel_mode =
        (b[3] >> 6) & 0x03;


    /*
     * Mode extension
     */

    h->mode_extension =
        (b[3] >> 4) & 0x03;


    /*
     * Copyright
     */

    h->copyright =
        (b[3] >> 3) & 0x01;


    /*
     * Original
     */

    h->original =
        (b[3] >> 2) & 0x01;


    /*
     * Emphasis
     */

    h->emphasis =
        b[3] & 0x03;


    /*
     * Channels
     */

    if (h->channel_mode == 3)
        h->channels = 1;
    else
        h->channels = 2;


    /*
     * Samples per frame
     */

    if (h->version == 1)
    {
        h->samples_per_frame = 1152;

        /*
         * MPEG-1 Layer III:
         *
         * frame_size =
         * 144 * bitrate / sample_rate
         * + padding
         */

        h->frame_size =
            (uint16_t)(
                (144UL *
                 h->bitrate) /
                h->sample_rate
            );

    }
    else
    {
        h->samples_per_frame = 576;

        /*
         * MPEG-2 / 2.5 Layer III:
         */

        h->frame_size =
            (uint16_t)(
                (72UL *
                 h->bitrate) /
                h->sample_rate
            );
    }


    if (h->padding)
        h->frame_size++;


    /*
     * Minimum sanity check
     */

    if (h->frame_size < 4)
        return MP3_INVALID;


    return MP3_OK;
}


int32_t mp3_find_sync(
    const uint8_t *buffer,
    uint32_t size
)
{

    if (buffer == 0)
        return -1;


    if (size < 4)
        return -1;


    for (
        uint32_t i = 0;
        i <= size - 4;
        i++
    )
    {

        MP3FrameHeader h;


        if (
            mp3_parse_header(
                &buffer[i],
                &h
            ) == MP3_OK
        )
        {
            return (int32_t)i;
        }
    }


    return -1;
}





















typedef struct
{
    const uint8_t *data;
    uint32_t bit_pos;
} MP3BitReader;


static uint32_t mp3_read_bits(
    MP3BitReader *br,
    uint8_t count
)
{
    uint32_t value = 0;

    while (count--)
    {
        uint32_t byte_pos =
            br->bit_pos >> 3;

        uint8_t bit_pos =
            7 - (br->bit_pos & 7);

        value <<= 1;

        value |=
            (br->data[byte_pos] >> bit_pos) & 1;

        br->bit_pos++;
    }

    return value;
}

MP3Result mp3_parse_side_info(
    const uint8_t *buffer,
    uint8_t channels,
    MP3SideInfo *s
)
{
    if (buffer == 0 || s == 0)
        return MP3_INVALID;

    if (channels != 1 && channels != 2)
        return MP3_INVALID;

    MP3BitReader br;

    br.data = buffer;
    br.bit_pos = 0;


    /*
     * MPEG-1 Layer III
     *
     * main_data_begin = 9 bits
     */

    s->main_data_begin =
        mp3_read_bits(&br, 9);


    /*
     * private_bits
     *
     * mono   = 5 bits
     * stereo = 3 bits
     */

    s->private_bits =
        mp3_read_bits(
            &br,
            channels == 1 ? 5 : 3
        );


    /*
     * scfsi
     */

    for (uint8_t ch = 0; ch < channels; ch++)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            s->scfsi[ch][i] =
                mp3_read_bits(&br, 1);
        }
    }


    /*
     * Two granules
     */

    for (uint8_t gr = 0; gr < 2; gr++)
    {
        for (uint8_t ch = 0; ch < channels; ch++)
        {
            MP3Granule *g =
                &s->granule[gr][ch];


            g->part2_3_length =
                mp3_read_bits(&br, 12);


            g->big_values =
                mp3_read_bits(&br, 9);


            g->global_gain =
                mp3_read_bits(&br, 8);


            g->scalefac_compress =
                mp3_read_bits(&br, 4);


            g->window_switching_flag =
                mp3_read_bits(&br, 1);


            if (g->window_switching_flag)
            {
                g->block_type =
                    mp3_read_bits(&br, 2);

                g->mixed_block_flag =
                    mp3_read_bits(&br, 1);


                g->table_select[0] =
                    mp3_read_bits(&br, 5);

                g->table_select[1] =
                    mp3_read_bits(&br, 5);


                g->table_select[2] = 0;


                for (uint8_t i = 0; i < 3; i++)
                {
                    g->subblock_gain[i] =
                        mp3_read_bits(&br, 3);
                }


                /*
                 * region counts are not stored
                 * when window_switching_flag = 1
                 */
                g->region0_count = 0;
                g->region1_count = 0;
            }
            else
            {
                g->block_type = 0;
                g->mixed_block_flag = 0;

                g->table_select[0] =
                    mp3_read_bits(&br, 5);

                g->table_select[1] =
                    mp3_read_bits(&br, 5);

                g->table_select[2] =
                    mp3_read_bits(&br, 5);


                g->region0_count =
                    mp3_read_bits(&br, 4);

                g->region1_count =
                    mp3_read_bits(&br, 3);


                g->subblock_gain[0] = 0;
                g->subblock_gain[1] = 0;
                g->subblock_gain[2] = 0;
            }


            g->preflag =
                mp3_read_bits(&br, 1);

            g->scalefac_scale =
                mp3_read_bits(&br, 1);

            g->count1table_select =
                mp3_read_bits(&br, 1);
        }
    }


    return MP3_OK;
}
