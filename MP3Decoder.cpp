#include "MP3Decoder.h"
#include "MP3BitReader.h"
#include "MP3Huffman.h"
#include "MP3Synthesis.h"
#include "MP3PWMOutput.h"
#include "MP3PCMCapture.h"

#ifndef MP3_WAV_ONLY
#define MP3_WAV_ONLY 1
#endif

#include <Arduino.h>
#include "USB.h"

// USBSerial is created by Storage.ino. This declaration makes the object
// visible to this separate translation unit.
extern USBCDC USBSerial;
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * NOTE:
 * The decoder core below is deliberately written without File, SPIFFS,
 * FATFS, ESP-IDF, or USB calls.  Only mp3DecoderTest() uses FILE* for the
 * ESP32 test harness.  The functions marked "portable" can be moved to STM32.
 */

static uint8_t gbuf[4096];

/*
 * ESP32-S2 stack protection:
 * These buffers are too large to keep as automatic locals inside
 * decodeFirstAudioFrame(). Keeping them static moves them out of the task
 * stack and prevents the reset that occurred immediately after entering
 * the STAGE 1 function. They are only used by this single-threaded test
 * decoder, so static storage is safe here.
 */
static int16_t gSpectral[2][576];
static int16_t gReordered[576];
static float   gXR[2][576];
static float   gHybrid[2][576];
static float   gHybridOverlap[2][32][18];
static int16_t gPCM[2][1152];

struct SideGranule {
    uint16_t part2_3_length;
    uint16_t big_values;
    uint8_t  global_gain;
    uint8_t  scalefac_compress;
    uint8_t  window_switching_flag;
    uint8_t  block_type;
    uint8_t  mixed_block_flag;
    uint8_t  table_select[3];
    uint8_t  subblock_gain[3];
    uint8_t  region0_count;
    uint8_t  region1_count;
    uint8_t  preflag;
    uint8_t  scalefac_scale;
    uint8_t  count1table_select;
};

static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           p[3];
}

/* MPEG-1 Layer III side-information parser. */
static bool parseSideInfo(const uint8_t *p, unsigned channels,
                          uint16_t &main_data_begin,
                          uint8_t scfsi[2][4],
                          SideGranule gr[2][2],
                          unsigned &sideBits)
{
    if (channels < 1 || channels > 2) return false;

    unsigned sideBytes = (channels == 1) ? 17 : 32;
    MP3BitReader br(p, sideBytes);

    main_data_begin = (uint16_t)br.read(9);

    unsigned privateBits = (channels == 1) ? 5 : 3;
    br.read(privateBits);

    for (unsigned ch=0; ch<channels; ++ch)
        for (unsigned s=0;s<4;++s)
            scfsi[ch][s] = (uint8_t)br.read(1);

    for (unsigned grn=0; grn<2; ++grn) {
        for (unsigned ch=0; ch<channels; ++ch) {
            SideGranule &g = gr[grn][ch];

            g.part2_3_length = (uint16_t)br.read(12);
            g.big_values = (uint16_t)br.read(9);
            if (g.big_values > 288) g.big_values = 288;
            g.global_gain = (uint8_t)br.read(8);
            g.scalefac_compress = (uint8_t)br.read(4);
            g.window_switching_flag = (uint8_t)br.read(1);

            g.block_type = 0;
            g.mixed_block_flag = 0;
            g.table_select[0]=g.table_select[1]=g.table_select[2]=0;
            g.subblock_gain[0]=g.subblock_gain[1]=g.subblock_gain[2]=0;
            g.region0_count=0;
            g.region1_count=0;

            if (g.window_switching_flag) {
                g.block_type = (uint8_t)br.read(2);
                g.mixed_block_flag = (uint8_t)br.read(1);

                g.table_select[0] = (uint8_t)br.read(5);
                g.table_select[1] = (uint8_t)br.read(5);

                for (unsigned i=0;i<3;++i)
                    g.subblock_gain[i] = (uint8_t)br.read(3);

                /* MPEG-1 Layer III: region boundaries are implicit for
                   switched blocks. */
                if (g.block_type == 2 && g.mixed_block_flag == 0) {
                    g.region0_count = 8;
                    g.region1_count = 12;
                } else {
                    g.region0_count = 7;
                    g.region1_count = 13;
                }
            } else {
                g.table_select[0] = (uint8_t)br.read(5);
                g.table_select[1] = (uint8_t)br.read(5);
                g.table_select[2] = (uint8_t)br.read(5);
                g.region0_count = (uint8_t)br.read(4);
                g.region1_count = (uint8_t)br.read(3);
            }

            g.preflag = (uint8_t)br.read(1);
            g.scalefac_scale = (uint8_t)br.read(1);
            g.count1table_select = (uint8_t)br.read(1);
        }
    }

    sideBits = (unsigned)br.bitsRead();
    return br.bitsRead() <= sideBytes*8u;
}

/*
 * MPEG-1 scalefactor-compress lookup.
 *
 * The returned values are the number of bits used by each scalefactor
 * group.  This is the exact MPEG-1 Layer III mapping.
 */
static const uint8_t slen1[16] =
    {0,0,0,0,3,1,1,1,2,2,2,3,3,3,4,4};

static const uint8_t slen2[16] =
    {0,1,2,3,0,1,2,3,1,2,3,1,2,3,2,3};

/*
 * MPEG-1 Layer III long-block scalefactors.
 *
 * sfb  0..10 use slen1
 * sfb 11..20 use slen2
 *
 * For granule 1, SCFSI suppresses four groups:
 *   0..5, 6..10, 11..15, 16..20
 */
static unsigned readLongScalefactorsMpeg1(
    MP3BitReader &br, const SideGranule &g, const uint8_t scfsi[4],
    bool granule1, uint8_t sf[45])
{
    unsigned sl1=slen1[g.scalefac_compress&0x0F];
    unsigned sl2=slen2[g.scalefac_compress&0x0F];
    unsigned used=0;
    for(unsigned sfb=0;sfb<21;++sfb){
        unsigned group=(sfb<=5)?0:(sfb<=10)?1:(sfb<=15)?2:3;
        bool reuse=granule1 && scfsi[group];
        unsigned nbits=(sfb<=10)?sl1:sl2;
        if(reuse){ sf[sfb]=0; continue; }
        if(!br.canRead(nbits)) return used;
        sf[sfb]=nbits?(uint8_t)br.read(nbits):0;
        used+=nbits;
    }
    return used;
}

static unsigned readShortScalefactorsMpeg1(
    MP3BitReader &br, const SideGranule &g, uint8_t sf[45])
{
    unsigned sl1=slen1[g.scalefac_compress&0x0F];
    unsigned sl2=slen2[g.scalefac_compress&0x0F];
    unsigned used=0;
    for(unsigned sfb=0;sfb<12;++sfb){
        unsigned nbits=(sfb<=5)?sl1:sl2;
        for(unsigned win=0;win<3;++win){
            if(!br.canRead(nbits)) return used;
            unsigned idx=sfb*3+win;
            sf[idx]=nbits?(uint8_t)br.read(nbits):0;
            used+=nbits;
        }
    }
    return used;
}

static unsigned readMixedScalefactorsMpeg1(
    MP3BitReader &br, const SideGranule &g, uint8_t sf[45])
{
    unsigned sl1=slen1[g.scalefac_compress&0x0F];
    unsigned sl2=slen2[g.scalefac_compress&0x0F];
    unsigned used=0;
    for(unsigned sfb=0;sfb<8;++sfb){
        unsigned nbits=(sfb<=5)?sl1:sl2;
        if(!br.canRead(nbits)) return used;
        sf[sfb]=nbits?(uint8_t)br.read(nbits):0;
        used+=nbits;
    }
    unsigned out=8;
    for(unsigned sfb=3;sfb<12;++sfb){
        unsigned nbits=(sfb<=5)?sl1:sl2;
        for(unsigned win=0;win<3;++win){
            if(!br.canRead(nbits)) return used;
            sf[out++]=nbits?(uint8_t)br.read(nbits):0;
            used+=nbits;
        }
    }
    return used;
}

static unsigned readScalefactorsMpeg1(
    MP3BitReader &br, const SideGranule &g, const uint8_t scfsi[4],
    bool granule1, uint8_t sf[45], unsigned &valueCount)
{
    memset(sf, 0, 45);
    valueCount = 0;

    /* MPEG-1 Layer III:
     *
     * window_switching_flag = 0
     *     -> long block: scalefac_l[0..20]
     *
     * window_switching_flag = 1, block_type = 2
     *     -> short block:
     *        non-mixed: scalefac_s[0..11][window 0..2]
     *        mixed:     scalefac_l[0..7] followed by
     *                   scalefac_s[3..11][window 0..2]
     *
     * block_type = 1 or 3 are start/end blocks. Their scalefactors use
     * the long-block layout. mixed_block_flag is not used for these.
     * SCFSI applies only to granule 1 long-block scalefactors.
     */
    /*
     * MPEG-1 SCFSI is valid only for granule 1 normal LONG blocks.
     * For switched blocks (block_type 1/2/3), scalefactors are coded
     * explicitly for the current granule.  Reusing SCFSI here would
     * consume too few bits and shift the Huffman stream, which then
     * appears as random Huffman-table errors in later granules.
     */
    if (!g.window_switching_flag) {
        valueCount = 21;
        return readLongScalefactorsMpeg1(br, g, scfsi, granule1, sf);
    }

    if (g.block_type == 1 || g.block_type == 3) {
        valueCount = 21;
        static const uint8_t noScfsi[4] = {0,0,0,0};
        return readLongScalefactorsMpeg1(br, g, noScfsi, false, sf);
    }

    if (g.block_type == 2) {
        if (g.mixed_block_flag) {
            valueCount = 35;
            return readMixedScalefactorsMpeg1(br, g, sf);
        }
        valueCount = 36;
        return readShortScalefactorsMpeg1(br, g, sf);
    }

    return 0;
}



/* MPEG-1 Layer III scalefactor-band boundaries for 44.1 kHz. */
static const uint16_t sfbLong44[23] = {
    0,4,8,12,16,20,24,30,36,44,52,62,
    74,90,110,134,162,196,238,288,342,418,576
};

static const uint16_t sfbShort44[14] = {
    0,4,8,12,16,22,30,40,52,66,84,106,136,192
};

static const uint8_t pretabMpeg1[21] = {
    0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,3,3,3,3,2,0
};

/*
 * MPEG-1 Layer III scalefactor-band reorder tables for 44.1 kHz.
 *
 * IMPORTANT:
 * Huffman order for a short block is:
 *   sfb0: window0[all], window1[all], window2[all]
 *   sfb1: window0[all], window1[all], window2[all]
 *   ...
 *
 * The IMDCT/alias-reduction side uses frequency-line order:
 *   line0: window0, window1, window2
 *   line1: window0, window1, window2
 *   ...
 */
static void reorderShortMpeg1(const int16_t in[576], int16_t out[576])
{
    memset(out, 0, sizeof(int16_t) * 576);

    unsigned src = 0;

    for (unsigned sfb = 0; sfb < 12; ++sfb) {
        const unsigned start = sfbShort44[sfb];
        const unsigned end   = sfbShort44[sfb + 1];
        const unsigned width = end - start;

        for (unsigned win = 0; win < 3; ++win) {
            for (unsigned j = 0; j < width; ++j) {
                if (src >= 576)
                    return;

                const unsigned line = start + j;
                out[3 * line + win] = in[src++];
            }
        }
    }
}

/*
 * Mixed block reorder.
 *
 * MPEG-1 mixed short block:
 *   spectral lines 0..35   -> long-block order, unchanged
 *   spectral lines 36..575 -> short-block reorder
 *
 * At 44.1 kHz, short-block sfb 3 starts at line 12 in the short-band
 * table, which corresponds to line 36 after the three-window expansion.
 */
static void reorderMixedMpeg1(const int16_t in[576], int16_t out[576])
{
    memcpy(out, in, sizeof(int16_t) * 576);

    unsigned src = 36;

    for (unsigned sfb = 3; sfb < 12; ++sfb) {
        const unsigned start = sfbShort44[sfb];
        const unsigned end   = sfbShort44[sfb + 1];
        const unsigned width = end - start;

        for (unsigned win = 0; win < 3; ++win) {
            for (unsigned j = 0; j < width; ++j) {
                if (src >= 576)
                    return;

                const unsigned shortLine = (start - 12) + j;
                out[36 + 3 * shortLine + win] = in[src++];
            }
        }
    }
}

/*
 * Debug helper used by the current ESP32 test harness.
 * It prints selected source/destination positions so that we can verify
 * the reorder without dumping all 576 values.
 */
static void printReorderTest(const int16_t spectral[576],
                             const int16_t reordered[576],
                             const SideGranule &g)
{
    if (!(g.window_switching_flag && g.block_type == 2))
        return;

    USBSerial.println("\nREORDERING TEST");
    USBSerial.println("--------------------------------------");
    USBSerial.printf("block_type         : %u\\n", g.block_type);
    USBSerial.printf("mixed_block        : %u\\n", g.mixed_block_flag);

    USBSerial.println("First 24 Huffman-order values:");
    for (unsigned i = 0; i < 24; ++i) {
        USBSerial.printf("%4u:%6d%s", i, (int)spectral[i],
                         ((i & 7) == 7) ? "\\n" : "  ");
    }

    USBSerial.println("First 24 reordered frequency-line values:");
    for (unsigned i = 0; i < 24; ++i) {
        USBSerial.printf("%4u:%6d%s", i, (int)reordered[i],
                         ((i & 7) == 7) ? "\\n" : "  ");
    }

    /* For a pure short block, the first sfb is 4 lines wide.  Therefore
     * source[0..3] are window 0, source[4..7] window 1 and source[8..11]
     * window 2.  The reordered result must interleave those windows:
     *   out[0..2]   = in[0], in[4], in[8]
     *   out[3..5]   = in[1], in[5], in[9]
     *   ...
     */
    if (!g.mixed_block_flag) {
        bool ok = true;
        for (unsigned j = 0; j < 4; ++j) {
            if (reordered[3 * j + 0] != spectral[j]) ok = false;
            if (reordered[3 * j + 1] != spectral[4 + j]) ok = false;
            if (reordered[3 * j + 2] != spectral[8 + j]) ok = false;
        }
        USBSerial.printf("First-sfb reorder check: %s\\n", ok ? "OK" : "ERROR");
    }
}


/* ================================================================
 * Stage 8: stereo processing + hybrid IMDCT
 * ================================================================ */

static const float SQRT2_INV = 0.7071067811865475244f;

static void applyMSStereo(float left[576], float right[576])
{
    for (unsigned i = 0; i < 576; ++i) {
        const float m = left[i];
        const float s = right[i];
        left[i]  = (m + s) * SQRT2_INV;
        right[i] = (m - s) * SQRT2_INV;
    }
}

/* MPEG-1 Layer III alias-reduction constants. */
static const float cs[8] = {
    0.8574929257f, 0.8817419973f, 0.9496286491f, 0.9833145925f,
    0.9955178161f, 0.9991605582f, 0.9998991952f, 0.9999931551f
};
static const float ca[8] = {
   -0.5144957554f, -0.4717319686f, -0.3133774542f, -0.1819131996f,
   -0.0945741925f, -0.0409655829f, -0.0141985686f, -0.0036999747f
};

static void aliasReduceMpeg1(float xr[576], const SideGranule &g)
{
    if (g.window_switching_flag && g.block_type == 2 && !g.mixed_block_flag)
        return;

    unsigned maxSb = 32;
    if (g.window_switching_flag && g.block_type == 2 && g.mixed_block_flag)
        maxSb = 2;

    for (unsigned sb = 1; sb < maxSb; ++sb) {
        for (unsigned i = 0; i < 8; ++i) {
            unsigned x = sb * 18 - 1 - i;
            unsigned y = sb * 18 + i;
            float bu = xr[x];
            float bd = xr[y];
            xr[x] = bu * cs[i] - bd * ca[i];
            xr[y] = bd * cs[i] + bu * ca[i];
        }
    }
}

static void makeIMDCTWindow(unsigned blockType, float win[36])
{
    static const float pi = 3.14159265358979323846f;
    for (unsigned i = 0; i < 36; ++i) {
        float v;
        if (blockType == 2) {
            /* The short-block window is applied to each 12-sample IMDCT. */
            v = sinf(pi / 12.0f * ((float)i + 0.5f));
        } else if (blockType == 1) {
            /* start block */
            if (i < 18) v = sinf(pi / 36.0f * ((float)i + 0.5f));
            else if (i < 24) v = 1.0f;
            else if (i < 30) v = sinf(pi / 12.0f * ((float)(i - 18) + 0.5f));
            else v = 0.0f;
        } else if (blockType == 3) {
            /* end block */
            if (i < 6) v = 0.0f;
            else if (i < 12) v = sinf(pi / 12.0f * ((float)(i - 6) + 0.5f));
            else if (i < 18) v = 1.0f;
            else v = sinf(pi / 36.0f * ((float)(i - 18) + 0.5f));
        } else {
            v = sinf(pi / 36.0f * ((float)i + 0.5f));
        }
        win[i] = v;
    }
}

static void imdct18(const float in[18], float out[36])
{
    static float c[18][36];
    static bool init = false;
    if (!init) {
        const float pi = 3.14159265358979323846f;
        for (unsigned k=0;k<18;++k)
            for (unsigned n=0;n<36;++n)
                c[k][n] = cosf((pi/18.0f) * (n + 0.5f + 9.0f) * (k + 0.5f));
        init = true;
    }
    for (unsigned n=0;n<36;++n) {
        float sum=0.0f;
        for (unsigned k=0;k<18;++k) sum += in[k] * c[k][n];
        out[n]=sum * (2.0f / 18.0f);
    }
}

static void imdct6(const float in[6], float out[12])
{
    const float pi = 3.14159265358979323846f;
    for (unsigned n=0;n<12;++n) {
        float sum=0.0f;
        for (unsigned k=0;k<6;++k)
            sum += in[k] * cosf((pi/6.0f) * (n + 0.5f + 3.0f) * (k + 0.5f));
        out[n]=sum * (2.0f / 6.0f);
    }
}

/* Hybrid filterbank stage.  Produces 576 time-domain samples for one
 * granule/channel.  The caller keeps the 18-sample overlap per subband. */
static void hybridIMDCTMpeg1(const float xr[576], const SideGranule &g,
                             float out[576], float overlap[32][18])
{
    memset(out, 0, sizeof(float)*576);

    float win36[36];
    makeIMDCTWindow(g.window_switching_flag ? g.block_type : 0, win36);

    for (unsigned sb=0; sb<32; ++sb) {
        float block[36] = {};

        if (g.window_switching_flag && g.block_type == 2) {
            /* Three short IMDCTs.  Reordered xr is laid out as
             * [frequency-line 0: w0,w1,w2, ...]. */
            for (unsigned w=0; w<3; ++w) {
                float in6[6], y12[12];
                for (unsigned k=0;k<6;++k)
                    in6[k] = xr[sb*18 + 3*k + w];
                imdct6(in6,y12);
                for (unsigned n=0;n<12;++n) {
                    float sw = sinf(3.14159265358979323846f/12.0f * (n+0.5f));
                    block[6 + 6*w + n] += y12[n] * sw;
                }
            }
        } else {
            float in18[18], y36[36];
            for (unsigned k=0;k<18;++k) in18[k]=xr[sb*18+k];
            imdct18(in18,y36);
            for (unsigned n=0;n<36;++n) block[n]=y36[n]*win36[n];
        }

        for (unsigned n=0;n<18;++n) {
            out[sb*18+n] = block[n] + overlap[sb][n];
            overlap[sb][n] = block[n+18];
        }
    }
}

static void printHybridTest(unsigned grn, unsigned ch, const float out[576])
{
    float mn=out[0], mx=out[0], energy=0.0f;
    for (unsigned i=0;i<576;++i) {
        if(out[i]<mn) mn=out[i];
        if(out[i]>mx) mx=out[i];
        energy += out[i]*out[i];
    }
    USBSerial.printf("\nHYBRID IMDCT GR %u CH %u\n",grn,ch);
    USBSerial.println("--------------------------------------");
    USBSerial.printf("PCM-like samples : 576\n");
    USBSerial.printf("min               : %.6f\n",mn);
    USBSerial.printf("max               : %.6f\n",mx);
    USBSerial.printf("energy            : %.6f\n",energy);
    USBSerial.println("First 32 hybrid samples:");
    for(unsigned i=0;i<32;++i)
        USBSerial.printf("%4u: %12.6f%s",i,out[i],((i&3)==3)?"\n":"  ");
}

static float requantOne(float q, float exponent)
{
    if (q == 0.0f) return 0.0f;
    float a = powf(fabsf(q), 4.0f / 3.0f);
    float v = a * powf(2.0f, exponent);
    return (q < 0.0f) ? -v : v;
}

/*
 * MPEG-1 Layer III requantization.
 *
 * The output is xr[576], in frequency-line order.  For short blocks the
 * input is first reordered and the short-block scalefactors/subblock_gain
 * are applied per window.  For long/start/end blocks scalefac_l and the
 * optional pretab are applied per scalefactor band.
 */
static void requantizeMpeg1(const int16_t spectral[576],
                            const SideGranule &g,
                            const uint8_t sf[45],
                            float xr[576])
{
    int16_t reordered[576];
    const int16_t *is = spectral;

    if (g.window_switching_flag && g.block_type == 2) {
        if (g.mixed_block_flag)
            reorderMixedMpeg1(spectral, reordered);
        else
            reorderShortMpeg1(spectral, reordered);
        is = reordered;
    }

    for (unsigned i = 0; i < 576; ++i)
        xr[i] = 0.0f;

    const float gainExp = ((float)g.global_gain - 210.0f) * 0.25f;
    const float sfMult = g.scalefac_scale ? 2.0f : 1.0f;

    if (g.window_switching_flag && g.block_type == 2) {
        for (unsigned sfb = 0; sfb < 12; ++sfb) {
            unsigned start = sfbShort44[sfb];
            unsigned end   = sfbShort44[sfb + 1];

            for (unsigned i = start; i < end; ++i) {
                for (unsigned win = 0; win < 3; ++win) {
                    unsigned idx = 3 * i + win;
                    unsigned sfIndex = sfb * 3 + win;
                    float exponent = gainExp
                                   - sfMult * (float)sf[sfIndex]
                                   - 2.0f * (float)g.subblock_gain[win];
                    xr[idx] = requantOne((float)is[idx], exponent);
                }
            }
        }
        return;
    }

    /* Long, start and end blocks. */
    for (unsigned sfb = 0; sfb < 21; ++sfb) {
        unsigned start = sfbLong44[sfb];
        unsigned end   = sfbLong44[sfb + 1];
        float pre = g.preflag ? (float)pretabMpeg1[sfb] : 0.0f;
        float exponent = gainExp - sfMult * (float)(sf[sfb] + pre);

        for (unsigned i = start; i < end; ++i)
            xr[i] = requantOne((float)is[i], exponent);
    }
}

static void printRequantized(unsigned grn, unsigned ch,
                             const SideGranule &g,
                             const int16_t spectral[576],
                             const float xr[576])
{
    unsigned nonzero = 0;
    float minv = 0.0f, maxv = 0.0f;
    bool first = true;

    for (unsigned i = 0; i < 576; ++i) {
        if (spectral[i] != 0) {
            ++nonzero;
            if (first) {
                minv = maxv = xr[i];
                first = false;
            } else {
                if (xr[i] < minv) minv = xr[i];
                if (xr[i] > maxv) maxv = xr[i];
            }
        }
    }

    USBSerial.printf("\nREQUANTIZATION GR %u CH %u\n", grn, ch);
    USBSerial.println("--------------------------------------");
    USBSerial.printf("global_gain       : %u\n", g.global_gain);
    USBSerial.printf("block_type        : %u\n", g.block_type);
    USBSerial.printf("mixed_block       : %u\n", g.mixed_block_flag);
    USBSerial.printf("scalefac_scale     : %u\n", g.scalefac_scale);
    USBSerial.printf("Non-zero spectral  : %u\n", nonzero);
    USBSerial.printf("xr min             : %.6f\n", minv);
    USBSerial.printf("xr max             : %.6f\n", maxv);

    USBSerial.println("\nFirst 64 requantized values:");
    for (unsigned i = 0; i < 64; ++i) {
        USBSerial.printf("%4u: %12.6f%s", i, xr[i],
                         ((i & 3) == 3) ? "\n" : "  ");
    }
}

static void printScalefactors(unsigned grn, unsigned ch,
                              const SideGranule &g,
                              const uint8_t sf[45],
                              unsigned bitsUsed,
                              size_t startBit)
{
    USBSerial.printf("\nGR %u CH %u SCALEFACTORS\n", grn, ch);
    USBSerial.println("--------------------------------------");
    USBSerial.printf("Scalefactor start bit : %u\n", (unsigned)startBit);
    USBSerial.printf("Scalefactor bits used : %u\n", bitsUsed);
    USBSerial.printf("scalefac_compress     : %u\n", g.scalefac_compress);
    USBSerial.printf("slen1 / slen2         : %u / %u\n",
                     slen1[g.scalefac_compress & 0x0F],
                     slen2[g.scalefac_compress & 0x0F]);

    if (!g.window_switching_flag || g.block_type == 1 || g.block_type == 3) {
        USBSerial.println("layout                 : LONG (start/end/normal)");
        USBSerial.println("sfb : value");
        for (unsigned sfb=0; sfb<21; ++sfb)
            USBSerial.printf("%2u  : %u\n", sfb, (unsigned)sf[sfb]);
    } else if (g.mixed_block_flag) {
        USBSerial.println("layout                 : MIXED");
        USBSerial.println("long sfb : value");
        for (unsigned sfb=0; sfb<8; ++sfb)
            USBSerial.printf("%2u       : %u\n", sfb, (unsigned)sf[sfb]);
        USBSerial.println("short sfb/window : value");
        unsigned k=8;
        for (unsigned sfb=3; sfb<12; ++sfb)
            for (unsigned win=0; win<3; ++win)
                USBSerial.printf("%2u/%u             : %u\n",
                                 sfb, win, (unsigned)sf[k++]);
    } else {
        USBSerial.println("layout                 : SHORT");
        USBSerial.println("sfb/window : value");
        unsigned k=0;
        for (unsigned sfb=0; sfb<12; ++sfb)
            for (unsigned win=0; win<3; ++win)
                USBSerial.printf("%2u/%u      : %u\n",
                                 sfb, win, (unsigned)sf[k++]);
    }
}

static void printGranule(unsigned grn, unsigned ch, const SideGranule &g) {
    USBSerial.printf("GR %u CH %u\n", grn, ch);
    USBSerial.printf("  part2_3_length : %u\n", g.part2_3_length);
    USBSerial.printf("  big_values     : %u\n", g.big_values);
    USBSerial.printf("  global_gain    : %u\n", g.global_gain);
    USBSerial.printf("  scalefac_comp  : %u\n", g.scalefac_compress);
    USBSerial.printf("  window_switch  : %u\n", g.window_switching_flag);

    if (g.window_switching_flag) {
        USBSerial.printf("  block_type     : %u\n", g.block_type);
        USBSerial.printf("  mixed_block    : %u\n", g.mixed_block_flag);
        USBSerial.printf("  table_select   : %u %u\n",
                         g.table_select[0], g.table_select[1]);
        USBSerial.printf("  subblock_gain  : %u %u %u\n",
                         g.subblock_gain[0], g.subblock_gain[1],
                         g.subblock_gain[2]);
    } else {
        USBSerial.printf("  table_select   : %u %u %u\n",
                         g.table_select[0], g.table_select[1],
                         g.table_select[2]);
        USBSerial.printf("  region0_count  : %u\n", g.region0_count);
        USBSerial.printf("  region1_count  : %u\n", g.region1_count);
    }

    USBSerial.printf("  preflag        : %u\n", g.preflag);
    USBSerial.printf("  scalefac_scale : %u\n", g.scalefac_scale);
    USBSerial.printf("  count1_table   : %u\n", g.count1table_select);
}

static int findSync(const uint8_t *p, size_t n) {
    for (size_t i=0;i+1<n;++i)
        if (p[i]==0xFF && (p[i+1]&0xE0)==0xE0)
            return (int)i;
    return -1;
}

static bool validMpeg1L3Header(const uint8_t *h) {
    if (h[0] != 0xFF || (h[1]&0xE0)!=0xE0) return false;
    unsigned version = (h[1]>>3)&3;
    unsigned layer = (h[1]>>1)&3;
    unsigned br = (h[2]>>4)&15;
    unsigned sr = (h[2]>>2)&3;
    if (version != 3 || layer != 1) return false;
    if (br==0 || br==15 || sr==3) return false;
    return true;
}

static unsigned bitrateKbps(unsigned index) {
    static const uint16_t t[16] =
        {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0};
    return t[index&15];
}

static unsigned sampleRate(unsigned index) {
    static const uint32_t t[4] = {44100,48000,32000,0};
    return t[index&3];
}

static unsigned frameSize(const uint8_t *h) {
    unsigned br = bitrateKbps((h[2]>>4)&15);
    unsigned sr = sampleRate((h[2]>>2)&3);
    unsigned pad = (h[2]>>1)&1;
    return (144000u*br)/sr + pad;
}

/*
 * Stage 1 decoder test.
 *
 * It finds the first real audio frame after ID3/Info, parses its side info,
 * and locates the beginning of part2_3 data.  It also shows the first bits
 * that the Huffman/scalefactor stage will consume.
 */

static void printStageSummary(unsigned grn, unsigned ch,
                              const int16_t spectral[576],
                              const float xr[576],
                              const float hybrid[576],
                              const int16_t pcm[576])
{
    unsigned nzSpec = 0;
    float xrMin = 0.0f, xrMax = 0.0f;
    float hyMin = hybrid[0], hyMax = hybrid[0];
    int16_t pcmMin = pcm[0], pcmMax = pcm[0];
    uint64_t pcmEnergy = 0;
    bool xrFirst = true;

    for (unsigned i = 0; i < 576; ++i) {
        if (spectral[i] != 0) ++nzSpec;

        if (xrFirst) {
            xrMin = xrMax = xr[i];
            xrFirst = false;
        } else {
            if (xr[i] < xrMin) xrMin = xr[i];
            if (xr[i] > xrMax) xrMax = xr[i];
        }

        if (hybrid[i] < hyMin) hyMin = hybrid[i];
        if (hybrid[i] > hyMax) hyMax = hybrid[i];

        if (pcm[i] < pcmMin) pcmMin = pcm[i];
        if (pcm[i] > pcmMax) pcmMax = pcm[i];

        int32_t v = pcm[i];
        pcmEnergy += (uint64_t)(v * (int64_t)v);
    }

    USBSerial.printf("\n******** DIAGNOSTIC GR %u CH %u ********\n", grn, ch);
    USBSerial.printf("1 SPECTRAL nonzero : %u / 576\n", nzSpec);
    USBSerial.printf("2 XR min/max       : %.6f / %.6f\n", xrMin, xrMax);
    USBSerial.printf("3 HYBRID min/max   : %.6f / %.6f\n", hyMin, hyMax);
    USBSerial.printf("4 PCM min/max      : %d / %d\n", (int)pcmMin, (int)pcmMax);
    USBSerial.printf("  PCM energy       : %llu\n",
                      (unsigned long long)pcmEnergy);

    USBSerial.print("  PCM first 16     :");
    for (unsigned i = 0; i < 16; ++i)
        USBSerial.printf(" %d", (int)pcm[i]);
    USBSerial.println();
    USBSerial.println("****************************************");
}

static void printPcmTest(unsigned grn, unsigned ch, const int16_t pcm[576])
{
    int16_t mn = pcm[0];
    int16_t mx = pcm[0];
    uint64_t energy = 0;

    for (unsigned i=0; i<576; ++i) {
        if (pcm[i] < mn) mn = pcm[i];
        if (pcm[i] > mx) mx = pcm[i];
        int32_t v = pcm[i];
        energy += (uint64_t)(v * (int64_t)v);
    }

    USBSerial.printf("\nPOLYPHASE SYNTHESIS GR %u CH %u\n", grn, ch);
    USBSerial.println("--------------------------------------");
    USBSerial.println("PCM samples       : 576");
    USBSerial.printf("min               : %d\n", (int)mn);
    USBSerial.printf("max               : %d\n", (int)mx);
    USBSerial.printf("energy            : %llu\n", (unsigned long long)energy);
    USBSerial.println("First 32 PCM samples:");
    for (unsigned i=0; i<32; ++i)
        USBSerial.printf("%4u:%7d%s", i, (int)pcm[i],
                         ((i & 7) == 7) ? "\n" : "  ");
}

/* Stage 1..10 development decoder test. */

/*
 * Stage16 continuous decoder/player.
 *
 * Unlike the previous diagnostic stage, this function does NOT replay one
 * PCM buffer and does NOT run a frame scanner after playback.  It decodes
 * successive MPEG-1 Layer III frames, keeps the main-data reservoir, runs
 * the existing hybrid + synthesis state continuously, and queues each
 * real PCM frame to the PWM audio ring buffer.
 */
static bool decodeOneStreamingFrame(const uint8_t *frame,
                                    unsigned fs,
                                    unsigned frameNo,
                                    uint8_t reservoir[511],
                                    unsigned reservoirBytes,
                                    unsigned &concealedGranules)
{
    const uint8_t *h = frame;
    unsigned channels = (((h[3] >> 6) & 3) == 3) ? 1 : 2;
    unsigned crcBytes = ((h[1] & 1) == 0) ? 2 : 0;
    unsigned sideBytes = (channels == 1) ? 17 : 32;

    if (fs < 4 + crcBytes + sideBytes)
        return false;

    uint16_t mainBegin = 0;
    uint8_t scfsi[2][4] = {};
    SideGranule gr[2][2] = {};
    unsigned sideBits = 0;

    if (!parseSideInfo(frame + 4 + crcBytes, channels,
                       mainBegin, scfsi, gr, sideBits)) {
        USBSerial.printf("FRAME %u: side-info ERROR\n", frameNo);
        return false;
    }

    unsigned mainOffset = 4 + crcBytes + sideBytes;
    unsigned mainBytes = fs - mainOffset;

    /*
     * main_data_begin points backwards from the current frame's main-data
     * start.  The previous 511 bytes are therefore the only reservoir
     * storage needed by MPEG-1 Layer III.
     */
    if (mainBegin > reservoirBytes) {
        USBSerial.printf("FRAME %u: reservoir need=%u have=%u -> skip\n",
                         frameNo, mainBegin, reservoirBytes);
        return false;
    }

    static uint8_t virtualMain[511 + 4096];
    const unsigned prefix = mainBegin;

    if (prefix)
        memcpy(virtualMain,
               reservoir + (reservoirBytes - prefix),
               prefix);

    memcpy(virtualMain + prefix, frame + mainOffset, mainBytes);

    const unsigned virtualBytes = prefix + mainBytes;
    MP3BitReader mainBR(virtualMain, virtualBytes);

    /* IMPORTANT: part2_3 starts at the beginning of the virtual stream.
       main_data_begin selects the bytes immediately preceding the current
       frame's main data; those bytes are part of the bitstream, not a
       prefix to skip. */

    /*
     * Decode all granules/channels into the persistent decoder state.
     * Nothing is sent to PWM until the complete frame has been decoded.
     */
    uint8_t sfPrev[2][45] = {};

    for (unsigned grn = 0; grn < 2; ++grn) {
        for (unsigned ch = 0; ch < channels; ++ch) {
            SideGranule &g = gr[grn][ch];
            float *xr = gXR[ch];

            memset(xr, 0, sizeof(gXR[ch]));

            if (g.part2_3_length == 0)
                continue;

            if (mainBR.bitsLeft() < g.part2_3_length) {
                USBSerial.printf("FRAME %u GR%u CH%u: part2_3 exceeds data\n",
                                 frameNo, grn, ch);
                return false;
            }

            size_t blockStart = mainBR.bitsRead();

            uint8_t sf[45] = {};
            unsigned valueCount = 0;

            readScalefactorsMpeg1(
                mainBR, g, scfsi[ch], grn == 1, sf, valueCount);

            /*
             * MPEG-1 SCFSI does not consume bits for reused bands in
             * granule 1.  The values are copied from granule 0 of the
             * same channel; zeroing them is bit-alignment-safe but gives
             * incorrect requantization and audible level/timbre errors.
             */
            if (grn == 1 && !g.window_switching_flag) {
                for (unsigned sfb = 0; sfb < 21; ++sfb) {
                    unsigned group = (sfb <= 5) ? 0 :
                                     (sfb <= 10) ? 1 :
                                     (sfb <= 15) ? 2 : 3;
                    if (scfsi[ch][group])
                        sf[sfb] = sfPrev[ch][sfb];
                }
            }

            if (grn == 0 && !g.window_switching_flag)
                memcpy(sfPrev[ch], sf, 21);

            size_t consumed = mainBR.bitsRead() - blockStart;

            if (consumed > g.part2_3_length) {
                USBSerial.printf("FRAME %u GR%u CH%u: scalefactor overflow\n",
                                 frameNo, grn, ch);
                return false;
            }

            unsigned huffBits =
                g.part2_3_length - (unsigned)consumed;

            /* All MPEG-1 Layer III Huffman tables 0..31 and Count1 32/33
               are implemented in MP3Huffman. */
            if (g.table_select[0] > 31 ||
                g.table_select[1] > 31 ||
                (!g.window_switching_flag && g.table_select[2] > 31)) {
                USBSerial.printf("FRAME %u GR%u CH%u: invalid Huffman table %u/%u/%u\n",
                                 frameNo, grn, ch,
                                 g.table_select[0], g.table_select[1], g.table_select[2]);
                mainBR.skip(huffBits);
                return false;
            }

            int16_t *spectral = gSpectral[ch];
            memset(spectral, 0, sizeof(gSpectral[ch]));

            unsigned pairsDecoded = 0;
            unsigned quadsDecoded = 0;
            unsigned bitsUsed = 0;
            uint32_t first24 = mainBR.peek(24);

            bool ok = MP3Huffman::decodeGranule(
                mainBR,
                huffBits,
                g.big_values,
                g.table_select[0],
                g.table_select[1],
                g.window_switching_flag ? 0 : g.table_select[2],
                (g.window_switching_flag && g.block_type == 2) ? 36u :
                    (unsigned)sfbLong44[(g.region0_count + 1 > 22) ? 22 : g.region0_count + 1],
                (g.window_switching_flag && g.block_type == 2) ? 576u :
                    (unsigned)sfbLong44[((g.region0_count + 1 + g.region1_count + 1) > 22) ? 22 :
                                         (g.region0_count + 1 + g.region1_count + 1)],
                g.count1table_select ? 33 : 32,
                spectral,
                pairsDecoded,
                quadsDecoded,
                bitsUsed);

            if (!ok) {
                /*
                 * Diagnostic concealment: do NOT drop the whole MPEG frame.
                 * Replace only this failed granule/channel with zero spectral
                 * coefficients, then advance to the exact end of its part2_3
                 * region. This preserves the 1152-sample time grid and keeps
                 * the persistent synthesis overlap continuous.
                 */
                ++concealedGranules;
                USBSerial.printf(
                    "FRAME %u GR%u CH%u: HUFFMAN CONCEALED "
                    "table=%u/%u/%u big=%u part23=%u sfbits=%u "
                    "huffbits=%u used=%u block=%u mixed=%u win=%u "
                    "r0=%u r1=%u count1=%u at=%u\n",
                    frameNo, grn, ch,
                    g.table_select[0], g.table_select[1], g.table_select[2],
                    g.big_values, g.part2_3_length, (unsigned)consumed,
                    huffBits, bitsUsed, g.block_type, g.mixed_block_flag,
                    g.window_switching_flag, g.region0_count,
                    g.region1_count, g.count1table_select,
                    (unsigned)mainBR.bitsRead());
                USBSerial.printf("  HUFF first24=0x%06lX startBit=%u\n",
                                 (unsigned long)first24,
                                 (unsigned)blockStart);

                /* spectral[] was zeroed before Huffman decoding. */
                memset(spectral, 0, sizeof(gSpectral[ch]));

                size_t part23End = blockStart + g.part2_3_length;
                if (mainBR.bitsRead() < part23End) {
                    mainBR.skip((unsigned)(part23End - mainBR.bitsRead()));
                } else if (mainBR.bitsRead() > part23End) {
                    USBSerial.printf(
                        "FRAME %u GR%u CH%u: concealment boundary overrun\n",
                        frameNo, grn, ch);
                    return false;
                }

                requantizeMpeg1(spectral, g, sf, xr);
                continue;
            }

            /*
             * The part2_3 region has a fixed length in the side info.
             * Huffman decoding may finish before the end because the
             * remaining bits are stuffing.  Always land exactly on the
             * end of this granule/channel region before decoding the next
             * one.
             */
            size_t part23End = blockStart + g.part2_3_length;
            if (mainBR.bitsRead() > part23End) {
                USBSerial.printf("FRAME %u GR%u CH%u: part2_3 boundary overrun\n",
                                 frameNo, grn, ch);
                return false;
            }
            if (mainBR.bitsRead() < part23End)
                mainBR.skip((unsigned)(part23End - mainBR.bitsRead()));

            requantizeMpeg1(spectral, g, sf, xr);

            /* Stage21 diagnostic: verify the magnitude entering alias/IMDCT. */
            if (frameNo <= 3 || (frameNo % 25) == 0) {
                float xrMax = 0.0f;
                for (unsigned qi = 0; qi < 576; ++qi) {
                    float av = fabsf(xr[qi]);
                    if (av > xrMax) xrMax = av;
                }
                USBSerial.printf("STAGE21 SCALE frame=%u GR%u CH%u maxXR=%.6f\n",
                                 frameNo, grn, ch, xrMax);
            }

            /*
             * decodeGranule consumes exactly the complete part2_3 region,
             * including stuffing, so the next granule starts at the correct
             * bit position.
             */
        }
    }

    /* Joint stereo conversion must happen after both channels are decoded. */
    unsigned channelMode = (h[3] >> 6) & 3;
    unsigned modeExt = (h[3] >> 4) & 3;
    bool msStereo = (channels == 2 &&
                     channelMode == 1 &&
                     (modeExt & 2));

    if (channels == 2 && msStereo)
        applyMSStereo(gXR[0], gXR[1]);

    /*
     * Hybrid overlap and the synthesis filterbank are persistent globals.
     * Therefore successive MP3 frames are joined naturally here.
     */
    for (unsigned grn = 0; grn < 2; ++grn) {
        for (unsigned ch = 0; ch < channels; ++ch) {
            SideGranule &g = gr[grn][ch];

            float *xr = gXR[ch];
            float *hybrid = gHybrid[ch];
            int16_t *pcm = &gPCM[ch][grn * 576];

            aliasReduceMpeg1(xr, g);

            memset(hybrid, 0, sizeof(gHybrid[ch]));

            hybridIMDCTMpeg1(
                xr, g, hybrid,
                gHybridOverlap[ch]);

            mp3SynthesisProcess(ch, hybrid, pcm);
        }
    }

    unsigned audioRate = sampleRate((h[2] >> 2) & 3);

    /*
     * The PCM is produced by the real MP3 decoder.  There is no generated
     * sine wave or diagnostic PCM in this path.
     */
    /*
     * Optional short real-PCM capture.  This is intentionally placed
     * AFTER synthesis and BEFORE PWM so the WAV contains exactly the PCM
     * that is fed to the audio output path.
     */
    mp3PcmCaptureFrame(gPCM[0],
                       channels == 2 ? gPCM[1] : gPCM[0],
                       1152, channels, audioRate);

#if !MP3_WAV_ONLY
    if (channels == 2)
        mp3PwmQueueStereo(gPCM[0], gPCM[1], 1152, audioRate);
    else
        mp3PwmQueueStereo(gPCM[0], gPCM[0], 1152, audioRate);
#endif

    if (frameNo < 4 || (frameNo % 25) == 0) {
        int16_t mn = 32767;
        int16_t mx = -32768;

        for (unsigned i = 0; i < 1152; ++i) {
            int32_t m = channels == 2
                      ? ((int32_t)gPCM[0][i] +
                         (int32_t)gPCM[1][i]) / 2
                      : gPCM[0][i];

            if (m < mn) mn = (int16_t)m;
            if (m > mx) mx = (int16_t)m;
        }

        USBSerial.printf(
            "FRAME %u: decoded -> PCM queued, rate=%u, min=%d max=%d\n",
            frameNo, audioRate, (int)mn, (int)mx);
    }

    return true;
}

static bool readNextMpeg1Frame(FILE *file, long &offset, uint8_t frame[4096], unsigned &fs);

static void playFileContinuously(FILE *file)
{
    static uint8_t frame[4096];
    static uint8_t reservoir[511];

    unsigned reservoirBytes = 0;
    long offset = 0;
    unsigned frameNo = 0;
    unsigned decoded = 0;
    unsigned skipped = 0;
    unsigned concealed = 0;
    unsigned audioRate = 44100;
    uint64_t totalPCMSamples = 0;
    uint64_t totalDecodedMs = 0;
    unsigned long underrunsAtLastReport = 0;
    bool pwmStarted = false;

    memset(reservoir, 0, sizeof(reservoir));

    /*
     * Reset all temporal decoder state exactly once for this file.
     * Do NOT reset synthesis between frames.
     */
    memset(gSpectral, 0, sizeof(gSpectral));
    memset(gReordered, 0, sizeof(gReordered));
    memset(gXR, 0, sizeof(gXR));
    memset(gHybrid, 0, sizeof(gHybrid));
    memset(gHybridOverlap, 0, sizeof(gHybridOverlap));
    memset(gPCM, 0, sizeof(gPCM));
    mp3SynthesisReset();
    mp3PcmCaptureReset();

    USBSerial.println();
    USBSerial.println("======================================");
    USBSerial.println("STAGE16 REAL MP3 CONTINUOUS PLAYBACK");
    USBSerial.println("NO TEST SINE");
    USBSerial.println("NO PCM REPEAT");
    USBSerial.println("MP3 -> DECODE -> SYNTHESIS -> PWM");
    USBSerial.println("======================================");

    for (;;) {
        unsigned fs = 0;
        long frameOffset = offset;

        if (!readNextMpeg1Frame(file, offset, frame, fs))
            break;

        ++frameNo;

        const uint8_t *h = frame;
        audioRate = sampleRate((h[2] >> 2) & 3);

        if (audioRate == 0) {
            ++skipped;
        } else {
            bool ok = decodeOneStreamingFrame(
                frame, fs, frameNo,
                reservoir, reservoirBytes,
                concealed);

            if (ok) {
                ++decoded;
                /*
                 * MPEG-1 Layer III produces 1152 PCM samples per decoded
                 * frame. This is the PCM that was actually queued to PWM.
                 */
                totalPCMSamples += 1152ULL;
                totalDecodedMs =
                    (totalPCMSamples * 1000ULL) / audioRate;
#if MP3_WAV_ONLY
                if (mp3PcmCaptureIsFinished()) {
                    USBSerial.println("WAV capture reached 5-second target.");
                    break;
                }
#endif
            } else {
                ++skipped;
            }

            /*
             * Print a compact progress report often enough to prove that
             * multiple REAL MP3 frames are being decoded and consumed.
             */
            if ((frameNo <= 5) || ((frameNo % 10) == 0)) {
                unsigned q = 0;
                unsigned long u = 0;
#if !MP3_WAV_ONLY
                q = mp3PwmQueuedSamples();
                u = mp3PwmUnderruns();
#endif

                USBSerial.printf(
                    "PROGRESS frame=%u decoded=%u skipped=%u concealed=%u "
                    "PCM=%llu samples time=%llu ms queue=%u underrun=%lu\n",
                    frameNo,
                    decoded,
                    skipped,
                    concealed,
                    (unsigned long long)totalPCMSamples,
                    (unsigned long long)totalDecodedMs,
                    q,
                    u);

                underrunsAtLastReport = u;
            }

            /*
             * Start PWM only after several real PCM frames are buffered.
             * This gives the decoder some headroom before playback begins.
             */
#if !MP3_WAV_ONLY
            if (!pwmStarted && mp3PwmQueuedSamples() >= 4096) {
                mp3PwmStartStream(audioRate);
                pwmStarted = true;

                USBSerial.printf(
                    "PWM STREAM STARTED: rate=%u queued=%u samples\n",
                    audioRate, mp3PwmQueuedSamples());
            }
#endif
        }

        /*
         * Update the MPEG-1 byte reservoir AFTER decoding the current frame.
         * Only the final 511 bytes of this frame can be referenced by the
         * following frame.
         */
        unsigned crcBytes = ((h[1] & 1) == 0) ? 2 : 0;
        unsigned channels = (((h[3] >> 6) & 3) == 3) ? 1 : 2;
        unsigned sideBytes = (channels == 1) ? 17 : 32;
        unsigned mainOffset = 4 + crcBytes + sideBytes;

        if (fs >= mainOffset) {
            unsigned mainBytes = fs - mainOffset;
            static uint8_t combinedReservoir[511 + 4096];
            unsigned total = reservoirBytes + mainBytes;
            if (reservoirBytes)
                memcpy(combinedReservoir, reservoir, reservoirBytes);
            memcpy(combinedReservoir + reservoirBytes, frame + mainOffset, mainBytes);

            unsigned keep = total > 511 ? 511 : total;
            if (keep)
                memcpy(reservoir, combinedReservoir + (total - keep), keep);
            reservoirBytes = keep;
        }

        if ((frameNo & 7u) == 0)
            yield();

        (void)frameOffset;
    }

#if !MP3_WAV_ONLY
    if (!pwmStarted && mp3PwmQueuedSamples() != 0) {
        mp3PwmStartStream(audioRate);
        pwmStarted = true;
    }
#endif

    USBSerial.println();
    USBSerial.println("--------------------------------------");
    USBSerial.printf("MP3 frames read      : %u\n", frameNo);
    USBSerial.printf("Frames decoded       : %u\n", decoded);
    USBSerial.printf("Frames skipped       : %u\n", skipped);
    USBSerial.printf("Granules concealed   : %u\n", concealed);
    USBSerial.printf("PCM samples produced : %llu\n",
                     (unsigned long long)totalPCMSamples);
    USBSerial.printf("PCM audio time       : %llu ms (%.3f sec)\n",
                     (unsigned long long)totalDecodedMs,
                     audioRate ? ((double)totalPCMSamples / (double)audioRate) : 0.0);
    USBSerial.printf("PCM still queued     : %u\n",
#if MP3_WAV_ONLY
                     0U);
#else
                     mp3PwmQueuedSamples());
#endif

    mp3PcmCaptureFinish();
    USBSerial.printf("PWM underruns        : %lu\n",
#if MP3_WAV_ONLY
                     0UL);
#else
                     (unsigned long)mp3PwmUnderruns());
#endif
    USBSerial.printf("Underruns at report  : %lu\n",
#if MP3_WAV_ONLY
                     0UL);
#else
                     underrunsAtLastReport);
#endif
    USBSerial.println("File decode finished.");
    USBSerial.println("--------------------------------------");

    /*
     * Do not stop PWM immediately.  Let the remaining decoded PCM drain.
     */
#if !MP3_WAV_ONLY
    while (mp3PwmQueuedSamples() != 0)
        delay(20);

    mp3PwmStop();
#endif
}

static bool readNextMpeg1Frame(FILE *file, long &offset, uint8_t frame[4096], unsigned &fs)
{
    if (!file) return false;
    uint8_t h[4];
    long pos = offset;
    for (;;) {
        if (fseek(file, pos, SEEK_SET) != 0) return false;
        if (fread(h,1,4,file) != 4) return false;
        if (validMpeg1L3Header(h)) {
            fs = frameSize(h);
            if (fs >= 100 && fs <= 4096) {
                memcpy(frame,h,4);
                if (fread(frame+4,1,fs-4,file) == fs-4) {
                    offset = pos + (long)fs;
                    return true;
                }
                return false;
            }
        }
        ++pos;
    }
}

static void printFrameSummary(unsigned frameNo, long offset, unsigned fs,
                              const uint8_t *h, unsigned mainBegin,
                              unsigned mainBytes, unsigned channels)
{
    unsigned br = bitrateKbps((h[2]>>4)&15);
    unsigned sr = sampleRate((h[2]>>2)&3);
    USBSerial.printf("FRAME %u  offset=%ld size=%u bitrate=%u kbps sr=%u channels=%u main_data_begin=%u main=%u\n",
                     frameNo, offset, fs, br, sr, channels, mainBegin, mainBytes);
}

/*
 * Scan every MPEG-1 Layer III frame in the file and maintain the byte
 * reservoir exactly as specified by main_data_begin.  This stage deliberately
 * does not pretend that the current Huffman subset can decode every possible
 * table.  It verifies the frame chain, side-info boundaries and reservoir
 * continuity first; unsupported Huffman tables are reported rather than
 * silently producing invalid PCM.
 */
static void scanAllFramesWithReservoir(FILE *file)
{
    static uint8_t frame[4096];
    static uint8_t reservoir[511];
    static uint8_t newReservoir[511];
    memset(reservoir, 0, sizeof(reservoir));
    unsigned reservoirBytes = 0;
    long offset = 0;
    unsigned frameNo = 0;
    unsigned decodedCandidate = 0;
    unsigned reservoirWait = 0;
    unsigned unsupported = 0;
    unsigned bad = 0;

    USBSerial.println();
    USBSerial.println("======================================");
    USBSerial.println("MP3 CONTINUOUS FRAME / BIT RESERVOIR TEST");
    USBSerial.println("======================================");

    for (;;) {
        unsigned fs = 0;
        long frameOffset = offset;
        if (!readNextMpeg1Frame(file, offset, frame, fs)) break;

        ++frameNo;
        const uint8_t *h = frame;
        unsigned channels = (((h[3]>>6)&3)==3) ? 1 : 2;
        unsigned crcBytes = ((h[1]&1)==0) ? 2 : 0;
        unsigned sideBytes = (channels == 1) ? 17 : 32;
        if (fs < 4 + crcBytes + sideBytes) { ++bad; continue; }

        uint16_t mainBegin = 0;
        uint8_t scfsi[2][4] = {};
        SideGranule gr[2][2] = {};
        unsigned sideBits = 0;
        if (!parseSideInfo(frame + 4 + crcBytes, channels, mainBegin,
                           scfsi, gr, sideBits)) {
            ++bad;
            USBSerial.printf("FRAME %u: invalid side info\n", frameNo);
            continue;
        }

        unsigned mainOffset = 4 + crcBytes + sideBytes;
        unsigned mainBytes = fs - mainOffset;
        if (mainBegin > reservoirBytes) {
            ++reservoirWait;
            USBSerial.printf("FRAME %u: NEED %u reservoir bytes, have %u -> cannot decode yet\n",
                             frameNo, mainBegin, reservoirBytes);
        }

        printFrameSummary(frameNo, frameOffset, fs, h, mainBegin, mainBytes, channels);

        /* The virtual main-data stream begins main_data_begin bytes before
           the current frame's main-data start. */
        if (mainBegin <= reservoirBytes) {
            static uint8_t virtualMain[511 + 4096];
            unsigned prefix = mainBegin;
            memcpy(virtualMain, reservoir + (reservoirBytes - prefix), prefix);
            memcpy(virtualMain + prefix, frame + mainOffset, mainBytes);
            unsigned virtualBytes = prefix + mainBytes;
            MP3BitReader br(virtualMain, virtualBytes);

            bool tablesSupported = true;
            for (unsigned grn=0; grn<2 && tablesSupported; ++grn) {
                for (unsigned ch=0; ch<channels; ++ch) {
                    const SideGranule &g = gr[grn][ch];
                    if (g.part2_3_length == 0) continue;
                    if (g.table_select[0] != 8 && g.table_select[0] != 9 &&
                        g.table_select[0] != 25) {
                        tablesSupported = false;
                        USBSerial.printf("  unsupported Huffman table GR%u CH%u: %u/%u\n",
                                         grn,ch,g.table_select[0],g.table_select[1]);
                    }
                }
            }
            if (tablesSupported) ++decodedCandidate;
            else ++unsupported;
        }

        /* Keep exactly the last 511 bytes of main data for the next frame. */
        unsigned keep = mainBytes > 511 ? 511 : mainBytes;
        if (keep) memcpy(newReservoir, frame + mainOffset + (mainBytes - keep), keep);
        memcpy(reservoir, newReservoir, keep);
        reservoirBytes = keep;

        if ((frameNo & 7u) == 0) yield();
    }

    USBSerial.println();
    USBSerial.println("--------------------------------------");
    USBSerial.printf("Frames scanned                 : %u\n", frameNo);
    USBSerial.printf("Frames with supported tables  : %u\n", decodedCandidate);
    USBSerial.printf("Frames with unsupported tables: %u\n", unsupported);
    USBSerial.printf("Reservoir unavailable frames  : %u\n", reservoirWait);
    USBSerial.printf("Bad frames                    : %u\n", bad);
    USBSerial.println("Reservoir scan status          : COMPLETE");
}

void mp3DecoderTest(FILE *file) {
    if (!file) {
        USBSerial.println("MP3 decoder: NULL file");
        return;
    }

    fseek(file, 0, SEEK_SET);
    playFileContinuously(file);
    fseek(file, 0, SEEK_SET);
}
