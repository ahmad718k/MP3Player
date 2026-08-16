#ifndef MP3_SYNTHESIS_H
#define MP3_SYNTHESIS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Reset the MPEG Layer III 32-subband synthesis state.
 *
 * Call exactly once before starting a new MP3 file.
 * Do NOT call this between frames or granules.
 */
void mp3SynthesisReset(void);

/*
 * Convert one 576-sample Hybrid granule into 576 signed 16-bit PCM samples.
 *
 * inputHybrid:
 *     32 subbands x 18 time slots = 576 values
 *
 * pcmOut:
 *     18 synthesis slots x 32 PCM samples = 576 samples
 *
 * channel:
 *     0 = left
 *     1 = right
 */
void mp3SynthesisProcess(unsigned channel,
                         const float inputHybrid[576],
                         int16_t pcmOut[576]);

#ifdef __cplusplus
}
#endif

#endif
