#ifndef MP3_PWM_OUTPUT_H
#define MP3_PWM_OUTPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Stage16 streaming PWM output.
 *
 * MP3 decoder -> signed 16-bit PCM -> 8-bit duty ring buffer
 * -> hardware timer -> LEDC carrier -> GPIO16.
 *
 * No test waveform is generated here.
 */
void mp3PwmQueueStereo(const int16_t *left,
                       const int16_t *right,
                       unsigned samples,
                       unsigned sampleRate);

bool mp3PwmStartStream(unsigned sampleRate);

void mp3PwmStop(void);

unsigned mp3PwmQueuedSamples(void);

unsigned long mp3PwmUnderruns(void);

/* Backward-compatible diagnostic API. */
void mp3PwmPlayStereo(const int16_t *left,
                      const int16_t *right,
                      unsigned samples,
                      unsigned sampleRate,
                      unsigned repeats);

#ifdef __cplusplus
}
#endif

#endif
