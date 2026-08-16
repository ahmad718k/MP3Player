#ifndef MP3_PCM_CAPTURE_H
#define MP3_PCM_CAPTURE_H

#include <stdint.h>

#ifndef MP3_PCM_CAPTURE_ENABLE
#define MP3_PCM_CAPTURE_ENABLE 1
#endif

/* Maximum WAV duration used for the decoder verification test. */
#ifndef MP3_PCM_CAPTURE_SECONDS
#define MP3_PCM_CAPTURE_SECONDS 5U
#endif

#ifdef __cplusplus
extern "C" {
#endif

void mp3PcmCaptureReset(void);
void mp3PcmCaptureFrame(const int16_t *left,
                        const int16_t *right,
                        unsigned samples,
                        unsigned channels,
                        unsigned sampleRate);
void mp3PcmCaptureFinish(void);
bool mp3PcmCaptureIsFinished(void);

#ifdef __cplusplus
}
#endif

#endif
