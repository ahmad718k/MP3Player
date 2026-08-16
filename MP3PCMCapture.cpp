#include "MP3PCMCapture.h"

#if MP3_PCM_CAPTURE_ENABLE

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "USB.h"

extern USBCDC USBSerial;

static FILE *gFile = nullptr;
static uint32_t gRate = 0;
static uint16_t gChannels = 0;
static uint64_t gFrames = 0;       // stereo/mono time samples written
static uint64_t gDataBytes = 0;
static bool gFinished = false;

static void putLE16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static void putLE32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}

static bool writeWavHeader(FILE *f, uint32_t rate, uint16_t channels, uint32_t dataBytes)
{
    if (!f || (channels != 1 && channels != 2) || rate == 0)
        return false;

    uint8_t h[44];
    memset(h, 0, sizeof(h));

    memcpy(h + 0, "RIFF", 4);
    putLE32(h + 4, 36U + dataBytes);
    memcpy(h + 8, "WAVE", 4);
    memcpy(h + 12, "fmt ", 4);
    putLE32(h + 16, 16);
    putLE16(h + 20, 1);                 // PCM
    putLE16(h + 22, channels);
    putLE32(h + 24, rate);
    putLE32(h + 28, rate * channels * 2U);
    putLE16(h + 32, channels * 2U);
    putLE16(h + 34, 16);
    memcpy(h + 36, "data", 4);
    putLE32(h + 40, dataBytes);

    if (fseek(f, 0, SEEK_SET) != 0)
        return false;

    return fwrite(h, 1, sizeof(h), f) == sizeof(h);
}

void mp3PcmCaptureReset(void)
{
    if (gFile) {
        fclose(gFile);
        gFile = nullptr;
    }

    gRate = 0;
    gChannels = 0;
    gFrames = 0;
    gDataBytes = 0;
    gFinished = false;

    USBSerial.println();
    USBSerial.println("========== WAV CAPTURE BEGIN ==========");
    USBSerial.println("WAV path     : /storage/test.wav");
    USBSerial.printf("WAV max time : %u seconds\n", MP3_PCM_CAPTURE_SECONDS);
    USBSerial.println("WAV format   : PCM signed 16-bit little-endian");
    USBSerial.println("WAV: opening file...");

    /* wb truncates the previous test.wav on every run. */
    gFile = fopen("/storage/test.wav", "wb+");
    if (!gFile) {
        USBSerial.println("WAV: ERROR opening /storage/test.wav for write");
        return;
    }

    USBSerial.println("WAV: file opened");

    /* Reserve the canonical 44-byte WAV header. */
    uint8_t blank[44] = {};
    size_t n = fwrite(blank, 1, sizeof(blank), gFile);
    if (n != sizeof(blank)) {
        USBSerial.printf("WAV: ERROR writing placeholder header (%u/44)\n",
                         (unsigned)n);
        fclose(gFile);
        gFile = nullptr;
        return;
    }

    /* Make the existence of the file visible even if decoding later fails. */
    fflush(gFile);
    USBSerial.println("WAV: 44-byte placeholder committed");
}

void mp3PcmCaptureFrame(const int16_t *left,
                        const int16_t *right,
                        unsigned samples,
                        unsigned channels,
                        unsigned sampleRate)
{
    if (!gFile || !left || samples == 0 || channels == 0 || gFinished)
        return;

    if (gRate == 0) {
        gRate = sampleRate;
        gChannels = channels >= 2 ? 2 : 1;
        USBSerial.printf("WAV: first PCM frame: rate=%u channels=%u samples=%u\n",
                         gRate, gChannels, samples);
    }

    if (gRate != sampleRate)
        return;

    const uint64_t maxFrames = (uint64_t)gRate * MP3_PCM_CAPTURE_SECONDS;
    if (gFrames >= maxFrames) {
        gFinished = true;
        return;
    }

    uint64_t allowed64 = maxFrames - gFrames;
    unsigned allowed = allowed64 < samples ? (unsigned)allowed64 : samples;

    /* Write small chunks so the decoder does not need a large temporary RAM buffer. */
    uint8_t buf[512];
    size_t pos = 0;

    while (pos < allowed) {
        size_t chunkSamples = (allowed - pos);
        size_t maxSamples = sizeof(buf) / (gChannels * sizeof(int16_t));
        if (chunkSamples > maxSamples)
            chunkSamples = maxSamples;

        size_t out = 0;
        for (size_t i = 0; i < chunkSamples; ++i) {
            int16_t l = left[pos + i];
            int16_t r = (gChannels == 2 && right) ? right[pos + i] : l;

            buf[out++] = (uint8_t)(l & 0xFF);
            buf[out++] = (uint8_t)((uint16_t)l >> 8);

            if (gChannels == 2) {
                buf[out++] = (uint8_t)(r & 0xFF);
                buf[out++] = (uint8_t)((uint16_t)r >> 8);
            }
        }

        if (fwrite(buf, 1, out, gFile) != out) {
            USBSerial.println("WAV: ERROR while writing PCM");
            gFinished = true;
            return;
        }

        gDataBytes += out;
        gFrames += chunkSamples;
        pos += chunkSamples;

        /* Keep the file visible on flash while the decoder is running. */
        fflush(gFile);
    }

    if (gFrames >= maxFrames)
        gFinished = true;
}

bool mp3PcmCaptureIsFinished(void)
{
    return gFinished || (gFrames >= (uint64_t)gRate * MP3_PCM_CAPTURE_SECONDS);
}

void mp3PcmCaptureFinish(void)
{
    if (!gFile)
        return;

    fflush(gFile);

    if (gRate && gChannels) {
        if (!writeWavHeader(gFile, gRate, gChannels, (uint32_t)gDataBytes))
            USBSerial.println("WAV: ERROR finalizing header");
    }

    fflush(gFile);
    fclose(gFile);
    gFile = nullptr;

    USBSerial.printf("WAV samples  : %llu\n", (unsigned long long)gFrames);
    USBSerial.printf("WAV data     : %llu bytes\n", (unsigned long long)gDataBytes);
    USBSerial.printf("WAV duration : %.3f sec\n",
                     gRate ? (double)gFrames / (double)gRate : 0.0);
    USBSerial.println("WAV capture finished: /storage/test.wav");
    USBSerial.println("========================================");
}

#else

void mp3PcmCaptureReset(void) {}
void mp3PcmCaptureFrame(const int16_t *, const int16_t *, unsigned, unsigned, unsigned) {}
void mp3PcmCaptureFinish(void) {}
bool mp3PcmCaptureIsFinished(void) { return true; }

#endif
