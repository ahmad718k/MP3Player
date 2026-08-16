#include "MP3PWMOutput.h"

#include <Arduino.h>
#include "USB.h"

extern USBCDC USBSerial;

#include <esp_timer.h>

#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef MP3_PWM_PIN
#define MP3_PWM_PIN 16
#endif

#define MP3_PWM_CARRIER_HZ 312500UL
#define MP3_PWM_BITS       8U

/*
 * Audio ring buffer:
 *
 * The MP3 decoder runs in the normal Arduino task.
 * A hardware timer consumes exactly one PWM duty sample per audio tick.
 *
 * We store the already converted 8-bit duty value rather than int16 PCM.
 * 8192 samples = about 186 ms at 44.1 kHz.
 */
#define AUDIO_RING_SIZE 8192U

static uint8_t gAudioRing[AUDIO_RING_SIZE];
static volatile uint32_t gRingRead = 0;
static volatile uint32_t gRingWrite = 0;
static volatile uint32_t gUnderruns = 0;

static hw_timer_t *gAudioTimer = nullptr;
static portMUX_TYPE gAudioMux = portMUX_INITIALIZER_UNLOCKED;

static bool gPwmAttached = false;
static bool gStreamStarted = false;
static unsigned gStreamRate = 0;

static bool pwmAttach(void)
{
    if (gPwmAttached)
        return true;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)

    /*
     * 80 MHz APB / 256 = 312500 Hz exactly.
     * This is the configuration already proven to work on this board.
     */
    if (!ledcSetClockSource(LEDC_USE_APB_CLK)) {
        USBSerial.println("ERROR: LEDC APB clock selection failed");
        return false;
    }

    gPwmAttached = ledcAttach(
        MP3_PWM_PIN,
        MP3_PWM_CARRIER_HZ,
        MP3_PWM_BITS);

    if (gPwmAttached)
        ledcWrite(MP3_PWM_PIN, 128);

#else

    static const uint8_t channel = 0;

    ledcSetup(
        channel,
        MP3_PWM_CARRIER_HZ,
        MP3_PWM_BITS);

    ledcAttachPin(MP3_PWM_PIN, channel);
    ledcWrite(channel, 128);

    gPwmAttached = true;

#endif

    if (!gPwmAttached) {
        USBSerial.println("ERROR: LEDC attach failed");
        return false;
    }

    USBSerial.printf(
        "LEDC OK: GPIO=%u carrier=%lu Hz bits=%u\n",
        (unsigned)MP3_PWM_PIN,
        (unsigned long)MP3_PWM_CARRIER_HZ,
        (unsigned)MP3_PWM_BITS);

    return true;
}

static inline void pwmWrite8ISR(uint8_t duty)
{
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(MP3_PWM_PIN, duty);
#else
    ledcWrite(0, duty);
#endif
}

/*
 * One audio sample per hardware-timer tick.
 *
 * Arduino-ESP32 3.x uses:
 *     timerBegin(frequency)
 *     timerAttachInterrupt(timer, callback)
 *     timerAlarm(timer, value, autoreload, reload_count)
 *
 * The callback has the required void (*)() signature.
 */
static void IRAM_ATTR audioTimerISR(void)
{
    uint8_t duty = 128;

    portENTER_CRITICAL_ISR(&gAudioMux);

    if (gRingRead != gRingWrite) {
        duty = gAudioRing[gRingRead];
        gRingRead = (gRingRead + 1) & (AUDIO_RING_SIZE - 1);
    } else {
        ++gUnderruns;
        duty = 128;
    }

    portEXIT_CRITICAL_ISR(&gAudioMux);

    pwmWrite8ISR(duty);
}

static uint32_t queuedSamplesUnsafe(void)
{
    return (gRingWrite - gRingRead) & (AUDIO_RING_SIZE - 1);
}

unsigned mp3PwmQueuedSamples(void)
{
    portENTER_CRITICAL(&gAudioMux);
    uint32_t n = queuedSamplesUnsafe();
    portEXIT_CRITICAL(&gAudioMux);
    return (unsigned)n;
}

unsigned long mp3PwmUnderruns(void)
{
    portENTER_CRITICAL(&gAudioMux);
    uint32_t n = gUnderruns;
    portEXIT_CRITICAL(&gAudioMux);
    return (unsigned long)n;
}

static uint8_t pcmToDuty(int32_t pcm)
{
    int32_t duty = (pcm + 32768) >> 8;

    if (duty < 0)
        duty = 0;
    else if (duty > 255)
        duty = 255;

    return (uint8_t)duty;
}

void mp3PwmQueueStereo(const int16_t *left,
                       const int16_t *right,
                       unsigned samples,
                       unsigned sampleRate)
{
    if (!left || !right || samples == 0 || sampleRate == 0)
        return;

    if (!pwmAttach())
        return;

    /*
     * The producer blocks only when the ring is full.
     * The hardware timer continues consuming samples while we wait.
     */
    for (unsigned i = 0; i < samples; ++i) {

        int32_t mix =
            ((int32_t)left[i] + (int32_t)right[i]) / 2;

        uint8_t duty = pcmToDuty(mix);

        for (;;) {
            portENTER_CRITICAL(&gAudioMux);

            uint32_t next =
                (gRingWrite + 1) & (AUDIO_RING_SIZE - 1);

            bool full = (next == gRingRead);

            if (!full) {
                gAudioRing[gRingWrite] = duty;
                gRingWrite = next;
                portEXIT_CRITICAL(&gAudioMux);
                break;
            }

            portEXIT_CRITICAL(&gAudioMux);

            /*
             * Ring full: let the consumer drain some samples.
             */
            delay(1);
        }
    }

    (void)sampleRate;
}

bool mp3PwmStartStream(unsigned sampleRate)
{
    if (sampleRate == 0)
        return false;

    if (!pwmAttach())
        return false;

    if (gStreamStarted)
        return true;

    /*
     * Request a timer frequency equal to the MP3 sample rate.
     *
     * For 44.1 kHz the ESP32 hardware divider is close enough that the
     * resulting sample-rate error is tiny, while the audio PWM carrier
     * remains independently fixed at 312.5 kHz.
     */
    gAudioTimer = timerBegin(sampleRate);

    if (!gAudioTimer) {
        USBSerial.println("ERROR: audio timerBegin failed");
        return false;
    }

    timerAttachInterrupt(gAudioTimer, &audioTimerISR);
    timerAlarm(gAudioTimer, 1, true, 0);

    gStreamRate = sampleRate;
    gStreamStarted = true;

    USBSerial.printf(
        "AUDIO TIMER STARTED: requested=%u Hz queued=%u\n",
        sampleRate,
        mp3PwmQueuedSamples());

    return true;
}

void mp3PwmStop(void)
{
    if (gAudioTimer) {
        timerAlarm(gAudioTimer, 0, false, 0);
        timerDetachInterrupt(gAudioTimer);
        timerEnd(gAudioTimer);
        gAudioTimer = nullptr;
    }

    gStreamStarted = false;
    gStreamRate = 0;

    portENTER_CRITICAL(&gAudioMux);
    gRingRead = 0;
    gRingWrite = 0;
    gUnderruns = 0;
    portEXIT_CRITICAL(&gAudioMux);

    if (gPwmAttached)
        pwmWrite8ISR(128);
}

/*
 * Kept for source compatibility with the previous diagnostic stage.
 * In Stage16, repeats are intentionally ignored: the continuous decoder
 * uses mp3PwmQueueStereo() + mp3PwmStartStream() instead.
 */
void mp3PwmPlayStereo(const int16_t *left,
                      const int16_t *right,
                      unsigned samples,
                      unsigned sampleRate,
                      unsigned repeats)
{
    if (!left || !right || samples == 0 ||
        sampleRate == 0 || repeats == 0)
        return;

    for (unsigned r = 0; r < repeats; ++r)
        mp3PwmQueueStereo(left, right, samples, sampleRate);

    if (!gStreamStarted)
        mp3PwmStartStream(sampleRate);
}
