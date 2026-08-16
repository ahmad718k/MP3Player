// STAGE21 BUILD MARKER: IMDCT normalization active (2/18, 2/6)
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <esp_partition.h>

extern "C" {
#include "mp3_parser.h"
#include "MP3Decoder.h"
}

#include "USB.h"
#include "USBHID.h"
#include "USBHIDKeyboard.h"
#include "USBHIDMouse.h"
#include "USBHIDGamepad.h"
#include "USBHIDConsumerControl.h"
#include "USBHIDSystemControl.h"
#include "USBHIDVendor.h"
#include "USBMSC.h"
#include "RawFATStorage.h"

#if !ARDUINO_USB_CDC_ON_BOOT
USBCDC USBSerial;
#endif

USBHID HID;
USBHIDKeyboard Keyboard;
USBHIDMouse Mouse;
USBHIDGamepad Gamepad;
USBHIDConsumerControl ConsumerControl;
USBHIDSystemControl SystemControl;
USBHIDVendor Vendor;
USBMSC MSC;

#define STORAGE_LABEL       "storage"
#define STORAGE_PATH       "/storage"
#define MSC_SECTOR_SIZE    512U
#define FLASH_SECTOR_SIZE  4096U

static const esp_partition_t *storagePartition = nullptr;
static uint32_t storageSize = 0;
static uint32_t sectorCount = 0;
static uint8_t flashSectorBuffer[FLASH_SECTOR_SIZE];

static volatile bool storageEjected = false;
static bool fatMounted = false;

static void usbEventCallback(void *arg,
                             esp_event_base_t event_base,
                             int32_t event_id,
                             void *event_data)
{
    (void)arg;

    if (event_base == ARDUINO_USB_EVENTS) {
        switch (event_id) {
            case ARDUINO_USB_STARTED_EVENT:
                USBSerial.println("USB STARTED");
                break;
            case ARDUINO_USB_STOPPED_EVENT:
                USBSerial.println("USB STOPPED");
                break;
            case ARDUINO_USB_SUSPEND_EVENT:
                USBSerial.println("USB SUSPENDED");
                break;
            case ARDUINO_USB_RESUME_EVENT:
                USBSerial.println("USB RESUMED");
                break;
            default:
                break;
        }
    }
    else if (event_base == ARDUINO_USB_CDC_EVENTS) {
        arduino_usb_cdc_event_data_t *data =
            (arduino_usb_cdc_event_data_t *)event_data;

        switch (event_id) {
            case ARDUINO_USB_CDC_CONNECTED_EVENT:
                USBSerial.println("CDC CONNECTED");
                break;
            case ARDUINO_USB_CDC_DISCONNECTED_EVENT:
                USBSerial.println("CDC DISCONNECTED");
                break;
            case ARDUINO_USB_CDC_LINE_STATE_EVENT:
                USBSerial.printf("CDC LINE STATE: DTR=%u RTS=%u\n",
                                 data->line_state.dtr,
                                 data->line_state.rts);
                break;
            default:
                break;
        }
    }
}

static bool initStorage()
{
    storagePartition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_FAT,
        STORAGE_LABEL
    );

    if (!storagePartition) {
        USBSerial.println("ERROR: storage partition NOT FOUND");
        return false;
    }

    storageSize = storagePartition->size;
    sectorCount = storageSize / MSC_SECTOR_SIZE;

    USBSerial.printf("Storage address : 0x%08X\n",
                     (unsigned)storagePartition->address);
    USBSerial.printf("Storage size    : %u bytes\n",
                     (unsigned)storageSize);
    USBSerial.printf("MSC sectors     : %u\n",
                     (unsigned)sectorCount);

    return true;
}

/*
 * MSC accesses the same raw FAT sectors used by RawFATStorage.
 * This keeps the Windows-visible filesystem identical to the filesystem
 * mounted by FatFs after Windows has ejected the drive.
 */
static int32_t mscRead(uint32_t lba,
                       uint32_t offset,
                       void *buffer,
                       uint32_t bufsize)
{
    if (!storagePartition || !buffer)
        return -1;

    uint64_t address = (uint64_t)lba * MSC_SECTOR_SIZE + offset;
    if (address + bufsize > storageSize)
        return -1;

    return esp_partition_read(storagePartition,
                              (size_t)address,
                              buffer,
                              bufsize) == ESP_OK
        ? (int32_t)bufsize : -1;
}

static int32_t mscWrite(uint32_t lba,
                        uint32_t offset,
                        uint8_t *buffer,
                        uint32_t bufsize)
{
    if (!storagePartition || !buffer)
        return -1;

    uint64_t address = (uint64_t)lba * MSC_SECTOR_SIZE + offset;
    if (address + bufsize > storageSize)
        return -1;

    uint32_t remaining = bufsize;
    uint32_t srcOffset = 0;

    while (remaining) {
        uint32_t sectorAddress = (uint32_t)(address & ~(uint64_t)(FLASH_SECTOR_SIZE - 1));
        uint32_t sectorOffset = (uint32_t)(address - sectorAddress);
        uint32_t writeSize = FLASH_SECTOR_SIZE - sectorOffset;
        if (writeSize > remaining)
            writeSize = remaining;

        if (esp_partition_read(storagePartition,
                               sectorAddress,
                               flashSectorBuffer,
                               FLASH_SECTOR_SIZE) != ESP_OK)
            return -1;

        memcpy(flashSectorBuffer + sectorOffset,
               buffer + srcOffset,
               writeSize);

        if (esp_partition_erase_range(storagePartition,
                                      sectorAddress,
                                      FLASH_SECTOR_SIZE) != ESP_OK)
            return -1;

        if (esp_partition_write(storagePartition,
                                sectorAddress,
                                flashSectorBuffer,
                                FLASH_SECTOR_SIZE) != ESP_OK)
            return -1;

        address += writeSize;
        srcOffset += writeSize;
        remaining -= writeSize;
    }

    return (int32_t)bufsize;
}

static bool mscStartStop(uint8_t power_condition,
                         bool start,
                         bool load_eject)
{
    USBSerial.printf("MSC START/STOP: power=%u start=%u eject=%u\n",
                     power_condition, start, load_eject);

    if (load_eject && !start) {
        USBSerial.println("MSC EJECTED");
        storageEjected = true;
    }

    return true;
}

static void listStorage()
{
    FILE *f = fopen(STORAGE_PATH "/music.mp3", "rb");
    if (!f) {
        USBSerial.println("music.mp3: NOT FOUND");
        return;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    fclose(f);

    USBSerial.printf("music.mp3 size: %ld bytes\n", size);
}

static bool startFatRW()
{
    if (fatMounted)
        return true;

    if (!rawFatMount(STORAGE_PATH, STORAGE_LABEL, 8)) {
        USBSerial.println("ERROR: RAW FAT RW mount failed");
        return false;
    }

    fatMounted = true;

    USBSerial.println("FAT mounted READ/WRITE");
    return true;
}

static void stopFatRW()
{
    if (!fatMounted)
        return;

    rawFatUnmount(STORAGE_PATH);
    fatMounted = false;
    USBSerial.println("FAT unmounted");
}

void setup()
{
#if !ARDUINO_USB_CDC_ON_BOOT
    USBSerial.begin();
#endif

    USB.onEvent(usbEventCallback);
#if !ARDUINO_USB_CDC_ON_BOOT
    USBSerial.onEvent(usbEventCallback);
#endif

    HID.begin();
    Keyboard.begin();
    Mouse.begin();
    Gamepad.begin();
    ConsumerControl.begin();
    SystemControl.begin();
    Vendor.begin();

    if (!initStorage()) {
        while (true) delay(1000);
    }

    MSC.vendorID("ESP32");
    MSC.productID("ESP32-S2 FLASH");
    MSC.productRevision("1.0");
    MSC.onRead(mscRead);
    MSC.onWrite(mscWrite);
    MSC.onStartStop(mscStartStop);
    MSC.mediaPresent(true);
    MSC.isWritable(true);
    MSC.begin(sectorCount, MSC_SECTOR_SIZE);

    USB.begin();

    delay(1200);

    USBSerial.println();
    USBSerial.println("======================================");
    USBSerial.println("STAGE17 MP3 -> 5 SEC WAV TEST");
    USBSerial.println("ESP32-S2 / 4MB FLASH");
    USBSerial.println("CDC + HID + MSC");
    USBSerial.println("Windows-visible internal FAT");
    USBSerial.println("======================================");
    USBSerial.println("1) Put music.mp3 on the drive if needed.");
    USBSerial.println("2) Safely EJECT the drive in Windows.");
    USBSerial.println("3) ESP32 will decode and create /test.wav.");
    USBSerial.println("4) After completion the board resets.");
    USBSerial.println("5) Windows will show the updated test.wav.");
    USBSerial.println();
    USBSerial.println("Waiting for Windows EJECT...");
}

void loop()
{
    if (storageEjected) {
        storageEjected = false;

        USBSerial.println();
        USBSerial.println("======================================");
        USBSerial.println("MSC released by host");
        USBSerial.println("Mounting FAT read/write...");
        USBSerial.println("======================================");

        delay(500);

        if (!startFatRW())
            return;

        listStorage();

        FILE *file = fopen(STORAGE_PATH "/music.mp3", "rb");
        if (!file) {
            USBSerial.println("ERROR: cannot open music.mp3");
            stopFatRW();
            return;
        }

        USBSerial.println();
        USBSerial.println("Starting REAL MP3 decoder...");
        USBSerial.println("WAV capture target: 5 seconds");
        USBSerial.println();

        mp3DecoderTest(file);
        fclose(file);

        USBSerial.println();
        USBSerial.println("Closing FAT filesystem...");
        stopFatRW();

        USBSerial.println();
        USBSerial.println("======================================");
        USBSerial.println("TEST WAV READY: /test.wav");
        USBSerial.println("Resetting so Windows can remount MSC...");
        USBSerial.println("======================================");

        delay(1000);
        ESP.restart();
    }

    while (USBSerial.available()) {
        uint8_t buffer[64];
        size_t available = USBSerial.available();
        size_t len = available > sizeof(buffer) ? sizeof(buffer) : available;
        len = USBSerial.read(buffer, len);
        if (HID.ready())
            Vendor.write(buffer, len);
    }

    delay(1);
}
