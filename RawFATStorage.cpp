#include "RawFATStorage.h"

#include <Arduino.h>
#include <string.h>

extern "C" {
#include "ff.h"
#include "diskio.h"
#include "diskio_impl.h"
#include "esp_vfs_fat.h"
#include "esp_partition.h"
}

#define RAW_FAT_SECTOR_SIZE 512U
#define RAW_FAT_ERASE_SIZE  4096U

static const esp_partition_t *gPartition = nullptr;
static uint8_t gPdrv = 0xFF;
static FATFS *gFs = nullptr;
static char gDrive[3] = {0, ':', 0};
static char gBasePath[32] = {0};
static bool gMounted = false;

static DSTATUS rawInitialize(uint8_t pdrv)
{
    return (pdrv == gPdrv && gPartition) ? 0 : STA_NOINIT;
}

static DSTATUS rawStatus(uint8_t pdrv)
{
    return (pdrv == gPdrv && gPartition) ? 0 : STA_NOINIT;
}

static DRESULT rawRead(uint8_t pdrv, uint8_t *buffer, DWORD sector, UINT count)
{
    if (!gPartition || pdrv != gPdrv || !buffer || count == 0)
        return RES_PARERR;

    const uint64_t address = (uint64_t)sector * RAW_FAT_SECTOR_SIZE;
    const uint64_t bytes   = (uint64_t)count * RAW_FAT_SECTOR_SIZE;

    if (address + bytes > gPartition->size)
        return RES_PARERR;

    return esp_partition_read(gPartition, (size_t)address, buffer, (size_t)bytes) == ESP_OK
        ? RES_OK : RES_ERROR;
}

static DRESULT rawWrite(uint8_t pdrv, const uint8_t *buffer, DWORD sector, UINT count)
{
    if (!gPartition || pdrv != gPdrv || !buffer || count == 0)
        return RES_PARERR;

    const uint64_t address = (uint64_t)sector * RAW_FAT_SECTOR_SIZE;
    const uint64_t bytes   = (uint64_t)count * RAW_FAT_SECTOR_SIZE;

    if (address + bytes > gPartition->size)
        return RES_PARERR;

    // Do not place a 4 KB erase buffer on the VFS/FatFs task stack.
    // fopen()/fwrite() may enter this callback from a relatively small
    // task stack and the previous local buffer could cause stack pressure.
    static uint8_t eraseBuf[RAW_FAT_ERASE_SIZE];

    uint64_t pos = address;
    size_t srcOffset = 0;
    size_t remaining = (size_t)bytes;

    while (remaining) {
        const uint32_t eraseAddress = (uint32_t)(pos & ~(uint64_t)(RAW_FAT_ERASE_SIZE - 1));
        const uint32_t eraseOffset  = (uint32_t)(pos - eraseAddress);
        size_t chunk = RAW_FAT_ERASE_SIZE - eraseOffset;
        if (chunk > remaining)
            chunk = remaining;

        if (esp_partition_read(gPartition, eraseAddress, eraseBuf, RAW_FAT_ERASE_SIZE) != ESP_OK)
            return RES_ERROR;

        memcpy(eraseBuf + eraseOffset, buffer + srcOffset, chunk);

        if (esp_partition_erase_range(gPartition, eraseAddress, RAW_FAT_ERASE_SIZE) != ESP_OK)
            return RES_ERROR;

        if (esp_partition_write(gPartition, eraseAddress, eraseBuf, RAW_FAT_ERASE_SIZE) != ESP_OK)
            return RES_ERROR;

        pos += chunk;
        srcOffset += chunk;
        remaining -= chunk;
    }

    return RES_OK;
}

static DRESULT rawIoctl(uint8_t pdrv, BYTE cmd, void *buff)
{
    if (!gPartition || pdrv != gPdrv || !buff)
        return RES_PARERR;

    switch (cmd) {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_COUNT:
            *(DWORD *)buff = (DWORD)(gPartition->size / RAW_FAT_SECTOR_SIZE);
            return RES_OK;
        case GET_SECTOR_SIZE:
            *(WORD *)buff = RAW_FAT_SECTOR_SIZE;
            return RES_OK;
        case GET_BLOCK_SIZE:
            // FatFs reports erase block size in units of logical sectors.
            *(DWORD *)buff = RAW_FAT_ERASE_SIZE / RAW_FAT_SECTOR_SIZE;
            return RES_OK;
        default:
            return RES_PARERR;
    }
}

static const ff_diskio_impl_t gImpl = {
    .init = rawInitialize,
    .status = rawStatus,
    .read = rawRead,
    .write = rawWrite,
    .ioctl = rawIoctl
};

bool rawFatMount(const char *basePath, const char *partitionLabel, int maxFiles)
{
    if (gMounted)
        return true;

    if (!basePath || !partitionLabel)
        return false;

    gPartition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_DATA_FAT,
        partitionLabel
    );

    if (!gPartition) {
        Serial.printf("RAW FAT: partition '%s' not found\n", partitionLabel);
        return false;
    }

    esp_err_t err = ff_diskio_get_drive(&gPdrv);
    if (err != ESP_OK || gPdrv == 0xFF) {
        gPartition = nullptr;
        return false;
    }

    gDrive[0] = (char)('0' + gPdrv);
    gDrive[1] = ':';
    gDrive[2] = 0;

    ff_diskio_register(gPdrv, &gImpl);

    strncpy(gBasePath, basePath, sizeof(gBasePath) - 1);
    gBasePath[sizeof(gBasePath) - 1] = 0;

    err = esp_vfs_fat_register(gBasePath, gDrive, maxFiles, &gFs);
    if (err != ESP_OK) {
        ff_diskio_register(gPdrv, nullptr);
        gPdrv = 0xFF;
        gPartition = nullptr;
        gFs = nullptr;
        return false;
    }

    FRESULT fr = f_mount(gFs, gDrive, 1);
    if (fr != FR_OK) {
        esp_vfs_fat_unregister_path(gBasePath);
        ff_diskio_register(gPdrv, nullptr);
        gPdrv = 0xFF;
        gPartition = nullptr;
        gFs = nullptr;
        return false;
    }

    gMounted = true;
    return true;
}

bool rawFatUnmount(const char *basePath)
{
    (void)basePath;

    if (!gMounted)
        return true;

    f_mount(nullptr, gDrive, 0);

    esp_err_t err = esp_vfs_fat_unregister_path(gBasePath);

    ff_diskio_register(gPdrv, nullptr);

    gMounted = false;
    gFs = nullptr;
    gPartition = nullptr;
    gPdrv = 0xFF;
    gDrive[0] = 0;
    gDrive[1] = ':';
    gDrive[2] = 0;
    gBasePath[0] = 0;

    return err == ESP_OK;
}
