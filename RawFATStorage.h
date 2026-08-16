#ifndef RAW_FAT_STORAGE_H
#define RAW_FAT_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool rawFatMount(const char *basePath, const char *partitionLabel, int maxFiles);
bool rawFatUnmount(const char *basePath);

#ifdef __cplusplus
}
#endif

#endif
