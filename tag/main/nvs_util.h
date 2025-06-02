#ifndef __NVS_UTIL_H
#define __NVS_UTIL_H

#include <stdio.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"


#ifdef __cplusplus
extern "C" {
#endif

void cd_nvs_write_panid(uint16_t _pan_id);
uint16_t cd_nvs_read_panid();

#ifdef __cplusplus
}
#endif


#endif