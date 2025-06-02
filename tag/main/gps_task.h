#ifndef GPS_TASK_H
#define GPS_TASK_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <string.h>

#include "common.h"

void gps_standby();
void gps_wakeup();

void gps_task(void *arg);

#endif