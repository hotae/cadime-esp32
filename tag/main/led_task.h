#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "common.h"

void led_task(void *arg);
void led_all_off();
void led_init();
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

#endif