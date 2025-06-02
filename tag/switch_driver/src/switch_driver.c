/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee switch driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "switch_driver.h"

#include "esp_timer.h"

/**
 * @brief:
 * This example code shows how to configure light switch with attribute as well as button switch handler.
 *
 * @note:
   Currently only support toggle switch functionality is available
 *
 * @note:
 * For other possible switch functions (on/off,level up/down,step up/down). User need to implement and create them by themselves
 */

static QueueHandle_t gpio_evt_queue = NULL;
/* button function pair, should be defined in switch example source file */
static switch_func_pair_t *switch_func_pair;
/* call back function pointer */
static esp_switch_callback_t func_ptr;
/* which button is pressed */
static uint8_t switch_num;
static const char *TAG = "ESP_ZB_SWITCH";

static void switch_driver_gpios_intr_enabled(bool enabled);

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    switch_driver_gpios_intr_enabled(false);
    xQueueSendFromISR(gpio_evt_queue, (switch_func_pair_t *)arg, NULL);
}

/**
 * @brief Enable GPIO (switches refer to) isr
 *
 * @param enabled      enable isr if true.
 */
static void switch_driver_gpios_intr_enabled(bool enabled)
{
    for (int i = 0; i < switch_num; ++i) {
        if (enabled) {
            gpio_intr_enable((switch_func_pair + i)->pin);
        } else {
            gpio_intr_disable((switch_func_pair + i)->pin);
        }
    }
}

/**
 * @brief Tasks for checking the button event and debounce the switch state
 *
 * @param arg      Unused value.
 */
// static void switch_driver_button_detected(void *arg)
// {
//     gpio_num_t io_num = GPIO_NUM_NC;
//     switch_func_pair_t button_func_pair;
//     static switch_state_t switch_state = SWITCH_IDLE;
//     bool evt_flag = false;

// 	//jylee-20241029
// 	int64_t press_start_time = 0;
// 	int64_t press_duration = 0;
// 	int64_t last_press_time = 0;
	
// 	const int64_t MIN_PRESS_THRESHOLD     = 400000;   // 0.3초
//     const int64_t MAX_PRESS_THRESHOLD     = 1000000;  // 1.0초
//     const int64_t LONG_PRESS_THRESHOLD    = 2000000;  // 2.0초
//     const int64_t DOUBLE_CLICK_THRESHOLD  = 300000;   // 더블클릭 간격(0.3초 이내)

// 	bool click_pending = false;
// 	bool is_double_click = false;

// 	for (;;)
// 	{
// 		/* check if there is any queue received, if yes read out the button_func_pair */
// 		if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY))
// 		{
// 			io_num = button_func_pair.pin;
// 			switch_driver_gpios_intr_enabled(false);
// 			evt_flag = true;
// 		}
// 		while (evt_flag)
// 		{
// 			bool value = gpio_get_level(io_num);
// 			switch (switch_state)
// 			{
// 			case SWITCH_IDLE:
// 				press_start_time = esp_timer_get_time();
// 				switch_state = (value == GPIO_INPUT_LEVEL_ON) ? SWITCH_PRESS_DETECTED : SWITCH_IDLE;
// 				press_duration = 0;
// 				break;
// 			case SWITCH_PRESS_DETECTED:
// 				press_duration = esp_timer_get_time() - press_start_time;
// 				switch_state = (value == GPIO_INPUT_LEVEL_ON) ? SWITCH_PRESS_DETECTED : SWITCH_RELEASE_DETECTED;
// 				break;
// 			case SWITCH_RELEASE_DETECTED:
// 				press_duration = esp_timer_get_time() - press_start_time;
// 				switch_state = SWITCH_IDLE;
// 				if (press_duration < MIN_PRESS_THRESHOLD)
//                 {
//                     //ignore the press event if the press duration is less than 1 sec
//                 }
// 				else if (press_duration >= LONG_PRESS_THRESHOLD)
// 				{
// 					button_func_pair.press_type = LONG_PRESS;
// 					printf("long\n");
// 					click_pending = false;

// 					(*func_ptr)(&button_func_pair);
// 				}
// 				else
// 				{
// 					if (press_duration < MAX_PRESS_THRESHOLD)
//                     {
// 						int64_t current_time = esp_timer_get_time();
// 						if (click_pending && current_time - last_press_time < DOUBLE_CLICK_THRESHOLD)
// 						{
// 							is_double_click = true;
// 							click_pending = false;
// 							button_func_pair.press_type = DOUBLE_CLICK;
// 							printf("double click\n");

// 							(*func_ptr)(&button_func_pair);
// 						}
// 						else
// 						{
// 							click_pending = true;
// 							is_double_click = false;
// 							last_press_time = current_time;
// 						}
// 					}
//                     else
//                     {
//                         // 1.0초 ~ 2.0초 사이는 아무 이벤트 없이 무시
//                     }
// 				}
				

// 				break;
// 			default:
// 				break;
// 			}

// 			if (click_pending)
// 			{
// 				int64_t current_time = esp_timer_get_time();
// 				if (current_time - last_press_time >= DOUBLE_CLICK_THRESHOLD)
// 				{
// 					button_func_pair.press_type = SHORT_PRESS;
// 					printf("short\n");
// 					click_pending = false;

// 					(*func_ptr)(&button_func_pair);
// 				}
// 				vTaskDelay(10 / portTICK_PERIOD_MS);
// 				continue;
// 			}

// 			if (switch_state == SWITCH_IDLE)
// 			{
// 				switch_driver_gpios_intr_enabled(true);
// 				evt_flag = false;
// 				break;
// 			}
// 			vTaskDelay(10 / portTICK_PERIOD_MS);
// 		}
// 	}
// }
static void switch_driver_button_detected(void *arg)
{
	gpio_num_t io_num = GPIO_NUM_NC;
	switch_func_pair_t button_func_pair;
	static switch_state_t switch_state = SWITCH_IDLE;
	bool evt_flag = false;

	const int64_t DOUBLE_CLICK_THRESHOLD = 300000; // 0.3초
	const int64_t SHORT_MIN_PRESS = 250000;		   // 0.5초 //jylee-20250415: 0.2초로 수정
	const int64_t SHORT_MAX_PRESS = 1500000;	   // 1.5초
	const int64_t LONG_PRESS_THRESHOLD = 1500000;  // 2.0초

	int64_t press_start_time = 0;
	int64_t press_duration = 0;
	int64_t last_release_time = 0;

	bool click_pending = false;
	bool is_double_click = false;
	bool first_click_was_short_under_05 = false;
	//   - true => 0.5초 미만
	//   - false => 0.5초 이상 ~ 1초 미만

	for (;;)
	{
		if (xQueueReceive(gpio_evt_queue, &button_func_pair, portMAX_DELAY))
		{
			io_num = button_func_pair.pin;
			// 처리 중 인터럽트 비활성화
			switch_driver_gpios_intr_enabled(false);
			evt_flag = true;
		}

		while (evt_flag)
		{
			bool value = gpio_get_level(io_num);

			switch (switch_state)
			{
			case SWITCH_IDLE:
				if (value == GPIO_INPUT_LEVEL_ON)
				{
					press_start_time = esp_timer_get_time();
					press_duration = 0;
					switch_state = SWITCH_PRESS_DETECTED;
				}
				else
				{
					switch_state = SWITCH_IDLE;
				}
				break;

			case SWITCH_PRESS_DETECTED:
				press_duration = esp_timer_get_time() - press_start_time;
				if (value != GPIO_INPUT_LEVEL_ON)
				{
					switch_state = SWITCH_RELEASE_DETECTED;
				}
				break;

			case SWITCH_RELEASE_DETECTED:
			{
				press_duration = esp_timer_get_time() - press_start_time;
				switch_state = SWITCH_IDLE;

				if (press_duration >= LONG_PRESS_THRESHOLD)
				{
					// 2초 이상 => Long click
					button_func_pair.press_type = LONG_PRESS;
					printf("[INFO] Long click\n");

					// 더블클릭 대기 해제
					click_pending = false;

					if (func_ptr)
						(*func_ptr)(&button_func_pair);
				}
				// else if (press_duration >= SHORT_MAX_PRESS)
				// {
				// 	// 1.0초 이상 ~ 2.0초 미만 => 무시
				// }
				else
				{
					// 0.5초 이상 ~ 1.0초 미만 OR 0.5초 미만
					int64_t current_time = esp_timer_get_time();

					if (press_duration < SHORT_MIN_PRESS)
					{
						if (click_pending &&
							(current_time - last_release_time < DOUBLE_CLICK_THRESHOLD))
						{
							// 두 번째 클릭이 0.5초 미만, 그리고 두 Release 간격이 0.3초 이내 => Double click
							is_double_click = true;
							click_pending = false;
							button_func_pair.press_type = DOUBLE_CLICK;
							printf("[INFO] Double click (<0.5s)\n");

							if (func_ptr)
								(*func_ptr)(&button_func_pair);
						}
						else
						{
							// 새롭게 "0.5초 미만 첫 클릭" 발생
							click_pending = true;
							is_double_click = false;
							first_click_was_short_under_05 = true;
							last_release_time = current_time;
							// => 두 번째 클릭을 기다린다
						}
					}
					else
					{
						// 0.5초 ~ 1.0초 미만
						if (click_pending &&
							(current_time - last_release_time < DOUBLE_CLICK_THRESHOLD))
						{
							// 이미 첫 클릭이 있었고, 더블클릭 타이밍 안 => Double click
							is_double_click = true;
							click_pending = false;
							button_func_pair.press_type = DOUBLE_CLICK;
							printf("[INFO] Double click\n");

							if (func_ptr)
								(*func_ptr)(&button_func_pair);
						}
						else
						{
							// 새롭게 "0.5~1.0초 첫 클릭" 발생
							click_pending = true;
							is_double_click = false;
							first_click_was_short_under_05 = false;
							last_release_time = current_time;
						}
					}
				}
				break;
			}
			default:
				break;
			}

			// 만약 click_pending == true 면,
			// 두 번째 클릭을 기다리는 중 (Double click 판단 대기)
			if (click_pending)
			{
				int64_t now = esp_timer_get_time();
				if ((now - last_release_time) >= DOUBLE_CLICK_THRESHOLD)
				{
					// Double click 불발
					// 첫 번째 클릭이 0.5초 미만이었다면 => 이벤트 없이 무시
					// 첫 번째 클릭이 0.5~1.0초였다면 => Single click 발생
					if (first_click_was_short_under_05)
					{
						// 0.5초 미만 클릭은 2번째가 없으면 이벤트 안 발생
						printf("[INFO] first click <0.5s => no event\n");
					}
					else
					{
						// 0.5~1.0초 첫 클릭은 single click 확정
						button_func_pair.press_type = SHORT_PRESS;
						printf("[INFO] Single click\n");

						if (func_ptr)
							(*func_ptr)(&button_func_pair);
					}

					click_pending = false;
				}

				vTaskDelay(pdMS_TO_TICKS(10));
				continue;
			}

			if (switch_state == SWITCH_IDLE)
			{
				switch_driver_gpios_intr_enabled(true);
				evt_flag = false;
				break;
			}

			vTaskDelay(pdMS_TO_TICKS(10));
		}
	}
}

/**
 * @brief init GPIO configuration as well as isr
 *
 * @param button_func_pair      pointer of the button pair.
 * @param button_num            number of button pair.
 */
static bool switch_driver_gpio_init(switch_func_pair_t *button_func_pair, uint8_t button_num)
{
	gpio_config_t io_conf = {};
    switch_func_pair = button_func_pair;
    switch_num = button_num;
    uint64_t pin_bit_mask = 0;

    /* set up button func pair pin mask */
    for (int i = 0; i < button_num; ++i) {
        pin_bit_mask |= (1ULL << (button_func_pair + i)->pin);
    }
    /* interrupt of falling edge */
	//jylee-20250102: One Button Control 관련 start 시 hangup 조치
    ///io_conf.intr_type = GPIO_INTR_LOW_LEVEL;
	io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = pin_bit_mask;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    /* configure GPIO with the given settings */
    gpio_config(&io_conf);
    /* create a queue to handle gpio event from isr */
    gpio_evt_queue = xQueueCreate(10, sizeof(switch_func_pair_t));
    if ( gpio_evt_queue == 0) {
        ESP_LOGE(TAG, "Queue was not created and must not be used");
        return false;
    }
    /* start gpio task */
    xTaskCreate(switch_driver_button_detected, "button_detected", 4096, NULL, 10, NULL);
	/* install gpio isr service */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    for (int i = 0; i < button_num; ++i) {
        gpio_isr_handler_add((button_func_pair + i)->pin, gpio_isr_handler, (void *) (button_func_pair + i));
    }
    return true;
}

bool switch_driver_init(switch_func_pair_t *button_func_pair, uint8_t button_num, esp_switch_callback_t cb)
{
    if (!switch_driver_gpio_init(button_func_pair, button_num)) {
        return false;
    }
    func_ptr = cb;
    return true;
}
