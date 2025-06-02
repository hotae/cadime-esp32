#include "led_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_sleep.h"

#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "gps_task.h"

float fVoltage_data = 0;
float pre_fVoltage_data = 0;
float fVoltage_tmp = 0;

extern bool receiver_connected;
extern int64_t receiver_response_time; 
extern bool gps_connected;
extern int64_t gps_receive_time; 
extern bool gps_activate;
extern int64_t panid_response_time;

int64_t charging_start_time = 0;
int64_t system_start_time = 0;

extern const char *TAG;

bool led_off_flag = false;

extern TaskHandle_t xZBTaskHandle;
extern TaskHandle_t xUSBTaskHandle;
bool power_off_flag = false;
bool charging_status = false;

#define MA_VOL_SIZE 10
float ma_vol[MA_VOL_SIZE] = { 0 };
uint8_t ma_vol_index = 0;
uint8_t ma_vol_count = 0;

float ma_filter(float* buf, uint8_t* index, uint8_t* count, int ma_size, float new_value, bool is_reset) {
	float sum = 0;

	if (is_reset) {
		for (uint8_t i = 0; i < ma_size; i++) {
			buf[i] = 0;
		}
		*count = *index = 0;
		return 0;
	}

	// add new value to buffer
	buf[*index] = new_value;
	(*count)++;
	if (*count > ma_size) *count = ma_size;

	// calculate sum of buffer
	for (uint8_t i = 0; i < ma_size; i++) {
		sum += buf[i];
	}

	// update index, wrapping around if necessary
	(*index)++;
	if (*index >= ma_size) {
		*index = 0;
	}

	// calculate and return average
	///return sum / FILTER_SIZE;
	return sum / *count;
}

void led_task(void *arg)
{
    uint32_t voltage = 0;
    // int8_t led_status_cnt = 0;
    // int8_t charge_led_status_cnt = 0;
    // int8_t network_disconnect_cnt = 0;
    int raw = 0;
    uint8_t max = 0;
    // bool bat_charge_status = false, bat_full_status = false;
    // bool bat = false, pwr = false;

    // gpio 3
    esp_log_level_set("gpio", ESP_LOG_ERROR);
    adc_oneshot_unit_init_cfg_t init_config1 = {};
    adc_oneshot_unit_handle_t adc1_handle;
    init_config1.unit_id = ADC_UNIT_1;

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
    
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_1, ADC_ATTEN_DB_12, &adc1_cali_chan0_handle);
    
    // ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 3583);
    // ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

	bool led_blinking_onoff = false; //jylee-20241125

	bool prev_charging_status = false;

	gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
	gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);

#ifdef POWER_CONTROL_20S
	int64_t power_cnt_time = esp_timer_get_time();
#endif
	//receiver_response_time = 0;

	ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

	//jylee-20250320
	system_start_time = esp_timer_get_time();
	bool check_first_connection = false;
	int16_t max_retry = 0;
	int16_t max_retry2 = 0;

	while (1)
	{
#ifdef POWER_CONTROL_20S
		//jylee-20241113: power control;
		if (esp_timer_get_time() - power_cnt_time > 20000000) // 20 sec
		{
			printf("Power Control...\n");
			gpio_set_level(PWR_CNT_GPIO, 1);

			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(PWR_CNT_GPIO, 0);
			power_cnt_time = esp_timer_get_time();
		}
#endif
		//jylee-20250207: power off during charging
		if (gpio_get_level((gpio_num_t)GPIO_IN_PWR_CNT_GPIO) == 1)
		{
			charging_status = true;
		}
		else
		{
			charging_status = false;
		}
		if (prev_charging_status != charging_status)
		{
			prev_charging_status = charging_status;
			if (charging_status == true)
			{
				charging_start_time = esp_timer_get_time();
			}
			else
			{
				charging_start_time = 0;
				panid_response_time = 0;	
			}
		}

		if (charging_status == true)
		{
			printf("Charging...\n");
			if (power_off_flag == false)
			{
				if (charging_start_time > 0 && esp_timer_get_time() - charging_start_time > POWEROFF_DURING_CHARGING_SEC * 1000000)
				{
					if (gps_activate)
					{
						printf("GPS activating...\n");

						printf("Power off...\n");
						gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL, 1);
						gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL2, 1);
						vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
						esp_task_wdt_reset();
						vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
						esp_task_wdt_reset();
						vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
						esp_task_wdt_reset();
						vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
						esp_task_wdt_reset();
						vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
						esp_task_wdt_reset();
						gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL, 0);
						gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL2, 0);
						printf("Done Power off...\n");
					}
					if (xZBTaskHandle != NULL)
					{
						printf("Delete ZB Task...\n");
						vTaskDelete(xZBTaskHandle);
						xZBTaskHandle = NULL;
					}
					if (xUSBTaskHandle != NULL)
					{
						printf("Delete USB Task...\n");
						vTaskDelete(xUSBTaskHandle);
						xUSBTaskHandle = NULL;
					}
					power_off_flag = true;
				}
			}
		}
		else
		{
			// jylee-20241125: check zb receiver connected
			if (receiver_response_time > 0)
			{
				if (esp_timer_get_time() - receiver_response_time > 5000000) // 5 sec
				{
					receiver_connected = false;

					// jylee-20250123: reboot if over 20 sec
					if (esp_timer_get_time() - receiver_response_time > 20000000)
					{
						ESP_LOGI(TAG, "Data receive timeout...");
						esp_restart();
					}
				}
			}
			//jylee-20250320: check first connection timeout (20 sec)
			else
			{
				if (system_start_time > 0 && esp_timer_get_time() - system_start_time > 20000000
					&& check_first_connection == false) // 20 sec
				{
					check_first_connection = true;
					if (receiver_connected == false)
					{
						ESP_LOGI(TAG, "First connection timeout...");
						esp_zb_factory_reset();
						esp_restart();
					}
				}
			}
		}

        do
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_1, &raw));
        } while ((raw == 0) && max++ < 10);
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, raw, (int*)&voltage));
		max = 0;
        fVoltage_tmp = (float)voltage/1000;
		fVoltage_data = ma_filter(ma_vol, &ma_vol_index, &ma_vol_count, MA_VOL_SIZE, fVoltage_tmp, false);
		printf("vol: %f, %f\n", fVoltage_data, fVoltage_data * 1.5);

        if(pre_fVoltage_data == 0) {
            pre_fVoltage_data = fVoltage_data;
        }

		// jylee-20250219: power off when battery voltage is lower than 3.2V (factor 1.5)
		if (fVoltage_data < 2.133 && power_off_flag == false)
		{
			//jylee=20250324: max_retry2 60 (1 min)
			if (max_retry2 <= 0)
			{
				// printf("Power off...\n");
				gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL, 1);
				gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL2, 1);
				vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
				esp_task_wdt_reset();
				vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
				esp_task_wdt_reset();
				vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
				esp_task_wdt_reset();
				vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
				esp_task_wdt_reset();
				vTaskDelay(2 * 1000 / portTICK_PERIOD_MS); // 10 sec
				esp_task_wdt_reset();
				gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL, 0);
				gpio_set_level((gpio_num_t)GPIO_OUT_POWER_CNTL2, 0);
				printf("Done Power off...\n");

				max_retry2 = 61;
			}

			max_retry2--;
		}

		//jylee-20250102: if led_off flag set, just off led and continue
		if (led_off_flag)
		{
			gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
			gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);
			
			vTaskDelay(100 / portTICK_PERIOD_MS);
			esp_task_wdt_reset();
			continue;
		}

        //printf("fdata %f, pre %f charge %d full %d\n",fVoltage_data,pre_fVoltage_data,bat_charge_status, bat_full_status);
		led_blinking_onoff = !led_blinking_onoff;
		//jylee-20241125
		if (power_off_flag == false)
		{
			if (gps_activate == false)
			{
				if (led_blinking_onoff == true)
				{
					gpio_set_level((gpio_num_t)BAT_LED_GPIO, 0);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
					vTaskDelay(100 / portTICK_PERIOD_MS);
					gpio_set_level((gpio_num_t)BAT_LED_GPIO, 0);
				}
				else
				{
					gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
				}
			}
			else
			{
				//if (fVoltage_data < 2.267)
				if (fVoltage_data < 2.347) //jylee-20250219: 3.4V -> 3.52V
				{
					if (led_blinking_onoff == true)
					{
						gpio_set_level((gpio_num_t)BAT_LED_GPIO, 0);
					}
					else
					{
						gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
					}
				}
				else
				{
					gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
				}
			}

			if (receiver_connected && gps_connected)
			{
				gpio_set_level((gpio_num_t)PWR_LED_GPIO, 0);
			}
			else
			{
				if (led_blinking_onoff == true)
				{
					if (!receiver_connected)
					{
						gpio_set_level((gpio_num_t)PWR_LED_GPIO, 0);
						vTaskDelay(100 / portTICK_PERIOD_MS);
						gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);
						vTaskDelay(100 / portTICK_PERIOD_MS);
						gpio_set_level((gpio_num_t)PWR_LED_GPIO, 0);
					}
					else
					{
						gpio_set_level((gpio_num_t)PWR_LED_GPIO, 0);
					}
				}
				else
				{
					gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);
				}
			}
		}
		else
		{
			gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
			gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);
		}

        // test
        // fVoltage_data = 20 * VOLTAGE_RATIO;
        // if(fVoltage_data < 25 * VOLTAGE_RATIO) {
        //     gpio_set_level(PWR_LED_GPIO, 1);
        //     gpio_set_level(BAT_LED_GPIO, 1);
        //     vTaskDelay(300 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage
        //     gpio_set_level(BAT_LED_GPIO, 0);
        // } else {
        //     if(bat_full_status == true) {
        //         gpio_set_level(BAT_LED_GPIO, 1);
        //         gpio_set_level(PWR_LED_GPIO, 0);
        //     } else {
        //         gpio_set_level(BAT_LED_GPIO, 0);
        //         gpio_set_level(PWR_LED_GPIO, 1);
        //     }
        // }

        // if((fVoltage_data - pre_fVoltage_data > 0.04) && (bat_charge_status == false)) {
        //     // printf("bat charge status true!!!\n");
        //     bat_charge_status = true;
        // }

        // if(((pre_fVoltage_data - fVoltage_data) > 0.01) && (bat_charge_status == true)) {
        //     // printf("bat charge status false!!!\n");
        //     bat_charge_status = false;
        //     bat_full_status = false;
        // }

        // if(bat_charge_status == true) {
        //      if(fVoltage_data > FULL_CHARGE) {
        //         // printf("full charge!!!!!\n");
        //         bat_full_status = true;
        //     } else {
        //         // printf("bat charge start!!!!\n");
        //         gpio_set_level(BAT_LED_GPIO, 1);
        //         gpio_set_level(PWR_LED_GPIO, 1);
        //         vTaskDelay(300 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage
        //         gpio_set_level(PWR_LED_GPIO, 0);
        //     }
        // }

        pre_fVoltage_data = fVoltage_data;
		esp_task_wdt_reset();
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage
	}
	ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    example_adc_calibration_deinit(adc1_cali_chan0_handle);
}

void led_all_off()
{
	led_off_flag = true;
	gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
	gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);
}

void led_init()
{
	gpio_set_level((gpio_num_t)BAT_LED_GPIO, 1);
	gpio_set_level((gpio_num_t)PWR_LED_GPIO, 1);
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
