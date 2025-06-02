/* 
GPS_TAG_MAIN
*/
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

#include "esp_task_wdt.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "nvs_flash.h"

#include "zigbee.h"

#include "esp_zigbee_core.h"
#include "switch_driver.h"

#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif
#include "driver/rtc_io.h"

#include "zcl/esp_zigbee_zcl_common.h"
// #include "zcl/zb_zcl_custom_cluster.h"
#include "zboss_api.h"

#include "common.h"
#include "nvs_util.h"
#include "rpi_comm_task.h"

#include "zcl/esp_zigbee_zcl_power_config.h"
#include "sound.h"

#include "esp_timer.h"

#include "led_task.h"
#include "gps_task.h"

uint16_t pan_id = 0;
uint32_t channel_mask = 0;

//jylee-20240624: 9번핀 Wakeup
#define CONFIG_GPIO_INPUT_IO_WAKEUP 9

uint16_t test_counter = 0;
/* remote device struct for recording and managing node info */
light_bulb_device_params_t on_off_light;

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light switch (End Device) source code.
#endif

/**
 * This is an example which echos any data it receives on configured UART back to the sender,
 * with hardware flow control turned off. It does not use UART driver event queue.
 *
 * - Port: configured UART
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: off
 * - Pin assignment: see defines below (See Kconfig)
 */

static const char *TAG = "MODELE_GPS_TAG";

//jylee-20241125: led status
bool receiver_connected = false;
int64_t receiver_response_time = 0; 
bool gps_connected = false;
int64_t gps_receive_time = 0; 
//jylee-20250207: poweroff during charging
bool gps_activate = false;

//jylee-20250304: voltage data for send
extern float fVoltage_data;

TaskHandle_t xZBTaskHandle = NULL;
TaskHandle_t xUSBTaskHandle = NULL;
extern bool power_off_flag;
uint8_t double_click_count = 0;
uint8_t long_click_count = 0;

int64_t last_button_time = 0;
extern bool charging_status;

static void set_panid_from_nvs();

//jylee-20250304: add voltage data
void sendGPSData(float lat, float lon, float spd)
{
    float data[4];
    uint16_t short_addr = esp_zb_get_short_address();
    data[0] = lat; data[1] = lon; data[2] = spd; data[3] = fVoltage_data * 1.5; //data[4] = pre_fVoltage_data; 
	printf("voltage: %f\n", data[3]);
    // esp_zb_zcl_attribute_data_t attr_desc_2 = {ESP_ZB_ZCL_ATTR_TYPE_SINGLE, sizeof(uint32_t), &data[0]};
    // esp_zb_zcl_attribute_data_t attr_desc_3 = {ESP_ZB_ZCL_ATTR_TYPE_SINGLE, sizeof(uint32_t), &data[1]};
    // esp_zb_zcl_attribute_data_t attr_desc_4 = {ESP_ZB_ZCL_ATTR_TYPE_SINGLE, sizeof(uint32_t), &data[2]};
    // esp_zb_zcl_attribute_t attr_field[] = 
    // {
    //     {2, attr_desc_2},
    //     {3, attr_desc_3},
    //     {4, attr_desc_4},
    // };
    //ESP_ZB_ZCL_ATTR_TYPE_32BIT_ARRAY
    // 첫 번째 2바이트에 데이터 길이를 설정하고, 나머지 데이터 복사
    uint8_t data_with_length[1 + sizeof(uint16_t) + sizeof(data)];
    uint16_t data_length = sizeof(uint16_t) + sizeof(data);
    data_with_length[0] = data_length;
    memcpy(data_with_length + 1, &short_addr, sizeof(uint16_t));
    memcpy(data_with_length + 1 + sizeof(uint16_t), data, sizeof(data)); // 나머지에 데이터 복사

    // 데이터 타입 확인 및 초기화
    esp_zb_zcl_attribute_data_t attr_desc_2 = {ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, sizeof(data_with_length), data_with_length};

///    esp_zb_zcl_attribute_data_t attr_desc_2 = {ESP_ZB_ZCL_ATTR_TYPE_32BIT_ARRAY, 2 + sizeof(float) * 3, data};
    esp_zb_zcl_attribute_t attr_field[] = 
    {
        {2, attr_desc_2},
    };

    // esp_zb_zcl_attribute_t attr_field2[] = 
    // {
    //     {0, v1},
    //     {1, v2},
    // };


    // esp_zb_zcl_write_attr_cmd_t cmd_req = {
    //                         .zcl_basic_cmd = {
    //                             .dst_addr_u.addr_short = on_off_light.short_addr,
    //                             .dst_endpoint = on_off_light.endpoint,
    //                             .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
    //                         },
    //                         .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
    //                         .clusterID = 0xFC00,
    //                         .attr_number = 1,
    //                         .attr_field = attr_field,
    //                     };

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        {
            // zcl_basic_cmd (첫 번째 구조체)
            {on_off_light.short_addr}, // dst_addr_u.addr_short (중첩 구조체)
            on_off_light.endpoint,     // dst_endpoint
            HA_ONOFF_SWITCH_ENDPOINT   // src_endpoint
        },
        ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT, // address_mode
        0xFC00,                               // clusterID
        1,                                    // attr_number
        attr_field                            // attr_field
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to write attribute: %s", esp_err_to_name(err));
    // }
    esp_zb_lock_release();
}

//presstype : 1-short, 2-long
void sendRemoconEvent(uint8_t presstype)
{
    uint16_t data;
    uint16_t short_addr = esp_zb_get_short_address();
	data = presstype;

    uint8_t data_with_length[1 + sizeof(uint16_t) + sizeof(data)];
    uint16_t data_length = sizeof(uint16_t) + sizeof(data);
    data_with_length[0] = data_length;
    memcpy(data_with_length + 1, &short_addr, sizeof(uint16_t));
    memcpy(data_with_length + 1 + sizeof(uint16_t), &data, sizeof(data)); // 나머지에 데이터 복사

    // 데이터 타입 확인 및 초기화
    esp_zb_zcl_attribute_data_t attr_desc_2 = {ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, sizeof(data_with_length), data_with_length};

///    esp_zb_zcl_attribute_data_t attr_desc_2 = {ESP_ZB_ZCL_ATTR_TYPE_32BIT_ARRAY, 2 + sizeof(float) * 3, data};
    esp_zb_zcl_attribute_t attr_field[] = 
    {
        {0, attr_desc_2},
    };

    // esp_zb_zcl_write_attr_cmd_t cmd_req = {
    //                         .zcl_basic_cmd = {
    //                             .dst_addr_u.addr_short = on_off_light.short_addr,
    //                             .dst_endpoint = on_off_light.endpoint,
    //                             .src_endpoint = HA_ONOFF_SWITCH_ENDPOINT,
    //                         },
    //                         .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
    //                         .clusterID = 0xFCFF,
    //                         .attr_number = 1,
    //                         .attr_field = attr_field,
    //                     };

    esp_zb_zcl_write_attr_cmd_t cmd_req = {
        {
            // zcl_basic_cmd 구조체 초기화
            {on_off_light.short_addr}, // dst_addr_u.addr_short
            on_off_light.endpoint,     // dst_endpoint
            HA_ONOFF_SWITCH_ENDPOINT   // src_endpoint
        },
        ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT, // address_mode
        0xFCFF,                               // clusterID
        1,                                    // attr_number
        attr_field                            // attr_field
    };

    esp_zb_lock_acquire(portMAX_DELAY);
    esp_err_t err = esp_zb_zcl_write_attr_cmd_req(&cmd_req);
    // if (err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to write attribute: %s", esp_err_to_name(err));
    // }
    esp_zb_lock_release();
}

// org code (before c++ conversion)
// void sound_timer_set() {
//     ledc_timer_config_t ledc_timer = {
//         .duty_resolution = LEDC_TIMER_13_BIT,
//         .freq_hz = LEDC_FREQUENCY,
//         .speed_mode = LEDC_MODE,
//         .timer_num = LEDC_TIMER,
//         .clk_cfg = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&ledc_timer);

//     ledc_channel_config_t ledc_channel = {
//         .channel    = LEDC_CHANNEL,
//         .duty       = 0,
//         .gpio_num   = SOUND_GPIO,
//         .speed_mode = LEDC_MODE,
//         .hpoint     = 0,
//         .timer_sel  = LEDC_TIMER,
//         .intr_type  = LEDC_INTR_DISABLE,
//     };
//     ledc_channel_config(&ledc_channel);
// }

// c++ conversion
// void sound_timer_set() {
//     // ledc_timer_config_t 구조체 초기화 (필드 순서 맞춰야 함)
//     ledc_timer_config_t ledc_timer = {
//         LEDC_MODE,          // speed_mode (첫 번째 필드)
//         LEDC_TIMER_13_BIT,  // duty_resolution
//         LEDC_TIMER,         // timer_num
//         LEDC_FREQUENCY,     // freq_hz
//         LEDC_AUTO_CLK,      // clk_cfg
//         false               // deconfigure (bool 타입)
//     };
//     ledc_timer_config(&ledc_timer);

//     // ledc_channel_config_t 구조체 초기화 (필드 순서대로)
//     ledc_channel_config_t ledc_channel = {
//         SOUND_GPIO,         // gpio_num (첫 번째 필드)
//         LEDC_MODE,          // speed_mode
//         LEDC_CHANNEL,       // channel
//         LEDC_INTR_DISABLE,  // intr_type
//         LEDC_TIMER,         // timer_sel
//         0,                  // duty
//         0,                  // hpoint
// 		(ledc_sleep_mode_t)0, //sleep_mode
//         { 0 }               // flags (output_invert = 0)
//     };
//     ledc_channel_config(&ledc_channel);
// }

// org code (before c++ conversion)
// void led_timer_set() {
//     // LEDC 타이머 구성
//     ledc_timer_config_t ledc_timer = {
//         .speed_mode = LEDC_MODE_LED,
//         .timer_num = LEDC_TIMER_LED,
//         .duty_resolution = LEDC_DUTY_RES_LED,
//         .freq_hz = LEDC_FREQUENCY_LED,
//         .clk_cfg = LEDC_AUTO_CLK
//     };
//     ledc_timer_config(&ledc_timer);

//     // LEDC 채널 설정
//     ledc_channel_config_t ledc_channel = {
//         .speed_mode = LEDC_MODE_LED,
//         .channel = LEDC_CHANNEL_LED,
//         .timer_sel = LEDC_TIMER_LED,
//         .intr_type = LEDC_INTR_FADE_END,
//         .gpio_num = LED_PIN,
//         .duty = 0,  // 처음에는 0% duty (LED OFF)
//         .hpoint = 0
//     };
//     ledc_channel_config(&ledc_channel);
// }

// c++ conversion
// void led_timer_set() {
//     // LEDC 타이머 구성 (구조체 필드 순서에 맞게 초기화)
//     ledc_timer_config_t ledc_timer = {
//         LEDC_MODE_LED,         // speed_mode (첫 번째 필드)
//         LEDC_DUTY_RES_LED,     // duty_resolution
//         LEDC_TIMER_LED,        // timer_num
//         LEDC_FREQUENCY_LED,    // freq_hz
//         LEDC_AUTO_CLK,         // clk_cfg
//         false                  // deconfigure (bool 타입)
//     };
//     ledc_timer_config(&ledc_timer);

//     // LEDC 채널 설정 (구조체 필드 순서에 맞게 초기화)
//     ledc_channel_config_t ledc_channel = {
//         LED_PIN,               // gpio_num (첫 번째 필드)
//         LEDC_MODE_LED,         // speed_mode
//         LEDC_CHANNEL_LED,      // channel
//         LEDC_INTR_FADE_END,    // intr_type
//         LEDC_TIMER_LED,        // timer_sel
//         0,                     // duty (0%로 시작)
//         0,                     // hpoint
// 		(ledc_sleep_mode_t)0, //sleep_mode
//         { 0 }                  // flags (output_invert = 0)
//     };
//     ledc_channel_config(&ledc_channel);
// }

static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
	if (power_off_flag == true)
	{
		return;
	}

	//jylee-20250402: ignore button event within 1 sec
	if (last_button_time > 0 && esp_timer_get_time() - last_button_time < 1000000)
	{
		return;
	}
	last_button_time = esp_timer_get_time();

    switch (button_func_pair->func) {
    case SWITCH_ONOFF_TOGGLE_CONTROL: {
		if (button_func_pair->press_type == SHORT_PRESS)
		{
			long_click_count = double_click_count = 0;
		}
		else if (button_func_pair->press_type == DOUBLE_CLICK)
		{
			double_click_count++;
			long_click_count = 0;
		}
		else if (button_func_pair->press_type == LONG_PRESS)
		{
			long_click_count++;
			if (double_click_count < 2)
			{
				double_click_count = 0;
			}

			if (double_click_count >= 2 && long_click_count >= 2)
			{
				long_click_count = double_click_count = 0;
				printf("INIT......\n");
				cd_nvs_write_panid(0);
				usleep(100000);
				esp_zb_factory_reset();
			}
		}

        if (esp_zb_bdb_dev_joined())
        {
            /* send on-off toggle command to remote device */
            esp_zb_zcl_on_off_cmd_t cmd_req;
            cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = on_off_light.short_addr;
            cmd_req.zcl_basic_cmd.dst_endpoint = on_off_light.endpoint;
            cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
            cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_on_off_cmd_req(&cmd_req);
            esp_zb_lock_release();
            ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command to address(0x%x) endpoint(%d)", on_off_light.short_addr, on_off_light.endpoint);

			if (button_func_pair->press_type == SHORT_PRESS)
			{
				//sendRemoconEvent(1); //short press: 1
				sendGPSData(999, 999, 1);
			}
			else if (button_func_pair->press_type == DOUBLE_CLICK)
			{
				//sendRemoconEvent(2); //double click: 2
				//sendGPSData(999, 999, 2);
				//jylee-20250409: ignore double click
				//sendGPSData(999, 999, 1); //jylee-20250401: short press로 변경
			}
			else if (button_func_pair->press_type == LONG_PRESS)
			{
				//sendRemoconEvent(3); //long press: 3
				sendGPSData(999, 999, 3);
			}
        }
        else
        {
        /*    
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            vTaskDelay(1000 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage

            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

            playSong();
        */
			if (button_func_pair->press_type == SHORT_PRESS)
			{
				//jylee-20240530: NVS Read
				set_panid_from_nvs();

				// Zigbee 네트워크를 초기화
				led_all_off();
				esp_zb_factory_reset();
				ESP_LOGI(TAG, "Zigbee factory reset");
			}
        }
    } break;
    default:
		ESP_LOGI(TAG, "button");
        break;
    }
}

static esp_err_t deferred_driver_init(void)
{
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler), ESP_FAIL, TAG,
                        "Failed to initialize switch driver");

    /* Configure RTC IO wake up:
    The configuration mode depends on your hardware design.
    Since the BOOT button is connected to a pull-up resistor, the wake-up mode is configured as LOW.
    */
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(
        1ULL << CONFIG_GPIO_INPUT_IO_WAKEUP, ESP_EXT1_WAKEUP_ANY_LOW));

#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
    rtc_gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
    rtc_gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#else
    gpio_pulldown_dis((gpio_num_t)CONFIG_GPIO_INPUT_IO_WAKEUP);
    gpio_pullup_en((gpio_num_t)CONFIG_GPIO_INPUT_IO_WAKEUP);
#endif
	return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bind response from address(0x%x), endpoint(%d) with status(%d)", ((zdo_info_user_ctx_t *)user_ctx)->short_addr,
                 ((zdo_info_user_ctx_t *)user_ctx)->endpoint, zdo_status);
        /* configure report attribute command */
        esp_zb_zcl_config_report_cmd_t report_cmd;
        bool report_change = 0;
        report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = on_off_light.short_addr;
        report_cmd.zcl_basic_cmd.dst_endpoint = on_off_light.endpoint;
        report_cmd.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        report_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;

        esp_zb_zcl_config_report_record_t records[] = {
            {ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, ESP_ZB_ZCL_ATTR_TYPE_BOOL, 0, 30, &report_change}};
        report_cmd.record_number = sizeof(records) / sizeof(esp_zb_zcl_config_report_record_t);
        report_cmd.record_field = records;

        esp_zb_zcl_config_report_cmd_req(&report_cmd);
    }
}

static void ieee_cb(esp_zb_zdp_status_t zdo_status, esp_zb_ieee_addr_t ieee_addr, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        memcpy(&(on_off_light.ieee_addr), ieee_addr, sizeof(esp_zb_ieee_addr_t));
        ESP_LOGI(TAG, "IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
                 ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
                 ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
        /* bind the on-off light to on-off switch */
        esp_zb_zdo_bind_req_param_t bind_req;
        memcpy(&(bind_req.src_address), on_off_light.ieee_addr, sizeof(esp_zb_ieee_addr_t));
        bind_req.src_endp = on_off_light.endpoint;
        bind_req.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
        bind_req.dst_endp = HA_ONOFF_SWITCH_ENDPOINT;
        bind_req.req_dst_addr = on_off_light.short_addr;
        static zdo_info_user_ctx_t test_info_ctx;
        test_info_ctx.endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        test_info_ctx.short_addr = on_off_light.short_addr;
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, (void *) & (test_info_ctx));
    }
}

static void ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t *ep_id_list, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Active endpoint response: status(%d) and endpoint count(%d)", zdo_status, ep_count);
        for (int i = 0; i < ep_count; i++) {
            ESP_LOGI(TAG, "Endpoint ID List: %d", ep_id_list[i]);
        }
    }
}

static void simple_desc_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t *simple_desc, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Simple desc response: status(%d), device_id(%d), app_version(%d), profile_id(0x%x), endpoint_ID(%d)", zdo_status,
                 simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);

        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            ESP_LOGI(TAG, "Cluster ID list: 0x%x", *(simple_desc->app_cluster_list + i));
        }
    }
}

static void user_find_cb(esp_zb_zdp_status_t zdo_status, uint16_t addr, uint8_t endpoint, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Match desc response: status(%d), address(0x%x), endpoint(%d)", zdo_status, addr, endpoint);
        /* save into remote device record structure for future use */
        on_off_light.endpoint = endpoint;
        on_off_light.short_addr = addr;
        /* find the active endpoint */
        esp_zb_zdo_active_ep_req_param_t active_ep_req;
        active_ep_req.addr_of_interest = on_off_light.short_addr;
        esp_zb_zdo_active_ep_req(&active_ep_req, ep_cb, NULL);
        /* get the node simple descriptor */
        esp_zb_zdo_simple_desc_req_param_t simple_desc_req;
        simple_desc_req.addr_of_interest = addr;
        simple_desc_req.endpoint = endpoint;
        esp_zb_zdo_simple_desc_req(&simple_desc_req, simple_desc_cb, NULL);
        /* get the light ieee address */
        esp_zb_zdo_ieee_addr_req_param_t ieee_req;
        ieee_req.addr_of_interest = on_off_light.short_addr;
        ieee_req.dst_nwk_addr = on_off_light.short_addr;
        ieee_req.request_type = 0;
        ieee_req.start_index = 0;
        esp_zb_zdo_ieee_addr_req(&ieee_req, ieee_cb, NULL);
        esp_zb_zcl_read_attr_cmd_t read_req;
        uint16_t attributes[] = {ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID};
        read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        read_req.attr_number = sizeof(attributes) / sizeof(uint16_t);
        read_req.attr_field = attributes;
        read_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
        read_req.zcl_basic_cmd.dst_endpoint = on_off_light.endpoint;
        read_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
        read_req.zcl_basic_cmd.dst_addr_u.addr_short = on_off_light.short_addr;
        esp_zb_zcl_read_attr_cmd_req(&read_req);
    }
}

static void set_steering_alarm(uint32_t time_ms){
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING);
    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, time_ms);
}
static void set_zigbee_init_alarm(uint32_t time_ms){
    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION);
    esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_INITIALIZATION, time_ms);
}

static void set_user_cb_init()
{
    /* device auto start successfully and on a formed network */
    ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
    esp_zb_ieee_addr_t extended_pan_id;
    esp_zb_get_extended_pan_id(extended_pan_id);
    ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
    esp_zb_zdo_match_desc_req_param_t  find_req;
    find_req.addr_of_interest = 0x0000;
    find_req.dst_nwk_addr = 0x0000;
    /* find the match on-off light device */
    esp_zb_zdo_find_on_off_light(&find_req, user_find_cb, NULL);
}

static void set_panid_from_nvs()
{
    //jylee-20240530: NVS Read
    pan_id = 0;
    pan_id = cd_nvs_read_panid();
    if (pan_id == 0)
    {
        pan_id = DEFAULT_PANID; // for test///
    }
	//jylee-20250325: split channel by pan_id
	//jylee-20250326: except channel 18
	u_int8_t _id = (pan_id % 15);
	u_int8_t _id2 = (pan_id % 15) + 1;
	if (_id >= 7) _id++;
	if (_id > 15) _id = 15;
	if (_id2 >= 7) _id2++;
	if (_id2 > 15) _id2 = 15;
	if (_id == 15) _id2 = 0;
	channel_mask = (1l << (11 + _id)) | (1l << (11 + _id2));

    //jylee-20240527: for test
    esp_zb_set_pan_id(pan_id);
    esp_zb_set_channel_mask(channel_mask);

	// uint8_t ext_pan_id[] = CUSTOM_EXT_PAN_ID;
	// esp_zb_set_extended_pan_id(ext_pan_id);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
	static int signal_count = 0;        // 시그널 카운트
    static int64_t last_time = 0;       // 마지막 체크 시간(us 단위)
    const int64_t threshold_interval_us = 1000000; // 1초(1000000us)
    const int signal_threshold = 8;

	static int error_count = 0;

    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = (esp_zb_app_signal_type_t)*p_sg_p;
    esp_zb_zdo_signal_leave_params_t *leave_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee signal: SKIP_STARTUP");
        set_zigbee_init_alarm(1500);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        ESP_LOGI(TAG, "Zigbee signal: DEVICE_FIRST_START");

        set_panid_from_nvs();

        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Zigbee stack initialized (FIRST START)");
            set_user_cb_init();
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack FIRST_START (status: %d)", err_status);
            set_zigbee_init_alarm(1500);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        //jylee-20240528: for test
        ESP_LOGI(TAG, "Zigbee signal: DEVICE_REBOOT");

        set_panid_from_nvs();

        if (esp_zb_bdb_dev_joined() == false)
        {
            ESP_LOGW(TAG, "Device is not joined to network.");
			error_count++;

			// if (error_count > 2)
			// {
			// 	// Zigbee 네트워크를 초기화
			// 	led_all_off();
            //     esp_zb_factory_reset();
            //     ESP_LOGI(TAG, "Zigbee factory reset");
			// 	esp_restart();
			// }
        }

        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Zigbee stack initialized (REBOOT)");

            set_user_cb_init();
        }
        else
        {
            ESP_LOGW(TAG, "1 Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            // esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack DEVICE_REBOOT (status: %d)", err_status);
            set_zigbee_init_alarm(1500);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        ESP_LOGI(TAG, "Zigbee signal: STEERING");

        if (err_status != ESP_OK) {
            ESP_LOGW(TAG, "2 Stack %s failure with %s status, steering",esp_zb_zdo_signal_to_string(sig_type), esp_err_to_name(err_status));
            ///esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);

            set_steering_alarm(1500);

            //계속 접속 안될 시 리셋
            test_counter++;
            if (test_counter > 10)
            {
                // Zigbee 네트워크를 초기화
				led_all_off();
                esp_zb_factory_reset();
                ESP_LOGI(TAG, "Zigbee factory reset");
				esp_restart();
            }
        } else {
            ESP_LOGI(TAG, "Zigbee stack initialized (STEERING)");

            set_user_cb_init();
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Zigbee signal: LEAVE");
        leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Reset device");
        }
        break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        ///ESP_LOGI(TAG, "Zigbee can sleep");
        esp_zb_sleep_now();
        break;
    default:
		if (charging_status == false)
		{
            int64_t now = esp_timer_get_time(); // 현재 시간 (us)
            if (last_time == 0) {
                last_time = now;
            }
			if (sig_type == 0x32 || sig_type == 0x3c)
			{
            	signal_count++;
			}

            // 1초 경과 여부 체크
            if (now - last_time >= threshold_interval_us) {
                // 1초 경과
                if (signal_count > signal_threshold) {
                    ESP_LOGE(TAG, "Signal count exceeded threshold (%d > %d). System resetting...",
                             signal_count, signal_threshold);

					gpio_set_level((gpio_num_t)PWR_LED_GPIO, 0);
					vTaskDelay(2000 / portTICK_PERIOD_MS);
					
                    //esp_restart();
					esp_zb_factory_reset();
                }
                // 카운터/시간 초기화
                signal_count = 0;
                last_time = now;
            }

            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
                     esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
            break;
        }
        break;
    }
}

static esp_err_t zb_attribute_reporting_handler(const esp_zb_zcl_report_attr_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGI(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x)", message->src_address.u.short_addr,
             message->src_endpoint, message->dst_endpoint, message->cluster);
    ESP_LOGI(TAG, "Received report information: attribute(0x%x), type(0x%x), value(%d)\n", message->attribute.id, message->attribute.data.type,
             message->attribute.data.value ? *(uint8_t *)message->attribute.data.value : 0);
    return ESP_OK;
}

static esp_err_t zb_read_attr_resp_handler(const esp_zb_zcl_cmd_read_attr_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_read_attr_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Read attribute response: status(%d), cluster(0x%x), attribute(0x%x), type(0x%x), value(%d)", variable->status,
                 message->info.cluster, variable->attribute.id, variable->attribute.data.type,
                 variable->attribute.data.value ? *(uint8_t *)variable->attribute.data.value : 0);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_configure_report_resp_handler(const esp_zb_zcl_cmd_config_report_resp_message_t *message)
{
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);

    esp_zb_zcl_config_report_resp_variable_t *variable = message->variables;
    while (variable) {
        ESP_LOGI(TAG, "Configure report response: status(%d), cluster(0x%x), attribute(0x%x)", message->info.status, message->info.cluster,
                 variable->attribute_id);
        variable = variable->next;
    }

    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID:
        ret = zb_read_attr_resp_handler((esp_zb_zcl_cmd_read_attr_resp_message_t *)message);
        break;
    case ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID:
        ret = zb_configure_report_resp_handler((esp_zb_zcl_cmd_config_report_resp_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }

	//jylee-20241125
	receiver_connected = true;
	receiver_response_time = esp_timer_get_time();

	return ret;
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

// // 사용자 정의 네트워크 파라미터
// #define CUSTOM_PAN_ID 0x4C49
// #define CUSTOM_EXT_PAN_ID {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67}

// // Zigbee 네트워크 초기화
// void initialize_zigbee_network(void)
// {
//     esp_zb_set_pan_id(CUSTOM_PAN_ID);
//     esp_zb_set_channel_mask(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);

//     uint8_t ext_pan_id[] = CUSTOM_EXT_PAN_ID;
//     esp_zb_set_extended_pan_id(ext_pan_id);

//     esp_zb_start(true);
// }

void zigbee_force_reset_every_boot()
{
    nvs_handle_t nvs;
    uint8_t reset_pending = 0;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_get_u8(nvs, "reset_pending", &reset_pending);
    if (err == ESP_ERR_NVS_NOT_FOUND || reset_pending == 0) {
        //reset 요청 → 플래그 저장 후 재부팅
        ESP_LOGW(TAG, "Zigbee NVRAM reset requested, triggering factory reset...");
        nvs_set_u8(nvs, "reset_pending", 1);
        nvs_commit(nvs);
        nvs_close(nvs);

        esp_zb_factory_reset();  //여기서 재부팅 발생
    }
    else {
        //재부팅 이후: 플래그 초기화
        ESP_LOGI(TAG, "Factory reset completed. Proceeding with normal boot.");
        nvs_set_u8(nvs, "reset_pending", 0);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static void esp_zb_task(void *pvParameters)
{
	esp_zb_nvram_erase_at_start(true);

    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();

    /* Enable zigbee light sleep */
    esp_zb_sleep_enable(true);

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
	ESP_ERROR_CHECK(esp_zb_platform_config(&config));
	
    esp_zb_init(&zb_nwk_cfg);
	//jylee-20250325: reset zb connection
	zigbee_force_reset_every_boot();

    uint8_t test_attr;
    test_attr = 0;
    /* basic cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr);
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);
    
    /* create client role of the cluster */
    esp_zb_attribute_list_t *esp_zb_on_off_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    // esp_zb_on_off_cluster_add_attr(esp_zb_on_off_client_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &test_attr);
    // esp_zb_cluster_update_attr(esp_zb_on_off_client_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &test_attr);

    esp_zb_attribute_list_t *esp_zb_identify_client_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_client_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    
    int16_t value = 0;
    uint64_t value2 = 0;
    float array_value[4];
	uint8_t data2[3];
    esp_zb_attribute_list_t *esp_zb_custom_cluster = esp_zb_zcl_attr_list_create(0xFC00);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 0, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,&value);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 1, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,&value2);
    // esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 2, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,&array_value[0]);
    // esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 3, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,&array_value[1]);
    // esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 4, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,&array_value[2]);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 2, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, array_value);
    ///esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    
	//User Attribute - remote control event
    esp_zb_attribute_list_t *esp_zb_custom_cluster2 = esp_zb_zcl_attr_list_create(0xFCFF);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster2, 0, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, data2);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_custom_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ONOFF_SWITCH_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

	//jylee-20250326: set channel mask
	set_panid_from_nvs();
	//jylee-20250325: split channel by pan_id
	//jylee-20250326: except channel 18
	u_int8_t _id = (pan_id % 15);
	u_int8_t _id2 = (pan_id % 15) + 1;
	if (_id >= 7) _id++;
	if (_id > 15) _id = 15;
	if (_id2 >= 7) _id2++;
	if (_id2 > 15) _id2 = 15;
	if (_id == 15) _id2 = 0;
	channel_mask = (1l << (11 + _id)) | (1l << (11 + _id2));

    esp_zb_set_primary_network_channel_set(channel_mask);
    esp_zb_set_secondary_network_channel_set(channel_mask);
    
    //jylee-20240530: button cb
    ESP_LOGI(TAG, "button cb initialization %s", deferred_driver_init() ? "failed" : "successful");        

    ESP_ERROR_CHECK(esp_zb_start(true));
    
    //initialize_zigbee_network();

    esp_zb_main_loop_iteration();
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
	
	//set_panid_from_nvs();

    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BAT_LED_GPIO) | (1ULL << PWR_LED_GPIO);
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

	led_init();

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ADC_GPIO); // ADC1_CHANNEL_3 --> VIN
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

	//jylee-20241113: power control
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_IN_PWR_CNT_GPIO);
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

	// sound_timer_set();
	// led_timer_set();

	// jylee-20241217: one button control (Main Button)
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = (1ULL << GPIO_IN_MAIN_BUTTON);
	io_conf.pull_down_en = (gpio_pulldown_t)0;
	io_conf.pull_up_en = (gpio_pullup_t)0;
	gpio_config(&io_conf);

	// jylee-20241217: one button control (Power Control)
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUT_POWER_CNTL);
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

	// jylee-20250408: Power Control Backup
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_OUT_POWER_CNTL2);
    io_conf.pull_down_en = (gpio_pulldown_t)0;
    io_conf.pull_up_en = (gpio_pullup_t)0;
    gpio_config(&io_conf);

	//jylee-20250217: watchdog (5sec)
	esp_task_wdt_config_t wdt_cfg = {
		.timeout_ms = 5 * 1000,                     // 5초
		.idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  // 모든 코어의 Idle Task 감시
		.trigger_panic = true,                      // 타임아웃 시 panic 발생(결국 리셋)
	};
	ESP_ERROR_CHECK(esp_task_wdt_init(&wdt_cfg));
	//ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
	//esp_task_wdt_reset();

	int level = 0;
	int64_t start_time = esp_timer_get_time();
#if ENABLE_SW_POWER_CONTROL
	while (true)
	{
		//jylee-20241219: reboot 상태
		if (gpio_get_level(GPIO_OUT_POWER_CNTL) == 1)
		{
			printf("GPIO power on.....\n");

			// vTaskDelay(pdMS_TO_TICKS(3000));
			// esp_restart();

			break;
		}

		int level = gpio_get_level(GPIO_IN_MAIN_BUTTON);
		// printf("GPIO %d 상태: %d\n", GPIO_INPUT_MAIN_BUTTON, level);
		if (level == 0 && esp_timer_get_time() - start_time > 3000000) // 3sec
		{
			printf("power on...\n");
			gpio_set_level(GPIO_OUT_POWER_CNTL, 1);
			break;
		}
		else if (level == 1)
		{
			start_time = esp_timer_get_time();
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);

	}
	printf("started...\n");
#endif
	xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, &xZBTaskHandle);

    xTaskCreate(gps_task, "uart_gps_task", 8192, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(led_task, "status_led_task", 4096, NULL, tskIDLE_PRIORITY, NULL);

    xTaskCreate(usb_serial_task, "usb_serial_task", 4096, NULL, 10, &xUSBTaskHandle);

	//jylee-20241218: power off
	start_time = 0;
	while (true)
	{
		int level = gpio_get_level((gpio_num_t)GPIO_IN_MAIN_BUTTON);
		// printf("GPIO %d 상태: %d\n", GPIO_INPUT_MAIN_BUTTON, level);
		if (level == 0 && start_time != 0 && esp_timer_get_time() - start_time > 3000000) // 3sec
		{
			printf("power off...\n");
#if ENABLE_SW_POWER_CONTROL
			led_all_off();
			gpio_set_level(GPIO_OUT_POWER_CNTL, 0);
#endif
			break;
		}
		else if (level == 1)
		{
			start_time = esp_timer_get_time();
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
