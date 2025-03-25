/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee customized server Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>

#include "receiver_main.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "msg_struct.h"
#include "sdkconfig.h"
#include "esp_timer.h"

static const char *TAG = "MODELE_RECEIVER";

#include "nvs_util.h"

#define ECHO_TEST_TXD 4
#define ECHO_TEST_RXD 5
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      1
#define ECHO_UART_BAUD_RATE     115200
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

//jylee-20241211: XIAO ESP32
#define GPIO_PIN_EXT_ANT_1	14
#define GPIO_PIN_EXT_ANT_2	3
#define ENABLE_EXT_ANT		1
#define GPIO_PIN_LED		15

#define BUF_SIZE (1024)

#include "switch_driver.h"

// #define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  /* Zigbee primary channel mask */
// #define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   (1l << 11)  /* Zigbee primary channel mask use in the example */
#define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define DEFAULT_PANID     0x2000

//jylee-20241017: for jylee test
// #define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  /* Zigbee primary channel mask */
// #define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   (1l << 11)  /* Zigbee primary channel mask use in the example */
// //#define DEFAULT_PANID     0x9436
// #define DEFAULT_PANID     0x9437

//jylee-20241017: for yeongam45 test2 set
// #define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  /* Zigbee primary channel mask */
// #define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   (1l << 11)  /* Zigbee primary channel mask use in the example */
// #define DEFAULT_PANID     0x1236

// 사용자 정의 네트워크 파라미터
#define CUSTOM_PAN_ID 0x4C49
#define CUSTOM_EXT_PAN_ID {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67}

uint16_t pan_id = 0;
uint32_t channel_mask = 0;

static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};

//jylee-20240530: for test
void reset_network(void)
{
    // Zigbee 네트워크를 초기화
    esp_zb_factory_reset();
    esp_zb_zcl_reset_nvram_to_factory_default();
    ESP_LOGI(TAG, "Zigbee factory reset");

    // // Zigbee 네트워크 재시작
    // ESP_ERROR_CHECK(esp_zb_start());
    // ESP_LOGI(TAG, "Zigbee stack restarted successfully");
}

//jylee-20240529: for test
static void zb_buttons_handler(switch_func_pair_t *button_func_pair)
{
    switch (button_func_pair->func) {
    case SWITCH_ONOFF_TOGGLE_CONTROL: {

        reset_network();

        ESP_LOGI(TAG, "reset network");

    } break;
    default:
        break;
    }
}

//press_type - 1: short, 2: double click, 3: long
void sendRemoconEvent(uint16_t short_addr, uint16_t press_type)
{
	E_PROTOCOL_REMOCON eProtocol;

	ESP_LOGI(TAG, "Send remocon event to server - id: %d, type: %d", short_addr, press_type);

	eProtocol.e_Data.stx = E_STX;
	eProtocol.e_Data.op = 0x03; //Remocon
	eProtocol.e_Data.len = 7; //packet size
	eProtocol.e_Data.id = short_addr;		// ID
	eProtocol.e_Data.mode = press_type;
	eProtocol.e_Data.etx = E_ETX;

	// ESP_LOGI(TAG, "size: %d, %s", sizeof(eProtocol), eProtocol.buffer);
	// for (int i = 0; i < 15; i++)
	// {
	//     ESP_LOGI(TAG, "%x", eProtocol.buffer[i]);
	// }
	uart_write_bytes(ECHO_UART_PORT_NUM, eProtocol.buffer, sizeof(eProtocol));

	// uart_flush(ECHO_UART_PORT_NUM);
}

void sendPanIdRequest()
{
	E_PROTOCOL_PANID eProtocol;

	ESP_LOGI(TAG, "Send panid to server - panid: %d", pan_id);

	eProtocol.e_Data.stx = E_STX;
	eProtocol.e_Data.op = 0x11; //Remocon
	eProtocol.e_Data.len = 6; //packet size
	eProtocol.e_Data.panid = pan_id;
	eProtocol.e_Data.etx = E_ETX;

	ESP_LOGI(TAG, "size: %d, %s", sizeof(eProtocol), eProtocol.buffer);
	for (int i = 0; i < 6; i++)
	{
	    //ESP_LOGI(TAG, "%02x ", eProtocol.buffer[i]);
		printf("%02x ", eProtocol.buffer[i]);
	}
	printf("\n");

	uart_write_bytes(ECHO_UART_PORT_NUM, eProtocol.buffer, sizeof(eProtocol));

	// uart_flush(ECHO_UART_PORT_NUM);
}

static esp_err_t deferred_driver_init(void)
{
	//jylee-20241211: xiao esp32
    //light_driver_init(LIGHT_DEFAULT_OFF);
	gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_PIN_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), zb_buttons_handler), ESP_FAIL, TAG,
                        "Failed to initialize switch driver");

    return ESP_OK;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
	//jylee-20250226: for recover from connection abnormal
	static int64_t last_time = 0;       // 마지막 체크 시간(us 단위)
	static int signal_count = 0;        // 시그널 카운트
    const int64_t threshold_interval_us = 500000; // 0.5초(1000000us)
    const int signal_threshold = 120;

    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t *dev_annce_params = NULL;
    esp_zb_zdo_signal_leave_indication_params_t *leave_ind_params = NULL;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network formation");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                //jylee-20240530: NVS Read
                pan_id = cd_nvs_read_panid();
                if (pan_id == 0)
                {
                    pan_id = DEFAULT_PANID; // for test///
                }
				//jylee-20250325: split channel for pan_id
				//channel_mask = (1l << (11 + (pan_id % 16)));
                //jylee-20240527: for test
				esp_zb_set_primary_network_channel_set(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);
				esp_zb_set_channel_mask(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);
                esp_zb_set_pan_id(pan_id);

                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
                esp_zb_bdb_open_network(180);
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Formed network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE_INDICATION:
        leave_ind_params = (esp_zb_zdo_signal_leave_indication_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (!leave_ind_params->rejoin) {
            esp_zb_ieee_addr_t leave_addr;
            memcpy(leave_addr, leave_ind_params->device_addr, sizeof(esp_zb_ieee_addr_t));
            ESP_LOGI(TAG, "Zigbee Node is leaving network: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x)",
                     leave_addr[7], leave_addr[6], leave_addr[5], leave_addr[4],
                     leave_addr[3], leave_addr[2], leave_addr[1], leave_addr[0]);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
        if (err_status == ESP_OK) {
            if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
            } else {
                ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
            }
        }
        break;
    case ESP_ZB_NLME_STATUS_INDICATION: 
        {
            //ZDO signal: NLME Status Indication (0x32), status: ESP_OK
            printf("%s, status: 0x%x\n", esp_zb_zdo_signal_to_string(sig_type), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));

			//jylee-20250226: for recover from connection abnormal
			if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p) == 0x12)
			{
				int64_t now = esp_timer_get_time(); // 현재 시간 (us)
				if (last_time == 0) {
					last_time = now;
				}
				signal_count++;
				// 1초 경과 여부 체크
				// if (now - last_time >= threshold_interval_us)
				// {
					// 1초 경과
					if (signal_count >= signal_threshold)
					{
						ESP_LOGE(TAG, "Signal count exceeded threshold (%d > %d). System resetting...",
								 signal_count, signal_threshold);

						esp_zb_factory_reset();
					}
				//}
				if (now - last_time >= 1000000 * 60) //1 minute
				{
					// 카운터/시간 초기화
					signal_count = 0;
				}
				last_time = now;
			}

			esp_zb_zdo_permit_joining_req_param_t cmd_req;
            cmd_req.dst_nwk_addr = 0x0000;
            cmd_req.permit_duration = 180;
            cmd_req.tc_significance = 1;
            esp_zb_zdo_permit_joining_req(&cmd_req, NULL, NULL);
        } 
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
	esp_err_t ret = ESP_OK;
	bool light_state = 0;
	E_PROTOCOL_GPS eProtocol;
	ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
	ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
						message->info.status);
	ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
			 message->attribute.id, message->attribute.data.size);
	if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT)
	{
		if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
		{
			if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
			{
				light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
				ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");

				//jylee-20241211: xiao esp32
				//light_driver_set_power(light_state);
				if (light_state == true)
				{
					gpio_set_level(GPIO_PIN_LED, 0);
				}
				else
				{
					gpio_set_level(GPIO_PIN_LED, 1);
				}

			}
		}
		else if (message->info.cluster == 0xFC00)
		{
			if (message->attribute.id == 2 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING)
			{
				uint16_t short_addr = 0;
				float lat = 0, lon = 0, spd = 0, voltage = 0;
				short_addr = *(uint16_t *)(message->attribute.data.value + 1);
				lat = *(float *)(message->attribute.data.value + 3);
				lon = *(float *)(message->attribute.data.value + 7);
				spd = *(float *)(message->attribute.data.value + 11);
				voltage = *(float*)(message->attribute.data.value + 15); //jylee-20250304
				// pre_data = *(float*)(message->attribute.data.value + 19);
				// ESP_LOGI(TAG, "cluster:0x%x, attribute:0x%x changed ", message->info.cluster, message->attribute.id);
				// ESP_LOGI(TAG, "cluster:0x%x, attribute:0x%x, value:%.2f changed ", message->info.cluster, message->attribute.id,
				//     (float)*(float*)(message->attribute.data.value));
				ESP_LOGI(TAG, "1 cluster:0x%x, attribute:0x%x, addr:%d, lat:%f, lon: %f, spd: %f, voltage: %f", message->info.cluster, message->attribute.id,
						 short_addr, lat, lon, spd, voltage);
				// ESP_LOGI(TAG, "1 cluster:0x%x, attribute:0x%x, addr:%d, lat:%f, lon: %f, spd:%f voltage:%f pre:%f", message->info.cluster, message->attribute.id,
				//     short_addr, lat, lon, spd,voltage,pre_data);
				
				if (lat == 999 && lon == 999)
				{
					sendRemoconEvent(short_addr, spd); // 1: short, 2: double click
				}
				else
				{
					eProtocol.e_Data.stx = E_STX;
					eProtocol.e_Data.op = 0x02; // GPS
					eProtocol.e_Data.len = 24;	// packet size
					eProtocol.e_Data.id = short_addr;
					if (lat >= 0)
					{
						eProtocol.e_Data.northSouth = 'N';
					}
					else
					{
						eProtocol.e_Data.northSouth = 'S';
					}

					if (lon >= 0)
					{
						eProtocol.e_Data.eastWest = 'E';
					}
					else
					{
						eProtocol.e_Data.eastWest = 'W';
					}
					eProtocol.e_Data.lat = lat * 1000000;
					eProtocol.e_Data.lon = lon * 1000000;
					eProtocol.e_Data.spd = spd * 1000000;
					eProtocol.e_Data.voltage = voltage * 1000000;
					eProtocol.e_Data.etx = E_ETX;

					// ESP_LOGI(TAG, "size: %d, %s", sizeof(eProtocol), eProtocol.buffer);
					// for (int i = 0; i < 15; i++)
					// {
					//     ESP_LOGI(TAG, "%x", eProtocol.buffer[i]);
					// }
					uart_write_bytes(ECHO_UART_PORT_NUM, eProtocol.buffer, sizeof(eProtocol));

					///uart_flush(ECHO_UART_PORT_NUM);
				}
			}
		}
		else if (message->info.cluster == 0xFCFF) // remocon event
		{
			printf("1 %d, %d\n", message->attribute.id, message->attribute.data.type);
			if (message->attribute.id == 0 && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING)
			{
				uint16_t short_addr = 0;
				uint16_t press_type = 0;
				short_addr = *(uint16_t *)(message->attribute.data.value + 1);
				press_type = *(uint16_t *)(message->attribute.data.value + 3);
				ESP_LOGI(TAG, "2 cluster:0x%x, attribute:0x%x, addr:%d, press_type:%d", message->info.cluster, message->attribute.id,
						 short_addr, press_type);

				sendRemoconEvent(short_addr, press_type); // 1: short, 2: double click
			}
		}
	}
	else
	{
		ESP_LOGI(TAG, "cluster:0x%x, attribute:0x%x, value:%.2f changed ", message->info.cluster, message->attribute.id,
				 (float)*(float *)(message->attribute.data.value));
	}
	return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static esp_err_t attr_cb(const esp_zb_zcl_set_attr_value_message_t message)
{
    esp_err_t ret = ESP_OK;
    if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Received message: endpoint(0x%x), cluster(0x%x), attribute(0x%x), data size(%d)", message.info.dst_endpoint,
                 message.info.cluster, message.attribute.id, message.attribute.data.size);
    } else {
        ESP_LOGE(TAG, "Received message: status(%d) error", message.info.status);
        ret = ESP_ERR_INVALID_ARG;
    }
    return ret;
}

// Zigbee 네트워크 초기화
void initialize_zigbee_network(void)
{
    //jylee-20240530: NVS Read
    pan_id = cd_nvs_read_panid();
    if (pan_id == 0)
    {
        pan_id = DEFAULT_PANID; // for test///
    }

	esp_zb_set_channel_mask(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);
	esp_zb_set_primary_network_channel_set(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);
	//jylee-20240527: for test
    esp_zb_set_pan_id(pan_id);
    ///esp_zb_set_channel_mask(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);
	
	// uint8_t ext_pan_id[] = CUSTOM_EXT_PAN_ID;
	// esp_zb_set_extended_pan_id(ext_pan_id);

	esp_zb_start(true);
}

static void esp_zb_task(void *pvParameters)
{
	esp_zb_nvram_erase_at_start(true);
	esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    /* initialize Zigbee stack with Zigbee coordinator config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    uint8_t test_attr, test_attr2;
    test_attr = 0;
    test_attr2 = 3;
    /* basic cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &test_attr);
    esp_zb_cluster_update_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &test_attr2);
    /* identify cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(esp_zb_identify_cluster, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &test_attr);
    /* group cluster create with fully customized */
    esp_zb_attribute_list_t *esp_zb_groups_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_GROUPS);
    esp_zb_groups_cluster_add_attr(esp_zb_groups_cluster, ESP_ZB_ZCL_ATTR_GROUPS_NAME_SUPPORT_ID, &test_attr);
    /* scenes cluster create with standard cluster + customized */
    esp_zb_attribute_list_t *esp_zb_scenes_cluster = esp_zb_scenes_cluster_create(NULL);
    esp_zb_cluster_update_attr(esp_zb_scenes_cluster, ESP_ZB_ZCL_ATTR_SCENES_NAME_SUPPORT_ID, &test_attr);
    /* on-off cluster create with standard cluster config*/
    esp_zb_on_off_cluster_cfg_t on_off_cfg;
    on_off_cfg.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE;
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
    ///esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);
    // esp_zb_on_off_cluster_add_attr(esp_zb_on_off_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &test_attr);
    // esp_zb_cluster_update_attr(esp_zb_on_off_cluster, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &test_attr);

    /* create cluster lists for this endpoint */
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    /* update basic cluster in the existed cluster list */
    esp_zb_cluster_list_update_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_groups_cluster(esp_zb_cluster_list, esp_zb_groups_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_scenes_cluster(esp_zb_cluster_list, esp_zb_scenes_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    int16_t value = 0;
    uint64_t value2 = 0;
    float array_value[4];
	uint8_t data2[3];
	//User Attribute - GPS
    esp_zb_attribute_list_t *esp_zb_custom_cluster = esp_zb_zcl_attr_list_create(0xFC00);
    ///esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_custom_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 0, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,&value);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 1, ESP_ZB_ZCL_ATTR_TYPE_U32, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,&value2);
    // esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 2, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,&array_value[0]);
    // esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 3, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,&array_value[1]);
    // esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 4, ESP_ZB_ZCL_ATTR_TYPE_SINGLE, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,&array_value[2]);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster, 2, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, array_value);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

	//User Attribute - remote control event
    esp_zb_attribute_list_t *esp_zb_custom_cluster2 = esp_zb_zcl_attr_list_create(0xFCFF);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_custom_cluster2, 0, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, data2);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_custom_cluster2, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    /* add created endpoint (cluster_list) to endpoint list */
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);

    // esp_zb_device_add_set_attr_value_cb(attr_cb);
    // esp_zb_device_add_report_attr_cb(attr_cb);

    /////esp_zb_set_primary_network_channel_set(DEFAULT_ZB_PRIMARY_CHANNEL_MASK);
	
    //jylee-20240527
    initialize_zigbee_network();
    //ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_main_loop_iteration();
}

// static void test_task(void *pvParameters) {
//     int test = 0;
//   for (;;) {
//     vTaskDelay(5000 / portTICK_PERIOD_MS);
//     // light_driver_set_power(true);
//     test = 1;
//     esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &test, false);
//     vTaskDelay(5000 / portTICK_PERIOD_MS);
//     // light_driver_set_power(false);
//     test = 0;
//     esp_zb_zcl_set_attribute_val(10, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ON_OFF_ON_TIME, &test, false);
//   }
// }

#define PACKET_BUF_SIZE 255

void usb_serial_task(void *pvParameters)
{
	uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
	uint8_t *packet = (uint8_t *)malloc(BUF_SIZE);

	uint8_t bNeedSTX = true;
	uint8_t bNeedOP = true;
	uint8_t bNeedLength = true;
	unsigned int nReadSize = 0;
	unsigned int nReadHeader = 0;
	uint8_t nOP = 0;
	unsigned int nPacketLength = 0;

	uart_flush(ECHO_UART_PORT_NUM);
	// 0: STX, 1: OP, 2: LEN, 3: DATA, 4: ETX
	int64_t panid_send_time = esp_timer_get_time();
	while (1)
	{
		//jylee-20241106: for receive test...
		// for (int i = 0; i < 6; i++)
		// {
		// 	size_t buff_size;
		// 	int nBytes = uart_read_bytes(ECHO_UART_PORT_NUM, &data[i], 1, 10 / portTICK_PERIOD_MS);
		// 	if (nBytes > 0)
		// 	{
		// 		uart_get_buffered_data_len(ECHO_UART_PORT_NUM, &buff_size);
		// 		printf("buffer: %d", buff_size);
		// 		// 바이트 수신 성공 시, 다음 바이트 수신 대기
		// 		printf("read: %02x\n", data[i]);
		// 	}
		// 	// else
		// 	// {
		// 	// 	// 필요시 딜레이를 추가하거나 오류 로그 출력
		// 	// 	vTaskDelay(10 / portTICK_PERIOD_MS);
		// 	// }
		// }
		// vTaskDelay(10 / portTICK_PERIOD_MS);
		// continue;

		if (esp_timer_get_time() - panid_send_time > 60000000) //1 min
		{
			sendPanIdRequest();
			panid_send_time = esp_timer_get_time();
		}

		// Read data from USB-Serial-JTAG
		// Preamble Search...
		if (bNeedSTX == true)
		{
			// Search STX
			int nBytes = uart_read_bytes(ECHO_UART_PORT_NUM, data + nReadSize, 1, 50 / portTICK_PERIOD_MS);
			// int nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * 1, 100 / portTICK_PERIOD_MS);
			if (nBytes <= 0)
			{
				vTaskDelay(10 / portTICK_PERIOD_MS);
				continue;
			}
			nReadSize += nBytes;
			if (nReadSize == 1)
			{
				if (data[0] != 0x02)
				{
					nReadSize = 0;
					vTaskDelay(10 / portTICK_PERIOD_MS);
					continue;
				}
				else
				{
					bNeedSTX = false;
				}
			}
			else
			{
				nReadSize = 0;
			}
		}
		else
		{
			if (bNeedOP == true)
			{
				size_t nBytes = uart_read_bytes(ECHO_UART_PORT_NUM, data + nReadSize, sizeof(uint8_t) * 1, 50 / portTICK_PERIOD_MS);
				if (nBytes <= 0)
				{
					nReadSize = 0;
					bNeedSTX = true;
					vTaskDelay(10 / portTICK_PERIOD_MS);
					continue;
				}
				nReadHeader += nBytes;
				nReadSize += nBytes;

				nOP = data[1];
				if (nOP != 0x10)
				{
					nReadSize = 0;
					bNeedSTX = true;
					vTaskDelay(10 / portTICK_PERIOD_MS);
					continue;
				}

				bNeedOP = false;
			}
			// Length Check...
			if (bNeedLength == true)
			{
				int nBytes = uart_read_bytes(ECHO_UART_PORT_NUM, data + nReadSize, sizeof(uint8_t) * 1, 100 / portTICK_PERIOD_MS);
				// int nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * 1, 100 / portTICK_PERIOD_MS);
				if (nBytes <= 0)
				{
					vTaskDelay(10 / portTICK_PERIOD_MS);
					continue;
				}
				nReadHeader += nBytes;
				nReadSize += nBytes;

				nPacketLength = data[2];

				bNeedLength = false;

				// ReadPacket...
				memset(packet, 0, sizeof(uint8_t) * (nPacketLength + 1));
				memcpy(packet, data, nReadSize);
			}
			else
			{
				// 길이 만큼 읽는다.
				int nMaxRead = 0;
				if (nPacketLength - nReadSize < PACKET_BUF_SIZE)
					nMaxRead = nPacketLength - nReadSize;
				else
					nMaxRead = PACKET_BUF_SIZE;
				// int nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * nMaxRead, 100 / portTICK_PERIOD_MS);
				int nBytes = uart_read_bytes(ECHO_UART_PORT_NUM, data + nReadSize, sizeof(uint8_t) * nMaxRead, 100 / portTICK_PERIOD_MS);
				if (nBytes <= 0)
				{
					continue;
				}

				memcpy(packet + nReadSize, data, nBytes);

				nReadSize += nBytes;
				// 더읽어야 함
				if (nReadSize < nPacketLength)
				{
					continue;
				}

				if (data[0] == 0x02 && data[nReadSize - 1] == 0x03)
				{
					if (data[1] == 0x10) // op
					{
						E_PROTOCOL_PANID packet;
						memcpy(packet.buffer, data, sizeof(packet.buffer));
						ESP_LOGI(TAG, "Packet Arrived - OP: %02x, PANID: %04x",
								 packet.e_Data.op, packet.e_Data.panid);

						uint16_t panid_rcv = packet.e_Data.panid;

						if (pan_id != panid_rcv)
						{
							cd_nvs_write_panid(panid_rcv);

							usleep(100000);

							// Zigbee 네트워크를 초기화
							esp_zb_factory_reset();
							ESP_LOGI(TAG, "Zigbee factory reset");

							ESP_LOGI(TAG, "Rebooting device...");
							esp_restart(); // ESP32 reboot
						}

						// /* send on-off toggle command to remote device */
						// esp_zb_zcl_on_off_cmd_t cmd_req;
						// cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = on_off_light.short_addr;
						// cmd_req.zcl_basic_cmd.dst_endpoint = on_off_light.endpoint;
						// cmd_req.zcl_basic_cmd.src_endpoint = HA_ONOFF_SWITCH_ENDPOINT;
						// cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
						// cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
						// esp_zb_lock_acquire(portMAX_DELAY);
						// esp_zb_zcl_on_off_cmd_req(&cmd_req);
						// esp_zb_lock_release();
						// ESP_EARLY_LOGI(TAG, "Send 'on_off toggle' command to address(0x%x) endpoint(%d)", on_off_light.short_addr, on_off_light.endpoint);
					}
					else if (data[1] == 0x30) // op
					{
						E_PROTOCOL_PANID packet;
						memcpy(packet.buffer, data, sizeof(packet.buffer));
						ESP_LOGI(TAG, "Packet Arrived - OP: %02x, PANID: %04x",
								 packet.e_Data.op, packet.e_Data.panid);

						uint16_t panid_rcv = packet.e_Data.panid;

						// Zigbee 네트워크를 초기화
						esp_zb_factory_reset();
						ESP_LOGI(TAG, "Zigbee factory reset");

						ESP_LOGI(TAG, "Rebooting device...");
						esp_restart(); // ESP32 reboot
					}
					else
					{
						ESP_LOGE(TAG, "Unknown Packet... - <OP>");
					}
				}
				else
				{
					ESP_LOGE(TAG, "Packet Error... - <STX, ETX>");
				}

				// 초기화
				memset(data, 0, (uint)sizeof(data));
				nReadSize = nReadHeader = 0;

				bNeedSTX = true;
				bNeedOP = true;
				bNeedLength = true;
			}
		}
		// if (data[index] == '\n')
		// {
		//     printf("new line\n");
		//     data[index] = '\0'; // Null-terminate the received data
		//     ESP_LOGI(TAG, "Received: %s", data);

		//     // Echo the received data back to USB-Serial-JTAG
		//     usb_serial_jtag_write_bytes(data, len, portMAX_DELAY);
		//     index = 0;
		// }
		// else if (data[index] == '\r')
		// {
		//     printf("line feed\n");
		//     continue;
		// }
		// else
		// {
		//     index += 1;
		//     if (index > BUF_SIZE - 1)
		//     {
		//         index = 0;
		//     }
		// }
		vTaskDelay(10 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage
	}
}

//jylee-20241211: xiao esp32
void init_ext_ant()
{
	gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GPIO_PIN_EXT_ANT_1) | (1ULL << GPIO_PIN_EXT_ANT_2);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	if (ENABLE_EXT_ANT)
	{
		gpio_set_level(GPIO_PIN_EXT_ANT_1, 1);
		gpio_set_level(GPIO_PIN_EXT_ANT_2, 0);
	}
	else
	{
		gpio_set_level(GPIO_PIN_EXT_ANT_1, 0);
		gpio_set_level(GPIO_PIN_EXT_ANT_2, 0);
	}
}

// void esp_zb_task2(void *pvParameters)
// {
//     ESP_LOGI(TAG, "Starting Zigbee Coordinator");

//     // 1. Zigbee NVRAM 초기화 (이전 네트워크 설정 제거)
//     esp_zb_nvram_erase_at_start(true);

//     // 2. Zigbee 플랫폼 설정
//     esp_zb_platform_config_t zb_platform_config = {
//         .radio_config = {
//             .radio_mode = ZB_RADIO_MODE_NATIVE,
//         },
//         .host_config = {
//             .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
//         }
//     };
//     esp_zb_platform_config(&zb_platform_config);

//     // 3. Zigbee Coordinator 네트워크 설정
//     esp_zb_cfg_t zb_nwk_cfg = {
//         .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
//         .install_code_policy = INSTALLCODE_POLICY_ENABLE,
//         .nwk_cfg.zczr_cfg = {
//             .max_children = MAX_CHILDREN,
//         }
//     };
//     esp_zb_init(&zb_nwk_cfg);

// 	//jylee-20240530: NVS Read
//     pan_id = cd_nvs_read_panid();
//     if (pan_id == 0)
//     {
//         pan_id = DEFAULT_PANID; // for test///
//     }

//     // 4. PAN ID 및 채널 마스크 설정 (이 순서 중요!)
//     esp_zb_set_pan_id(pan_id);
//     esp_zb_set_primary_network_channel_set((1L << 15));

//     // 5. 네트워크 형성 시작
//     ESP_LOGI(TAG, "Starting network formation on channel 15...");
//     esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);

//     // 필수: ZBOSS 메인 루프 돌리기
//     esp_zb_main_loop_iteration();
// }

void app_main(void)
{
    // esp_zb_platform_config_t config = {
    //     .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
    //     .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    // };
    ESP_ERROR_CHECK(nvs_flash_init());
    //ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

	//jylee-20241211: xiao esp32
	init_ext_ant();

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);

    xTaskCreate(usb_serial_task, "usb_serial_task", 8192, NULL, 10, NULL);

    ///xTaskCreate(test_task, "Test", 4096, NULL, 5, NULL);
}
