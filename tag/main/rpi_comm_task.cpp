#include "rpi_comm_task.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "zigbee.h"
#include "esp_zigbee_core.h"

#include "switch_driver.h"

#include "driver/usb_serial_jtag.h"

#include "common.h"
#include "nvs_util.h"
#include "msg_struct.h"

#define BUF_SIZE (1024)
#define PACKET_BUF_SIZE 255

extern const char *TAG = "MODELE_GPS_TAG";
extern light_bulb_device_params_t on_off_light;

extern uint16_t pan_id;

int64_t panid_response_time = 0;

void usb_serial_echo_test()
{
	const char *test_data = "Hello from ESP32-H2";
	uint8_t buffer[128];
	int bytes_written, bytes_read;

	// 데이터를 라즈베리파이로 송신
	bytes_written = usb_serial_jtag_write_bytes((const uint8_t *)test_data, strlen(test_data), 500 / portTICK_PERIOD_MS);
	printf("%s", test_data);
	if (bytes_written > 0)
	{
		ESP_LOGI("USB_SERIAL", "Sent data: %s", test_data);
	}
	else
	{
		ESP_LOGE("USB_SERIAL", "Failed to send data");
	}

	// 라즈베리파이에서 수신한 데이터 읽기
	bytes_read = usb_serial_jtag_read_bytes(buffer, sizeof(buffer) - 1, 100 / portTICK_PERIOD_MS);
	if (bytes_read > 0)
	{
		buffer[bytes_read] = '\0'; // Null-terminate the received string
		ESP_LOGI("USB_SERIAL", "Received data: %s", buffer);
	}

	vTaskDelay(1000 / portTICK_PERIOD_MS); // 1초 간격으로 반복
}

void sendPanIdRequest()
{
	E_PROTOCOL_PANID eProtocol;

	ESP_LOGI(TAG, "Send panid to server - panid: %d", pan_id);

	eProtocol.e_Data.stx = E_STX;
	eProtocol.e_Data.op = 0x21; // PanID Request
	eProtocol.e_Data.len = 6;	// packet size
	eProtocol.e_Data.panid = pan_id;
	eProtocol.e_Data.etx = E_ETX;

	ESP_LOGI(TAG, "size: %d", sizeof(eProtocol));
	for (int i = 0; i < 6; i++)
	{
		printf("%02x ", eProtocol.buffer[i]);
	}
	printf("\n");

	int bytes_written = usb_serial_jtag_write_bytes((const uint8_t *)eProtocol.buffer, sizeof(eProtocol.buffer), 100 / portTICK_PERIOD_MS);
	if (bytes_written > 0)
	{
		ESP_LOGI("USB_SERIAL", "Sent byte: %d", bytes_written);
	}
	else
	{
		ESP_LOGE("USB_SERIAL", "Failed to send data");
	}
}

void usb_serial_task(void *pvParameters)
{
	uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
	uint8_t *packet = (uint8_t *)malloc(BUF_SIZE);

	// Install USB-Serial-JTAG driver
	// const usb_serial_jtag_driver_config_t usb_serial_config = {
	// 	.rx_buffer_size = PACKET_BUF_SIZE,
	// 	.tx_buffer_size = PACKET_BUF_SIZE,
	// };
	usb_serial_jtag_driver_config_t usb_serial_config = {
		PACKET_BUF_SIZE,
		PACKET_BUF_SIZE,
	};

	esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_config);
	if (ret != ESP_OK)
	{
		ESP_LOGE("USB_SERIAL", "Failed to install USB-Serial-JTAG driver: %s", esp_err_to_name(ret));
	}
	else
	{
		ESP_LOGI("USB_SERIAL", "Installed USB-Serial-JTAG driver");
	}

	uint8_t bNeedSTX = true;
	uint8_t bNeedOP = true;
	uint8_t bNeedLength = true;
	unsigned int nReadSize = 0;
	unsigned int nReadHeader = 0;
	uint8_t nOP = 0;
	unsigned int nPacketLength = 0;

	// 0: STX, 1: OP, 2: LEN, 3: DATA, 4: ETX
	int64_t panid_send_time = esp_timer_get_time();
	panid_response_time = 0;
	while (1)
	{
		//no response from request -> reboot
		// if (esp_timer_get_time() - panid_response_time > 60000000)
		// {
		// 	if (!esp_zb_bdb_dev_joined() || esp_zb_get_current_channel() == 255)
		// 	{
		// 		// Zigbee 네트워크를 초기화
		// 		esp_zb_factory_reset();
		// 		ESP_LOGI(TAG, "Zigbee factory reset");

		// 		ESP_LOGI(TAG, "Rebooting device...");
		// 		esp_restart(); // ESP32 reboot
		// 	}
		// }

		if (esp_timer_get_time() - panid_send_time > 5000000) // 5 sec
		{
			sendPanIdRequest();
			panid_send_time = esp_timer_get_time();
		}

		// Read data from USB-Serial-JTAG
		// Preamble Search...
		if (bNeedSTX == true)
		{
			// Search STX
			int nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * 1, 100 / portTICK_PERIOD_MS);
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
					vTaskDelay(10 / portTICK_PERIOD_MS);
					nReadSize = 0;
					continue;
				}
				else
				{
					bNeedSTX = false;
				}
			}
			else
			{
				nReadSize = false;
			}
		}
		else
		{
			if (bNeedOP == true)
			{
				size_t nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * 1, 100 / portTICK_PERIOD_MS);
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
				if (nOP != 0x22)
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
				int nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * 1, 100 / portTICK_PERIOD_MS);
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

				int nBytes = usb_serial_jtag_read_bytes(data + nReadSize, sizeof(uint8_t) * nMaxRead, 100 / portTICK_PERIOD_MS);

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
					if (data[1] == 0x22) // op
					{
						panid_response_time = esp_timer_get_time();

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

		vTaskDelay(10 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage
	}
}
