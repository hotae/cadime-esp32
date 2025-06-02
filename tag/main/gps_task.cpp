#include "gps_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "esp_timer.h"

#include "gps_parser.h"
extern struct minmea_sentence_rmc frame;

extern bool gps_connected;
extern int64_t gps_receive_time;
extern bool gps_activate;

extern char *TAG;

void sendGPSData(float lat, float lon, float spd);

void gps_standby()
{
    const uint8_t UBX_PMREQ_STANDBY[] = {
        0xB5, 0x62,       // UBX sync chars
        0x02, 0x41,       // Class=0x02 (RXM), ID=0x41 (PMREQ)
        0x10, 0x00,       // length=16 (little endian)
        // Payload (16 bytes)
        0x00, 0x00, 0x00, 0x00, // version=0, reserved[3]
        0x00, 0x00, 0x00, 0x00, // duration=0 (무기한)
        0x06, 0x00, 0x00, 0x00, // flags=0x06 (backup+force)
        0x08, 0x00, 0x00, 0x00, // wakeupSources=0x20 (EXTINT0)
        // Checksum
        0x61, 0x6B
    };
    const uint8_t UBX_MON_VER_REQ[] = {
        0xB5, 0x62,  // Sync chars
        0x0A, 0x04,  // Class=MON(0x0A), ID=VER(0x04)
        0x00, 0x00,  // Length=0
        0x0E, 0x34   // Checksum CK_A=0x0E, CK_B=0x34
    };
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char *)UBX_PMREQ_STANDBY, sizeof(UBX_PMREQ_STANDBY));

    // const char *str = "$PMTK161,0*28\r\n";
    // uart_write_bytes(ECHO_UART_PORT_NUM, str, strlen(str));

    printf("gps standby...\n");
}

void gps_wakeup()
{
    const char *str = "$PMTK010\r\n";
    uart_write_bytes(ECHO_UART_PORT_NUM, str, strlen(str));

    printf("gps wakeup...\n");
}

void gps_task(void *arg)
{
	/* Configure parameters of an UART driver,
     * communication pins and install the driver */
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

    // Configure a temporary buffer for the incoming data
    char *data = (char *) malloc(BUF_SIZE);
    char *line = (char *) malloc(BUF_SIZE);

    ///$GNRMC,082403.00,A,3724.41359,N,12705.32317,E,0.488,,210524,,,A,V*10
    int index = 0;
    int line_idx = 0;

    ///int cnt = 0;
	gps_receive_time = esp_timer_get_time();
	int64_t gps_activate_time = 0;

	ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

	gps_wakeup();
	
    while (1) {
        // Read data from the UART
        //int len = uart_read_bytes(ECHO_UART_PORT_NUM, data + index, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
        
        try
        {
            int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, 20, 20 / portTICK_PERIOD_MS);
            if (len)
            {
                for (int i = 0; i < len; i++)
                {
                    line[line_idx] = data[i];
                    if (line[line_idx] == '\n')
                    {
                        line[line_idx] = '\0';

                        ///ESP_LOGI(TAG, "Recv str: %s", (char *) line);
                        if (strlen(line) > 6 && !memcmp(line, "$GNRMC", 6))
                        {
                            // ESP_LOGI(TAG, "Recv str: %s", (char *) data);
                            if (gps_parser((const char *)line) > 0)
                            {
                                // printf("$GNRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                //         minmea_tocoord(&frame.latitude),
                                //         minmea_tocoord(&frame.longitude),
                                //         minmea_tofloat(&frame.speed));
                                float lat = minmea_tocoord(&frame.latitude);
                                float lon = minmea_tocoord(&frame.longitude);
                                float spd = minmea_tofloat(&frame.speed);

                                // jylee-20241107: for UART to RPI
                                printf("$GNRMC floating point degree coordinates and speed: (%f,%f) %f\n",
                                       lat,
                                       lon,
                                       spd);
                                // jylee-20240612
                                if (esp_zb_bdb_dev_joined())
                                {
                                    sendGPSData(lat, lon, spd);
                                }
                                gps_connected = true;
                                gps_receive_time = esp_timer_get_time();
                            }
                            // jylee-20240619: for test
                            else
                            {
                                // jylee-20240612
                                if (esp_zb_bdb_dev_joined())
                                {
                                    printf("$GNRMC floating point degree coordinates and speed: (0, 0) 0\n");
                                    sendGPSData(0, 0, 0);
                                }
                            }
                            gps_activate = true;
                            gps_activate_time = esp_timer_get_time();
                        }
                        line_idx = 0;
                    }
                    else
                    {
                        line_idx++;
                        if (line_idx >= BUF_SIZE - 1)
                        {
                            line_idx = 0;
                        }
                    }
                }
            }
            else
            {
                if (gps_activate_time > 0 && esp_timer_get_time() - gps_activate_time > 10000000) // 10 sec
                {
                    gps_activate = false;
                }
                //     cnt++;
                //     if (cnt > 200)
                //     {
                //         printf("GPS data receive failed...\n");
                //         sendGPSData(-999, -999, -999);
                //         cnt = 0;
                //     }
            }

            // jylee-20241125: check gps status
            if (esp_timer_get_time() - gps_receive_time > 5000000) // 5 sec
            {
                gps_connected = false;
            }
        }
        catch(const std::exception& e)
        {
            ESP_LOGE(TAG, "error: %s", e.what());
        }

		esp_task_wdt_reset();
        vTaskDelay(10 / portTICK_PERIOD_MS); // Add some delay to avoid high CPU usage
    }
}
