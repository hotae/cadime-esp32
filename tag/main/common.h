#ifndef __COMMON_H
#define __COMMON_H

#include <stdio.h>

#include "zigbee.h"
#include "esp_zigbee_core.h"

#include "switch_driver.h"

//jylee-20241218: for ONE_BUTTON_CONTROL else comment out
#define ONE_BUTTON_CONTROL 1
#define ENABLE_SW_POWER_CONTROL 0
#define _DEV_TEST 0
#define _SERVICE_BUILD 1
//jylee-20250102: every 20second power control
//#define POWER_CONTROL_20S 1
/////

#if _SERVICE_BUILD
//#define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  /* Zigbee primary channel mask */
//#define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   (1l << 13)  /* Zigbee primary channel mask use in the example */
#define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK
#define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

//jylee-20241119: for taegwang set
//#define DEFAULT_PANID     0x2001
//jylee-20250121: for service-test #1
//#define DEFAULT_PANID 		0x2100
//jylee-20250121: for service-test #2
//#define DEFAULT_PANID 		0x2100
#define DEFAULT_PANID 		0x2000
#else
#if !_DEV_TEST
#define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  /* Zigbee primary channel mask */
#define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   (1l << 11)  /* Zigbee primary channel mask use in the example */
#define DEFAULT_PANID     0x1234
#endif
#if _DEV_TEST
//jylee-20241017: for jylee test
#define DEFAULT_ZB_PRIMARY_CHANNEL_MASK     (1l << 13)  /* Zigbee primary channel mask */
#define DEFAULT_ZB_SECONDARY_CHANNEL_MASK   (1l << 11)  /* Zigbee primary channel mask use in the example */
//#define DEFAULT_PANID     0x9436
#define DEFAULT_PANID     0x9437 //zero
#endif
#endif

#define CUSTOM_PAN_ID 0x4C49
//#define CUSTOM_EXT_PAN_ID {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67}
#define CUSTOM_EXT_PAN_ID {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x68}

#define ADC_GPIO 2
#define PWR_LED_GPIO 8
#define BAT_LED_GPIO 3
#define SOUND_GPIO 14

//jylee-20241217: one button control
#ifdef ONE_BUTTON_CONTROL
#define GPIO_IN_MAIN_BUTTON 0 
#else
#define GPIO_IN_MAIN_BUTTON GPIO_INPUT_IO_TOGGLE_SWITCH
#endif

#define GPIO_OUT_POWER_CNTL 1
#define GPIO_OUT_POWER_CNTL2 14 //jylee-20250408: for backup power control
//jylee-20241113: 100ms / 20 sec power control
#define GPIO_IN_PWR_CNT_GPIO 22

#define VOLTAGE_RATIO 0.028
#define FULL_CHARGE 2.75

#define LED_PIN 8             // LED가 연결된 GPIO 핀 번호
#define LEDC_TIMER_LED LEDC_TIMER_0  // LEDC 타이머 0 사용
#define LEDC_MODE_LED LEDC_LOW_SPEED_MODE  // 저속 PWM 모드 사용
#define LEDC_CHANNEL_LED LEDC_CHANNEL_0    // LEDC 채널 0 사용
#define LEDC_DUTY_RES_LED LEDC_TIMER_13_BIT // 13비트 해상도 (0~8191까지의 값)
#define LEDC_FREQUENCY_LED 1000    // 5kHz의 주파수 사용

#define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
#define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

//#define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM) //jylee-20250217
#define ECHO_UART_PORT_NUM  ((uart_port_t)(CONFIG_EXAMPLE_UART_PORT_NUM))
// #define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
#define ECHO_UART_BAUD_RATE     (38400)
#define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

#define BUF_SIZE (1024)

//#define POWEROFF_DURING_CHARGING_SEC 300
#define POWEROFF_DURING_CHARGING_SEC 30

static switch_func_pair_t button_func_pair[] = {
    {GPIO_IN_MAIN_BUTTON, SWITCH_ONOFF_TOGGLE_CONTROL}
};

typedef struct zdo_info_ctx_s {
    uint8_t endpoint;
    uint16_t short_addr;
} zdo_info_user_ctx_t;

typedef struct light_bulb_device_params_s {
    esp_zb_ieee_addr_t ieee_addr;
    uint8_t  endpoint;
    uint16_t short_addr;
} light_bulb_device_params_t;

#endif