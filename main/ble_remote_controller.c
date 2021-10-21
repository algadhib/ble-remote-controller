#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include "driver/gpio.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

// Mainly based on https://github.com/lucadentella/esp32-tutorial/blob/master/26_ble_advertise/main/main.c

/**
 * Theory of operation:
 * 		- HOLD_EN_GPIO is connected to the enable bin of LDO.
 * 		- To start button is connected to LDO enable pin.
 * 		- On startup the esp32 will set the enable pin to high to keep the LDO ON.
 * 		- When the transmission is over, the LDO EN pin is pulled down and the system will be turned off.
 */

// Timer setup
TimerHandle_t advTimer;
int advTimerID = 0;
int advTimerInterval = 500; // in ms

#define HOLD_EN_GPIO 4

static void hold_en(void)
{
	gpio_reset_pin(HOLD_EN_GPIO);
	gpio_set_direction(HOLD_EN_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(HOLD_EN_GPIO, 1);
}

static void turn_off_esp(void)
{
	gpio_set_level(HOLD_EN_GPIO, 0);
}

void advTimerHandler(TimerHandle_t xTimer)
{
	esp_ble_gap_stop_advertising();
	turn_off_esp();
}

static esp_ble_adv_params_t ble_adv_params = {
	.adv_int_min = 0x20, // 20ms
	.adv_int_max = 0x40, // 40ms
	.adv_type = ADV_TYPE_NONCONN_IND,
	.own_addr_type = BLE_ADDR_TYPE_PUBLIC,
	.channel_map = ADV_CHNL_ALL,
	.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint8_t adv_raw_data[30] = {0x02, 0x01, 0x06, 0x1A, 0xFF, 0x4C, 0x00, 0x02, 0x15, 0xFD,
								   0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, 0xAF, 0xCF, 0xC6,
								   0xEB, 0x07, 0x64, 0x78, 0x25, 0x00, 0x00, 0x00, 0x00, 0xC5};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{
	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
		esp_ble_gap_start_advertising(&ble_adv_params);
		break;

	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
		if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
		{
			xTimerStart(advTimer, 10);
		}
		break;

	default:
		break;
	}
}

void app_main()
{
	hold_en();
	nvs_flash_init();

	// BLE config
	esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);
	esp_bt_controller_enable(ESP_BT_MODE_BLE);

	// Timer setup
	advTimer = xTimerCreate("advTimer", pdMS_TO_TICKS(advTimerInterval), pdFALSE, (void *)advTimerID, &advTimerHandler);

	// Start BLE and start advertising
	esp_bluedroid_init();
	esp_bluedroid_enable();
	esp_ble_gap_register_callback(esp_gap_cb);
	esp_ble_gap_config_adv_data_raw(adv_raw_data, 30);
}
