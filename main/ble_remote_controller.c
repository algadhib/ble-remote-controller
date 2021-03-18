#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"

// Mainly based on https://github.com/lucadentella/esp32-tutorial/blob/master/26_ble_advertise/main/main.c

// Timer setup
TimerHandle_t advTimer;
int advTimerID = 0;
int advTimerInterval = 500; // in ms

// timer handler
void advTimerHandler(TimerHandle_t xTimer)
{
	printf("Advertismet timeout.\n");
	esp_ble_gap_stop_advertising();
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

// GAP callback
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event)
	{

	case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:

		printf("ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT\n");
		esp_ble_gap_start_advertising(&ble_adv_params);
		break;

	case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:

		printf("ESP_GAP_BLE_ADV_START_COMPLETE_EVT\n");
		if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
		{
			printf("Advertising started\n\n");

			// Start the adv timer
			xTimerStart(advTimer, 10);
		}
		else
			printf("Unable to start advertising process, error code %d\n\n", param->scan_start_cmpl.status);
		break;

	case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
		printf("ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT\n");

		// mission accomplished, deep sleep time...
		printf("Deep Sleep Time. Good Bye...\n\n");
		esp_deep_sleep_start();

		break;

	default:

		printf("Event %d unhandled\n\n", event);
		break;
	}
}

void app_main()
{

	printf("BT broadcast\n\n");

	// set components to log only errors
	esp_log_level_set("*", ESP_LOG_ERROR);

	// initialize nvs
	ESP_ERROR_CHECK(nvs_flash_init());
	printf("- NVS init ok\n");

	// release memory reserved for classic BT (not used)
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
	printf("- Memory for classic BT released\n");

	// initialize the BT controller with the default config
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_bt_controller_init(&bt_cfg);
	printf("- BT controller init ok\n");

	// enable the BT controller in BLE mode
	esp_bt_controller_enable(ESP_BT_MODE_BLE);
	printf("- BT controller enabled in BLE mode\n");

	// Create a timer to stop adv
	advTimer = xTimerCreate("advTimer", pdMS_TO_TICKS(advTimerInterval), pdFALSE, (void *)advTimerID, &advTimerHandler);
	printf("- Adv timer set up for %d ms\n\n", advTimerInterval);

	// initialize Bluedroid library
	esp_bluedroid_init();
	esp_bluedroid_enable();
	printf("- Bluedroid initialized and enabled\n");

	// register GAP callback function
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_gap_cb));
	printf("- GAP callback registered\n\n");

	// configure the adv data
	ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(adv_raw_data, 30));
	printf("- ADV data configured\n\n");
}
