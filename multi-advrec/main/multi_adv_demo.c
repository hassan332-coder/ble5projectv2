#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "driver/gpio.h"

#define TAG "BIKE_RECEIVER"

/* ==================== CONFIG ==================== */
#define TARGET_DEVICE_ID     0xCAFE
#define TARGET_UUID          0x3412
#define SCANNER_LATITUDE     48.119237
#define SCANNER_LONGITUDE   -1.628055
#define PROXIMITY_DISTANCE_M 10.0
#define ALERT_LED_GPIO       8
#define EARTH_RADIUS_KM      6371.0
#define LED_BLINK_MS         200

/* ==================== DATA STRUCTURE ==================== */
typedef struct {
    double   latitude;
    double   longitude;
    float    speed_kmh;
    float    altitude;
    uint16_t device_id;
    int8_t   rssi;
} gps_data_t;

/* ==================== GLOBALS ==================== */
static bool     proximity_alert = false;
static uint32_t packet_count    = 0;
static uint32_t packets_10s     = 0;

static esp_ble_ext_scan_params_t ext_scan_params = {
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate= BLE_SCAN_DUPLICATE_DISABLE,
    .cfg_mask      = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK |
                     ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
    .uncoded_cfg   = {
        .scan_interval = 0x50,
        .scan_window   = 0x50,
        .scan_type     = BLE_SCAN_TYPE_PASSIVE,
    },
    .coded_cfg     = {
        .scan_interval = 0x50,
        .scan_window   = 0x50,
        .scan_type     = BLE_SCAN_TYPE_PASSIVE,
    }
};

static SemaphoreHandle_t scan_sem = NULL;

/* ==================== LED ==================== */
static void led_task(void *pv)
{
    bool state = false;
    while (1) {
        if (proximity_alert) {
            state = !state;
            gpio_set_level(ALERT_LED_GPIO, state);
            vTaskDelay(pdMS_TO_TICKS(LED_BLINK_MS));
        } else {
            gpio_set_level(ALERT_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ==================== PACKET MONITOR ==================== */
static void monitor_task(void *pv)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGI(TAG, "──────────────────────────────────────");
        ESP_LOGI(TAG, "📊 Packets last 10s: %lu | Total: %lu",
                 packets_10s, packet_count);
        ESP_LOGI(TAG, "──────────────────────────────────────");
        packets_10s = 0;
    }
}

/* ==================== MATH ==================== */
static double deg2rad(double deg) { return deg * M_PI / 180.0; }

static double calculate_distance(double lat1, double lon1,
                                  double lat2, double lon2)
{
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a    = sin(dlat/2) * sin(dlat/2) +
                  cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
                  sin(dlon/2) * sin(dlon/2);
    return EARTH_RADIUS_KM * 2 * atan2(sqrt(a), sqrt(1-a)) * 1000.0;
}

/* ==================== PARSER ==================== */
static int parse_packet(uint8_t *data, uint16_t len,
                         gps_data_t *out, int8_t rssi)
{
    uint8_t pos = 0;

    while (pos < len) {
        uint8_t ad_len  = data[pos];
        if (ad_len == 0) break;
        if (pos + ad_len + 1 > len) break;

        uint8_t ad_type = data[pos + 1];

        if (ad_type == 0x16 && ad_len >= 18) {
            uint16_t uuid = data[pos + 2] | (data[pos + 3] << 8);

            if (uuid == TARGET_UUID) {
                uint16_t dev_id = (data[pos + 4] << 8) | data[pos + 5];

                if (dev_id == TARGET_DEVICE_ID) {
                    uint8_t *g = &data[pos + 6];

                    /* Latitude */
                    int32_t lat_raw = ((int32_t)g[0] << 24) |
                                      ((int32_t)g[1] << 16) |
                                      ((int32_t)g[2] <<  8) |
                                       (int32_t)g[3];

                    /* Longitude */
                    int32_t lon_raw = ((int32_t)g[4] << 24) |
                                      ((int32_t)g[5] << 16) |
                                      ((int32_t)g[6] <<  8) |
                                       (int32_t)g[7];

                    /* Altitude */
                    uint16_t alt_raw   = (g[8]  << 8) | g[9];

                    /* Speed cm/s → km/h */
                    uint16_t spd_cms   = (g[10] << 8) | g[11];

                    out->device_id  = dev_id;
                    out->latitude   = lat_raw / 1000000.0;
                    out->longitude  = lon_raw / 1000000.0;
                    out->altitude   = (float)alt_raw;
                    out->speed_kmh  = (spd_cms * 36.0f) / 1000.0f;
                    out->rssi       = rssi;

                    return 1;
                }
            }
        }
        pos += ad_len + 1;
    }
    return 0;
}

/* ==================== PROXIMITY CHECK ==================== */
static void check_proximity(gps_data_t *b)
{
    double dist = calculate_distance(b->latitude,  b->longitude,
                                     SCANNER_LATITUDE, SCANNER_LONGITUDE);

    ESP_LOGI(TAG, "═══════════════════════════════════════════");
    ESP_LOGI(TAG, "📦 Packet #%lu", packet_count);
    ESP_LOGI(TAG, "📍 Beacon:  (%.6f, %.6f)", b->latitude, b->longitude);
    ESP_LOGI(TAG, "📍 Scanner: (%.6f, %.6f)",
             SCANNER_LATITUDE, SCANNER_LONGITUDE);
    ESP_LOGI(TAG, "🚀 Speed:   %.1f km/h", b->speed_kmh);
    ESP_LOGI(TAG, "🏔  Altitude: %.1f m",   b->altitude);
    ESP_LOGI(TAG, "📏 Distance: %.2f m",    dist);
    ESP_LOGI(TAG, "📶 RSSI:    %d dBm",     b->rssi);

    bool alert = (dist <= PROXIMITY_DISTANCE_M);
    if (alert && !proximity_alert) {
        ESP_LOGW(TAG, "🚨 ALERT! Beacon within %.0fm!", PROXIMITY_DISTANCE_M);
        proximity_alert = true;
    } else if (!alert && proximity_alert) {
        ESP_LOGI(TAG, "✅ Beacon out of range — alert cleared");
        proximity_alert = false;
    }
    ESP_LOGI(TAG, "═══════════════════════════════════════════");
}

/* ==================== GAP CALLBACK ==================== */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_SET_STATIC_RAND_ADDR_EVT:
        ESP_LOGI(TAG, "✅ Random address set");
        xSemaphoreGive(scan_sem);
        break;

    case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "🔍 Scanning started");
        break;

    case ESP_GAP_BLE_EXT_ADV_REPORT_EVT: {
        esp_ble_gap_ext_adv_report_t *r = &param->ext_adv_report.params;

        /* Accept CODED PHY only — matches sender */
        if (r->secondly_phy != ESP_BLE_GAP_PHY_CODED) return;
        if (r->adv_data_len < 30) return;

        gps_data_t beacon = {0};
        if (parse_packet(r->adv_data, r->adv_data_len,
                         &beacon, r->rssi)) {
            packet_count++;
            packets_10s++;
            check_proximity(&beacon);
        }
        break;
    }

    default:
        break;
    }
}

/* ==================== APP MAIN ==================== */
void app_main(void)
{
    /* NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

    /* LED */
    gpio_reset_pin(ALERT_LED_GPIO);
    gpio_set_direction(ALERT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALERT_LED_GPIO, 0);
    xTaskCreate(led_task,     "led",     2048, NULL, 5, NULL);
    xTaskCreate(monitor_task, "monitor", 2048, NULL, 4, NULL);

    /* BLE */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));

    scan_sem = xSemaphoreCreateBinary();

    ESP_LOGI(TAG, "╔════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   🚲 BIKE PROXIMITY RECEIVER       ║");
    ESP_LOGI(TAG, "║   Target: 0x%04X  Threshold: %.0fm ║",
             TARGET_DEVICE_ID, PROXIMITY_DISTANCE_M);
    ESP_LOGI(TAG, "╚════════════════════════════════════╝");

    /* Random address */
    esp_bd_addr_t rand_addr;
    esp_ble_gap_addr_create_static(rand_addr);
    ESP_ERROR_CHECK(esp_ble_gap_set_rand_addr(rand_addr));
    xSemaphoreTake(scan_sem, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Scan params */
    ESP_ERROR_CHECK(esp_ble_gap_set_ext_scan_params(&ext_scan_params));
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Start scanning forever */
    ESP_ERROR_CHECK(esp_ble_gap_start_ext_scan(0, 0));
    ESP_LOGI(TAG, "✅ Scanning for CODED PHY beacons...");
}