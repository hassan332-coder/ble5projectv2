#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "cJSON.h"

/* ─── Config ─────────────────────────────── */
#define WIFI_SSID       "Hassan S24"
#define WIFI_PASSWORD   "11111111"
#define DEVICE_ID       0xCAFE
#define TARGET_UUID     0x3412
#define BLE_UPDATE_MS   2000

#define ESP_BLE_GAP_EXT_ADV_PROP_NONCONN_NONSCAN 0x20

static const char *TAG = "BIKE_SENDER";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* ─── GPS variables ───────────────────────── */
static double    latitude   = 0.0;
static double    longitude  = 0.0;
static float     speed_kmh  = 0.0f;
static float     altitude   = 0.0f;
static bool      gps_valid  = false;
static portMUX_TYPE gps_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t ble_task_handle = NULL;

/* ─── BLE state ───────────────────────────── */
static bool ble_adv_started = false;

static esp_ble_gap_ext_adv_params_t ext_adv_params = {
    .type           = ESP_BLE_GAP_EXT_ADV_PROP_NONCONN_NONSCAN,
    .interval_min   = 0x140,    // 400 ms (change this)
    .interval_max   = 0x140,  
    .channel_map    = ADV_CHNL_ALL,
    .own_addr_type  = BLE_ADDR_TYPE_RANDOM,
    .filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy    = ESP_BLE_GAP_PHY_1M,
    .max_skip       = 0,
    .secondary_phy  = ESP_BLE_GAP_PHY_CODED,
    .sid            = 0,
    .scan_req_notif = false,
    .tx_power       = EXT_ADV_TX_PWR_NO_PREFERENCE,
};

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    {.instance = 0, .duration = 0}
};

/* ═══════════════════════════════════════════
   Build and start BLE advertisement
   ═══════════════════════════════════════════ */
static void build_and_start_adv(double lat, double lon,
                                 float spd, float alt)
{
    /* Convert to integers */
    int32_t  lat_int   = (int32_t)(lat * 1000000.0);
    int32_t  lon_int   = (int32_t)(lon * 1000000.0);
    uint16_t alt_int   = (uint16_t)(alt);
    uint16_t speed_cms = (uint16_t)(spd * 100.0f / 3.6f); /* km/h → cm/s */

    uint8_t adv_data[64];
    memset(adv_data, 0, sizeof(adv_data));
    int i = 0;

    /* ── Flags (3 bytes) ── */
    adv_data[i++] = 0x02;
    adv_data[i++] = 0x01;
    adv_data[i++] = 0x06;

    /* ── TX Power (3 bytes) ── */
    adv_data[i++] = 0x02;
    adv_data[i++] = 0x0A;
    adv_data[i++] = 0x00;

    /* ── Short device name (8 bytes) ── */
    adv_data[i++] = 0x07;
    adv_data[i++] = 0x09;
    adv_data[i++] = 'B';
    adv_data[i++] = 'I';
    adv_data[i++] = 'K';
    adv_data[i++] = 'E';
    adv_data[i++] = 'G';
    adv_data[i++] = 'S';

    /* ── Service UUID (4 bytes) ── */
    adv_data[i++] = 0x03;
    adv_data[i++] = 0x03;
    adv_data[i++] = (TARGET_UUID & 0xFF);
    adv_data[i++] = ((TARGET_UUID >> 8) & 0xFF);

    /* ── GPS Service Data ── */
    int svc_len_pos = i;        /* save length byte position */
    adv_data[i++] = 0x00;       /* length placeholder */
    adv_data[i++] = 0x16;       /* type: Service Data */
    adv_data[i++] = (TARGET_UUID & 0xFF);
    adv_data[i++] = ((TARGET_UUID >> 8) & 0xFF);

    /* Device ID (2 bytes) */
    adv_data[i++] = (DEVICE_ID >> 8) & 0xFF;
    adv_data[i++] = DEVICE_ID & 0xFF;

    /* Latitude (4 bytes big endian) */
    adv_data[i++] = (lat_int >> 24) & 0xFF;
    adv_data[i++] = (lat_int >> 16) & 0xFF;
    adv_data[i++] = (lat_int >>  8) & 0xFF;
    adv_data[i++] = (lat_int      ) & 0xFF;

    /* Longitude (4 bytes big endian) */
    adv_data[i++] = (lon_int >> 24) & 0xFF;
    adv_data[i++] = (lon_int >> 16) & 0xFF;
    adv_data[i++] = (lon_int >>  8) & 0xFF;
    adv_data[i++] = (lon_int      ) & 0xFF;

    /* Altitude (2 bytes) */
    adv_data[i++] = (alt_int >> 8) & 0xFF;
    adv_data[i++] = (alt_int     ) & 0xFF;

    /* Speed in cm/s (2 bytes) */
    adv_data[i++] = (speed_cms >> 8) & 0xFF;
    adv_data[i++] = (speed_cms     ) & 0xFF;

    /* Heading (2 bytes — zero for now) */
    adv_data[i++] = 0x00;
    adv_data[i++] = 0x00;

    /* Satellites (1 byte) */
    adv_data[i++] = 0x08;

    /* Timestamp (4 bytes) */
    adv_data[i++] = 0x65;
    adv_data[i++] = 0x5F;
    adv_data[i++] = 0x3A;
    adv_data[i++] = 0x80;

    /* Fill correct service data length */
    adv_data[svc_len_pos] = i - svc_len_pos - 1;

    /* Debug */
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "📡 Sending BLE advert (%d bytes)", i);
    ESP_LOGI(TAG, "📍 Lat: %.6f  Lon: %.6f", lat, lon);
    ESP_LOGI(TAG, "🚀 Speed: %.1f km/h  Alt: %.1f m", spd, alt);
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOG_BUFFER_HEX(TAG, adv_data, i);

    /* Stop → update → restart */
    if (ble_adv_started) {
        uint8_t inst = 0;
        esp_ble_gap_ext_adv_stop(1, &inst);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    esp_ble_gap_config_ext_adv_data_raw(0, i, adv_data);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_ble_gap_ext_adv_start(1, ext_adv);
    ble_adv_started = true;
}

/* ═══════════════════════════════════════════
   BLE GAP event handler
   ═══════════════════════════════════════════ */
static void gap_event_handler(esp_gap_ble_cb_event_t event,
                               esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE params set ✓");
        break;
    case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE random addr set ✓");
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE adv data updated ✓");
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        if (param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "BLE advertising started ✓");
        else
            ESP_LOGE(TAG, "BLE advertising FAILED: %d",
                     param->ext_adv_start.status);
        break;
    default:
        break;
    }
}

/* ═══════════════════════════════════════════
   BLE update task
   ═══════════════════════════════════════════ */
static void ble_update_task(void *pv)
{
    ble_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGI(TAG, "Waiting for first GPS from OwnTracks...");
    while (!gps_valid) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    while (1) {
        /* Wait for GPS notification OR timeout */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(BLE_UPDATE_MS));

        double lat, lon;
        float  spd, alt;

        portENTER_CRITICAL(&gps_mux);
        lat = latitude;
        lon = longitude;
        spd = speed_kmh;
        alt = altitude;
        portEXIT_CRITICAL(&gps_mux);

        build_and_start_adv(lat, lon, spd, alt);
    }
}

/* ═══════════════════════════════════════════
   HTTP POST — OwnTracks sends GPS here
   ═══════════════════════════════════════════ */
static esp_err_t gps_post_handler(httpd_req_t *req)
{
    char buf[512];
    int len = req->content_len;

    if (len >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Too large");
        return ESP_FAIL;
    }

    httpd_req_recv(req, buf, len);
    buf[len] = '\0';

    ESP_LOGI(TAG, "OwnTracks payload: %s", buf);

    cJSON *root = cJSON_Parse(buf);
    if (root) {
        cJSON *lat = cJSON_GetObjectItem(root, "lat");
        cJSON *lon = cJSON_GetObjectItem(root, "lon");
        cJSON *vel = cJSON_GetObjectItem(root, "vel");
        cJSON *alt = cJSON_GetObjectItem(root, "alt");

        if (cJSON_IsNumber(lat) && cJSON_IsNumber(lon)) {
            portENTER_CRITICAL(&gps_mux);
            latitude  = lat->valuedouble;
            longitude = lon->valuedouble;
            speed_kmh = cJSON_IsNumber(vel) ? (float)vel->valuedouble : 0.0f;
            altitude  = cJSON_IsNumber(alt) ? (float)alt->valuedouble : 0.0f;
            gps_valid = true;
            portEXIT_CRITICAL(&gps_mux);

            ESP_LOGI(TAG, "📍 Lat: %.6f  Lon: %.6f  "
                          "🚀 Speed: %.1f km/h  Alt: %.1f m",
                     latitude, longitude, speed_kmh, altitude);

            /* Wake BLE task immediately */
            if (ble_task_handle != NULL)
                xTaskNotifyGive(ble_task_handle);
        }
        cJSON_Delete(root);
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "[]");
    return ESP_OK;
}

/* ═══════════════════════════════════════════
   WiFi event handler
   ═══════════════════════════════════════════ */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "✅ IP: " IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "👉 OwnTracks URL: http://" IPSTR "/gps",
                 IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ═══════════════════════════════════════════
   app_main
   ═══════════════════════════════════════════ */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* WiFi */
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_cfg = {
        .sta = { .ssid = WIFI_SSID, .password = WIFI_PASSWORD }
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /* HTTP server */
    httpd_handle_t server;
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    httpd_start(&server, &hcfg);

    httpd_uri_t uri_post = {
        .uri     = "/gps",
        .method  = HTTP_POST,
        .handler = gps_post_handler
    };
    httpd_register_uri_handler(server, &uri_post);

    /* BLE */
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gap_register_callback(gap_event_handler);

    esp_bd_addr_t addr;
    esp_ble_gap_addr_create_static(addr);
    esp_ble_gap_ext_adv_set_params(0, &ext_adv_params);
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_ble_gap_ext_adv_set_rand_addr(0, addr);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "✅ Ready — waiting for OwnTracks GPS...");

    xTaskCreate(ble_update_task, "ble_task", 4096, NULL, 5, NULL);
}