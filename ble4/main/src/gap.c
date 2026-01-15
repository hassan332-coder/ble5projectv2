/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "gap.h"
#include "common.h"

/* Private function declarations */
inline static void format_addr(char *addr_str, uint8_t addr[]);
static void start_advertising(void);

/* Private variables */
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static uint8_t esp_uri[] = {BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f', '.', 'c', 'o', 'm'};

/* Private functions */
inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}
static void start_advertising(void) {
    /* Local variables */
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    ESP_LOGI(TAG, "Starting data beacon with custom advertising data...");

    /* Stop any existing advertising and wait */
    ble_gap_adv_stop();
    vTaskDelay(pdMS_TO_TICKS(500));  // Increased delay

    /* ⭐⭐ SMALLER DATA PAYLOAD ⭐⭐ */
    uint8_t custom_data[] = {
        0xC0, 0x04,                    // Espressif Company ID
        0xDA, 0xDA,                    // Custom marker
        // Reduced payload - 12 bytes of actual data
        0x01,                          // Data type
        0x13,                          // Temperature
        0x37,                          // Humidity  
        0x64,                          // Battery
        0xAA, 0xBB, 0xCC,              // Value 1
        0x11, 0x22, 0x33,              // Value 2
        0x44, 0x55, 0x66               // Value 3
    };

    /* Configure MINIMAL advertising data */
    memset(&adv_fields, 0, sizeof(adv_fields));  // Clear all fields
    
    /* Only set manufacturer data - remove name to save space */
    adv_fields.mfg_data = custom_data;
    adv_fields.mfg_data_len = sizeof(custom_data);

    /* Set advertising data with retry */
    int retry_count = 0;
    while (retry_count < 3) {
        rc = ble_gap_adv_set_fields(&adv_fields);
        if (rc == 0) break;
        ESP_LOGW(TAG, "Retry %d setting adv data: %d", retry_count + 1, rc);
        vTaskDelay(pdMS_TO_TICKS(200));
        retry_count++;
    }

    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data after %d retries: %d", retry_count, rc);
        return;
    }

    /* Configure advertising parameters */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_NON;
    adv_params.itvl_min = 0x00A0;
    adv_params.itvl_max = 0x00A0;
    adv_params.channel_map = 0x07;

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                          &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "✅ Data beacon started! Payload: %d bytes", sizeof(custom_data));
}
/* Public functions */
void adv_init(void) {
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);

    /* Start advertising. */
    start_advertising();
}

int gap_init(void) {
    /* Local variables */
    int rc = 0;

    /* Initialize GAP service */
    ble_svc_gap_init();

    /* Set GAP device name */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d",
                 DEVICE_NAME, rc);
        return rc;
    }

    /* Set GAP device appearance */
    rc = ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE_GENERIC_TAG);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device appearance, error code: %d", rc);
        return rc;
    }
    return rc;
}
