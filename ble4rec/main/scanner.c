/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* Includes */
#include "scanner.h"
#include "driver/gpio.h"

/* Private variables */
static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static int strong_rssi_count = 0;
static bool alert_active = false;

/* Private function declarations */
static void check_rssi_and_alert(int8_t rssi);
static void trigger_alert(void);
static void reset_alert(void);
static bool is_my_beacon(struct ble_gap_disc_desc *disc);  // ADD THIS

/* Private function: Check if advertisement is from our beacon */
static bool is_my_beacon(struct ble_gap_disc_desc *disc) {
    /* Look for manufacturer-specific data */
    const uint8_t *data = disc->data;
    uint8_t data_len = disc->length_data;
    
    /* Our beacon data pattern:
     * 0xC0, 0x04 - Espressif Company ID (LSB, MSB)
     * 0xDA, 0xDA - Custom marker
     */
    const uint8_t beacon_pattern[] = {0xC0, 0x04, 0xDA, 0xDA};
    
    /* Search through advertisement data for manufacturer-specific data (type 0xFF) */
    int offset = 0;
    while (offset < data_len) {
        uint8_t field_len = data[offset];
        if (field_len == 0 || offset + field_len >= data_len) {
            break;
        }
        
        uint8_t field_type = data[offset + 1];
        
        /* Manufacturer Specific Data (type 0xFF) */
        if (field_type == 0xFF) {
            /* Check if this matches our beacon pattern */
            if (field_len >= 6) {  // At least type(1) + len(1) + 4 bytes pattern
                const uint8_t *mfg_data = &data[offset + 2];  // Skip length and type
                
                /* Compare with our pattern */
                if (mfg_data[0] == beacon_pattern[0] &&
                    mfg_data[1] == beacon_pattern[1] &&
                    mfg_data[2] == beacon_pattern[2] &&
                    mfg_data[3] == beacon_pattern[3]) {
                    return true;
                }
            }
        }
        
        offset += field_len + 1;
    }
    
    return false;
}

/* Private function: Format MAC address */
inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", 
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

/* Private function: Trigger proximity alert */
static void trigger_alert(void) {
    ESP_LOGW(TAG, "🚨 PROXIMITY ALERT! Bike is NEAR!");
    ESP_LOGW(TAG, "   RSSI > %d dBm", RSSI_THRESHOLD);
    ESP_LOGW(TAG, "   Estimated distance: < 10 meters");
    
    /* Flash LED rapidly to alert user */
    for (int i = 0; i < 10; i++) {
        gpio_set_level(ALERT_LED_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(ALERT_LED_GPIO, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    /* Keep LED on to indicate active alert */
    gpio_set_level(ALERT_LED_GPIO, 1);
}

/* Private function: Reset alert */
static void reset_alert(void) {
    gpio_set_level(ALERT_LED_GPIO, 0);
    ESP_LOGI(TAG, "alert deactivated");
}

/* Private function: Check RSSI and trigger alert */
static void check_rssi_and_alert(int8_t rssi) {
    /* Print RSSI for monitoring */
    ESP_LOGI(TAG, "RSSI: %d dBm", rssi);
    
    /* Check if RSSI is strong enough */
    if (rssi > RSSI_THRESHOLD) {
        strong_rssi_count++;
        ESP_LOGI(TAG, "strong signal detected! count: %d/%d", 
                 strong_rssi_count, RSSI_SAMPLES_REQUIRED);
        
        /* Check if we have enough strong readings */
        if (strong_rssi_count >= RSSI_SAMPLES_REQUIRED && !alert_active) {
            trigger_alert();
            alert_active = true;
        }
    } else {
        /* RSSI is weak - reset counter */
        if (strong_rssi_count > 0) {
            ESP_LOGI(TAG, "signal weakened, resetting counter");
            strong_rssi_count = 0;
            
            /* Turn off alert if it was active */
            if (alert_active) {
                reset_alert();
                alert_active = false;
            }
        }
    }
}

/* Private function: BLE event handler */
static int on_ble_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            /* Advertisement received - check if it's our beacon first */
            if (is_my_beacon(&event->disc)) {
                ESP_LOGI(TAG, "My beacon detected!");
                check_rssi_and_alert(event->disc.rssi);
            } else {
                /* Optional: Log other beacons for debugging */
                // ESP_LOGD(TAG, "Other device detected, RSSI: %d dBm", event->disc.rssi);
            }
            break;
            
        case BLE_GAP_EVENT_DISC_COMPLETE:
            /* Scan cycle complete - restart scanning */
            ESP_LOGI(TAG, "scan cycle complete, restarting...");
            scanner_start();
            break;
    }
    return 0;
}

/* Public function: Initialize scanner */
int scanner_init(void) {
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return rc;
    }

    /* Figure out BT address to use while scanning */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return rc;
    }

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return rc;
    }
    
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "scanner device address: %s", addr_str);

    /* Configure LED GPIO for alerts */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ALERT_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(ALERT_LED_GPIO, 0);

    return 0;
}

/* Public function: Start scanning */
void scanner_start(void) {
    int rc = 0;
    
    /* Configure scan parameters */
    struct ble_gap_disc_params disc_params = {
        .itvl = 0x0060,        /* Scan interval: 60ms (0x60 * 0.625ms) */
        .window = 0x0060,      /* Scan window: 60ms */
        .filter_policy = 0,    /* Accept all advertisements */
        .limited = 0,          /* General discovery */
        .passive = 1,          /* Passive scanning (no scan requests) */
        .filter_duplicates = 0 /* Show all advertisements */
    };

    /* Start continuous scanning */
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, 
                     &disc_params, on_ble_event, NULL);
    
    if (rc == 0) {
        ESP_LOGI(TAG, "continuous scanning started");
        ESP_LOGI(TAG, "alert threshold: RSSI > %d dBm", RSSI_THRESHOLD);
        ESP_LOGI(TAG, "looking for beacon pattern: C0:04:DA:DA");
    } else {
        ESP_LOGE(TAG, "failed to start scanning, error code: %d", rc);
    }
}

/* Public function: Stop scanning */
void scanner_stop(void) {
    ble_gap_disc_cancel();
    ESP_LOGI(TAG, "scanning stopped");
}