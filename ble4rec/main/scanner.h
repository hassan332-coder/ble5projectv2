/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#ifndef SCANNER_H
#define SCANNER_H

#include "common.h"

/* Scanner configuration */
#define RSSI_THRESHOLD        -30     /* Alert when stronger than -60 dBm */
#define RSSI_SAMPLES_REQUIRED  3       /* Need 3 strong readings before alert */
#define ALERT_LED_GPIO         2       /* Built-in LED on GPIO 2 */

/* Remove these static declarations from the header */
/* They should only be declared in scanner.c */

/* Public function declarations */
int scanner_init(void);
void scanner_start(void);
void scanner_stop(void);

#endif // SCANNER_H