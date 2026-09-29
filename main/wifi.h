#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Station mode with credentials kept in flash by the WiFi driver.
 * No stored SSID, or no IP within WIFI_CONNECT_TIMEOUT: start SmartConfig
 * (ESPTouch / ESPTouch v2 / AirKiss app on a phone) while still retrying. */

typedef struct {
    const char *state;  /* "connecting", "connected", "smartconfig", "off" */
    char ssid[33];
    char ip[16];
    int8_t rssi;
    uint8_t channel;
} wifi_info_t;

void wifi_start(const char *hostname);
void wifi_forget_and_smartconfig(void);  /* erase stored credentials, start SmartConfig */
void wifi_get_info(wifi_info_t *info);
