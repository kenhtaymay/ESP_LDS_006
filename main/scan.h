#pragma once

#include <stdint.h>

#include "controller.h"

/* Latest revolution and controller status, written by the control task (core 1)
 * and read by the web push task (core 0). Guarded by a spinlock; copies are small. */

#define SCAN_POINTS 360

typedef struct {
    ctrl_status_t ctrl;
    ctrl_config_t config;   /* current (possibly unsaved) configuration */
    float valid_pct;        /* measurement frames / all frames over the last ~1 s */
    uint32_t frames_per_s;
    float revolutions_per_s;
    uint16_t raw_speed;     /* last speed field, unfiltered */
    uint32_t total_valid, total_speed_errors, total_bad_checksum;
    uint32_t total_raw_bytes;  /* everything UART1 received, parsed or not */
    int lidar_rx_gpio;
} live_status_t;

void scan_set(int angle, uint16_t distance_mm);  /* 0 = no return */
void scan_publish_status(const live_status_t *st);
void scan_snapshot(uint16_t distance_out[SCAN_POINTS], live_status_t *st_out);
