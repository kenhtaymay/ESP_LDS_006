#include "scan.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_distance[SCAN_POINTS];
static live_status_t s_status;

void scan_set(int angle, uint16_t distance_mm)
{
    if (angle < 0 || angle >= SCAN_POINTS) {
        return;
    }
    taskENTER_CRITICAL(&s_lock);
    s_distance[angle] = distance_mm;
    taskEXIT_CRITICAL(&s_lock);
}

void scan_publish_status(const live_status_t *st)
{
    taskENTER_CRITICAL(&s_lock);
    s_status = *st;
    taskEXIT_CRITICAL(&s_lock);
}

void scan_snapshot(uint16_t distance_out[SCAN_POINTS], live_status_t *st_out)
{
    taskENTER_CRITICAL(&s_lock);
    memcpy(distance_out, s_distance, sizeof(s_distance));
    *st_out = s_status;
    taskEXIT_CRITICAL(&s_lock);
}
