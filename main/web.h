#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define WEB_COMMAND_LEN 64

/* HTTP server on port 80:
 *   GET  /     embedded web page (live scan map, status, PID tuning, OTA upload)
 *   GET  /ws   WebSocket: server pushes a binary scan frame and a JSON status ~4x/s;
 *              client text frames are command lines (same protocol as USB) that are
 *              queued to the control task
 *   POST /ota  raw firmware .bin body -> inactive OTA slot -> reboot */
void web_start(QueueHandle_t command_queue);
