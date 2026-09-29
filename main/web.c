#include "web.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_chip_info.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "motor_pwm.h"
#include "scan.h"
#include "sdkconfig.h"
#include "wifi.h"

#define PUSH_INTERVAL_MS 250
#define MAX_CLIENTS 7
#define OTA_CHUNK 4096
#define SCAN_MAGIC 0x4E414353  /* "SCAN", first 4 bytes of the binary frame */

static const char *TAG = "web";

extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[] asm("_binary_index_html_end");

static httpd_handle_t s_server;
static QueueHandle_t s_commands;
static volatile bool s_ota_running;

typedef struct {
    size_t len;
    httpd_ws_type_t type;
    uint8_t data[];
} ws_message_t;

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, index_html_end - index_html_start - 1);
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ESP_OK;  /* handshake done */
    }
    httpd_ws_frame_t frame = {.type = HTTPD_WS_TYPE_TEXT};
    esp_err_t err = httpd_ws_recv_frame(req, &frame, 0);  /* length only */
    if (err != ESP_OK) {
        return err;
    }
    uint8_t *buf = calloc(1, frame.len + 1);
    if (!buf) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buf;
    err = frame.len ? httpd_ws_recv_frame(req, &frame, frame.len) : ESP_OK;
    if (err == ESP_OK && frame.type == HTTPD_WS_TYPE_TEXT) {
        char line[WEB_COMMAND_LEN] = {0};
        strncpy(line, (char *)buf, sizeof(line) - 1);
        xQueueSend(s_commands, line, 0);
    }
    free(buf);
    return err;
}

static void restart_cb(void *arg)
{
    esp_restart();
}

/* The body is the raw .bin (the page sends the file with fetch/XHR, no multipart). */
static esp_err_t ota_post(httpd_req_t *req)
{
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target || req->content_len <= 0 || req->content_len > target->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no OTA slot or bad size");
        return ESP_FAIL;
    }
    esp_ota_handle_t ota;
    if (esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin failed");
        return ESP_FAIL;
    }
    char *buf = malloc(OTA_CHUNK);
    if (!buf) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
        return ESP_FAIL;
    }
    s_ota_running = true;
    int remaining = req->content_len;
    esp_err_t err = ESP_OK;
    while (remaining > 0) {
        int n = httpd_req_recv(req, buf, remaining < OTA_CHUNK ? remaining : OTA_CHUNK);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (n <= 0) {
            err = ESP_FAIL;
            break;
        }
        err = esp_ota_write(ota, buf, n);
        if (err != ESP_OK) {
            break;
        }
        remaining -= n;
    }
    free(buf);
    s_ota_running = false;

    if (err != ESP_OK) {
        esp_ota_abort(ota);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "upload failed");
        return ESP_FAIL;
    }
    err = esp_ota_end(ota);  /* validates the image (magic, chip, checksum) */
    if (err == ESP_OK) {
        err = esp_ota_set_boot_partition(target);
    }
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "image rejected (not a valid ESP32-S3 app?)");
        return ESP_FAIL;
    }
    httpd_resp_sendstr(req, "OK, rebooting into the new firmware");
    ESP_LOGW(TAG, "OTA written to %s, rebooting", target->label);

    static esp_timer_handle_t timer;
    const esp_timer_create_args_t args = {.callback = restart_cb, .name = "ota_restart"};
    if (!timer) {
        esp_timer_create(&args, &timer);
    }
    esp_timer_start_once(timer, 1000 * 1000);
    return ESP_OK;
}

/* Runs in the httpd task via httpd_queue_work, so socket writes never race. */
static void broadcast_work(void *arg)
{
    ws_message_t *msg = arg;
    size_t count = MAX_CLIENTS;
    int fds[MAX_CLIENTS];
    if (httpd_get_client_list(s_server, &count, fds) == ESP_OK) {
        httpd_ws_frame_t frame = {.type = msg->type, .payload = msg->data, .len = msg->len};
        for (size_t i = 0; i < count; i++) {
            if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_send_frame_async(s_server, fds[i], &frame);
            }
        }
    }
    free(msg);
}

static bool any_ws_client(void)
{
    size_t count = MAX_CLIENTS;
    int fds[MAX_CLIENTS];
    if (httpd_get_client_list(s_server, &count, fds) != ESP_OK) {
        return false;
    }
    for (size_t i = 0; i < count; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            return true;
        }
    }
    return false;
}

static void queue_broadcast(httpd_ws_type_t type, const void *data, size_t len)
{
    ws_message_t *msg = malloc(sizeof(ws_message_t) + len);
    if (!msg) {
        return;
    }
    msg->type = type;
    msg->len = len;
    memcpy(msg->data, data, len);
    if (httpd_queue_work(s_server, broadcast_work, msg) != ESP_OK) {
        free(msg);
    }
}

static const char *reset_reason_text(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "usb";
    case ESP_RST_EXT: return "external pin";
    case ESP_RST_DEEPSLEEP: return "deep sleep";
    default: return "other";
    }
}

/* SSIDs are arbitrary bytes; keep the JSON valid. */
static void json_safe(char *s)
{
    for (; *s; s++) {
        if (*s == '"' || *s == '\\' || (unsigned char)*s < 0x20) {
            *s = '_';
        }
    }
}

static int build_status_json(char *out, size_t size, const live_status_t *st)
{
    wifi_info_t wifi;
    wifi_get_info(&wifi);
    json_safe(wifi.ssid);
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    const ctrl_config_t *c = &st->config;

    return snprintf(out, size,
        "{\"type\":\"status\","
        "\"motor\":{\"mode\":\"%s\",\"state\":\"%s\",\"setpoint\":%.0f,\"filtered\":%.0f,\"raw_speed\":%u,"
        "\"duty\":%.1f,\"duty_pct\":%.2f,\"lidar_wanted\":%s,\"restarts\":%lu},"
        "\"lidar\":{\"valid_pct\":%.1f,\"frames_per_s\":%lu,\"rev_per_s\":%.2f,\"total_valid\":%lu,"
        "\"total_speed_errors\":%lu,\"total_bad_checksum\":%lu,\"raw_bytes\":%lu},"
        "\"config\":{\"setpoint\":%.0f,\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,\"duty_min\":%.0f,\"duty_max\":%.0f,"
        "\"slew\":%.2f,\"auto_start\":%s,\"pwm_hz\":%lu},"
        "\"hw\":{\"lidar_rx_gpio\":%d,\"robot_rx_gpio\":%d,\"pwm_gpio\":%d,\"pwm_freq_hz\":%lu,"
        "\"pwm_ticks\":%lu,\"duty_scale\":799,\"uart_baud\":115200},"
        "\"wifi\":{\"state\":\"%s\",\"ssid\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"channel\":%u,\"hostname\":\"%s\"},"
        "\"system\":{\"firmware\":\"%s\",\"project\":\"%s\",\"build\":\"%s %s\",\"idf\":\"%s\","
        "\"chip\":\"ESP32-S3 rev %d.%d, %d cores\",\"uptime_s\":%lld,\"free_heap\":%lu,\"min_free_heap\":%lu,"
        "\"reset_reason\":\"%s\",\"partition\":\"%s\",\"ota_running\":%s}}",
        st->ctrl.auto_mode ? "AUTO" : "HOST", ctrl_state_name(st->ctrl.state), st->ctrl.setpoint,
        st->ctrl.filtered, st->raw_speed, st->ctrl.duty, st->ctrl.duty / 7.99f,
        st->ctrl.lidar_wanted ? "true" : "false", (unsigned long)st->ctrl.restarts,
        st->valid_pct, (unsigned long)st->frames_per_s, st->revolutions_per_s, (unsigned long)st->total_valid,
        (unsigned long)st->total_speed_errors, (unsigned long)st->total_bad_checksum,
        (unsigned long)st->total_raw_bytes,
        c->setpoint, c->kp, c->ki, c->kd, c->duty_min, c->duty_max, c->slew, c->auto_start ? "true" : "false", (unsigned long)c->pwm_hz,
        st->lidar_rx_gpio, CONFIG_LDS_ROBOT_RX_GPIO, CONFIG_LDS_PWM_GPIO,
        (unsigned long)motor_pwm_get_frequency(), (unsigned long)motor_pwm_get_period_ticks(),
        wifi.state, wifi.ssid, wifi.ip, wifi.rssi, wifi.channel, CONFIG_LDS_HOSTNAME,
        app->version, app->project_name, app->date, app->time, app->idf_ver,
        chip.revision / 100, chip.revision % 100, chip.cores, esp_timer_get_time() / 1000000,
        (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size(),
        reset_reason_text(esp_reset_reason()), running ? running->label : "?", s_ota_running ? "true" : "false");
}

static void push_task(void *arg)
{
    static uint8_t scan_frame[4 + SCAN_POINTS * 2];
    static char json[1400];
    uint16_t distance[SCAN_POINTS];
    live_status_t st;
    const uint32_t magic = SCAN_MAGIC;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(PUSH_INTERVAL_MS));
        if (s_ota_running || !any_ws_client()) {
            continue;
        }
        scan_snapshot(distance, &st);
        memcpy(scan_frame, &magic, 4);
        memcpy(scan_frame + 4, distance, sizeof(distance));  /* little-endian u16 per degree, 0 = none */
        queue_broadcast(HTTPD_WS_TYPE_BINARY, scan_frame, sizeof(scan_frame));

        int n = build_status_json(json, sizeof(json), &st);
        if (n > 0 && n < (int)sizeof(json)) {
            queue_broadcast(HTTPD_WS_TYPE_TEXT, json, n);
        }
    }
}

void web_start(QueueHandle_t command_queue)
{
    s_commands = command_queue;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = MAX_CLIENTS;
    config.lru_purge_enable = true;
    config.core_id = 0;
    config.stack_size = 6144;
    if (httpd_start(&s_server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return;
    }
    const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_get},
        {.uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true},
        {.uri = "/ota", .method = HTTP_POST, .handler = ota_post},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(s_server, &uris[i]);
    }
    xTaskCreatePinnedToCore(push_task, "web_push", 4096, NULL, 5, NULL, 0);
}
