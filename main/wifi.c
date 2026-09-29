#include "wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "mdns.h"

#define WIFI_CONNECT_TIMEOUT_US (30 * 1000000LL)

static const char *TAG = "wifi";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static esp_netif_t *s_netif;
static const char *s_state = "off";
static char s_ip[16];
static bool s_smartconfig_running;
static bool s_mdns_started;
static const char *s_hostname;
static esp_timer_handle_t s_timeout_timer;

static void set_state(const char *state)
{
    taskENTER_CRITICAL(&s_lock);
    s_state = state;
    taskEXIT_CRITICAL(&s_lock);
}

static void start_smartconfig(void)
{
    if (s_smartconfig_running) {
        return;
    }
    ESP_LOGW(TAG, "starting SmartConfig");
    esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS);
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    if (esp_smartconfig_start(&cfg) == ESP_OK) {
        s_smartconfig_running = true;
        set_state("smartconfig");
    }
}

static void stop_smartconfig(void)
{
    if (s_smartconfig_running) {
        esp_smartconfig_stop();
        s_smartconfig_running = false;
    }
}

/* No IP within the timeout after (re)starting: fall back to SmartConfig. */
static void on_connect_timeout(void *arg)
{
    if (!s_ip[0]) {
        start_smartconfig();
    }
}

static void start_mdns(void)
{
    if (s_mdns_started || mdns_init() != ESP_OK) {
        return;
    }
    mdns_hostname_set(s_hostname);
    mdns_instance_name_set("LDS-006 lidar motor controller");
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    s_mdns_started = true;
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        wifi_config_t cfg;
        if (esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK && cfg.sta.ssid[0]) {
            set_state("connecting");
            esp_wifi_connect();
            esp_timer_start_once(s_timeout_timer, WIFI_CONNECT_TIMEOUT_US);
        } else {
            start_smartconfig();
        }
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        taskENTER_CRITICAL(&s_lock);
        s_ip[0] = 0;
        taskEXIT_CRITICAL(&s_lock);
        if (!s_smartconfig_running) {
            /* SmartConfig scans channels itself; reconnecting would disturb it */
            set_state("connecting");
            esp_wifi_connect();
            if (!esp_timer_is_active(s_timeout_timer)) {
                esp_timer_start_once(s_timeout_timer, WIFI_CONNECT_TIMEOUT_US);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        taskENTER_CRITICAL(&s_lock);
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&event->ip_info.ip));
        taskEXIT_CRITICAL(&s_lock);
        esp_timer_stop(s_timeout_timer);
        if (!s_smartconfig_running) {
            set_state("connected");
        }
        start_mdns();
    } else if (base == SC_EVENT && id == SC_EVENT_GOT_SSID_PSWD) {
        smartconfig_event_got_ssid_pswd_t *event = (smartconfig_event_got_ssid_pswd_t *)data;
        wifi_config_t cfg = {0};
        memcpy(cfg.sta.ssid, event->ssid, sizeof(cfg.sta.ssid));
        memcpy(cfg.sta.password, event->password, sizeof(cfg.sta.password));
        cfg.sta.bssid_set = event->bssid_set;
        if (cfg.sta.bssid_set) {
            memcpy(cfg.sta.bssid, event->bssid, sizeof(cfg.sta.bssid));
        }
        ESP_LOGW(TAG, "SmartConfig got SSID %s", (char *)cfg.sta.ssid);
        esp_wifi_disconnect();
        esp_wifi_set_config(WIFI_IF_STA, &cfg);  /* stored in flash (WIFI_STORAGE_FLASH) */
        esp_wifi_connect();
    } else if (base == SC_EVENT && id == SC_EVENT_SEND_ACK_DONE) {
        stop_smartconfig();
        set_state(s_ip[0] ? "connected" : "connecting");
    }
}

void wifi_start(const char *hostname)
{
    s_hostname = hostname;
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(s_netif, hostname);

    const esp_timer_create_args_t timer_args = {.callback = on_connect_timeout, .name = "wifi_timeout"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_timeout_timer));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_forget_and_smartconfig(void)
{
    wifi_config_t empty = {0};
    esp_timer_stop(s_timeout_timer);
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &empty);
    taskENTER_CRITICAL(&s_lock);
    s_ip[0] = 0;
    taskEXIT_CRITICAL(&s_lock);
    stop_smartconfig();
    start_smartconfig();
}

void wifi_get_info(wifi_info_t *info)
{
    memset(info, 0, sizeof(*info));
    taskENTER_CRITICAL(&s_lock);
    info->state = s_state;
    memcpy(info->ip, s_ip, sizeof(info->ip));
    taskEXIT_CRITICAL(&s_lock);

    wifi_ap_record_t ap;
    if (info->ip[0] && esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        memcpy(info->ssid, ap.ssid, sizeof(info->ssid) - 1);
        info->rssi = ap.rssi;
        info->channel = ap.primary;
    } else {
        wifi_config_t cfg;
        if (esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK) {
            memcpy(info->ssid, cfg.sta.ssid, sizeof(info->ssid) - 1);
        }
    }
}
