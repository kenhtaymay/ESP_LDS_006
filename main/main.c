/*
 * LDS-006 motor controller, ESP32-S3 / ESP-IDF.
 *
 * Core 1: one control task owns the motor. It reads the lidar TX on UART1, feeds
 * each frame's speed field to the PID controller, drives the MCPWM output, keeps
 * the latest revolution for the web map, optionally watches the robot->lidar line
 * on UART2 for "startlds$"/"stoplds$", and handles command lines from USB and
 * from the web page (via a queue), so controller state is never shared.
 * Core 0: WiFi (SmartConfig), mDNS, HTTP/WebSocket server, OTA.
 *
 * Command lines (USB Serial/JTAG or the web page's WebSocket; compatible with
 * lds_tuner.py and MotorController.cs, which drive HOST mode with P/S and send F/A):
 *   G                 AUTO mode: firmware runs kick -> ramp -> PID (see controller.h)
 *   P <duty>          HOST mode, open-loop duty 0..799
 *   S <speed>         setpoint (HOST: enables PID bumplessly)
 *   K <kp> <ki> <kd>  PID gains
 *   L <min> <max>     PID duty limits
 *   R <steps>         slew limit per PID update
 *   W                 save config (and AUTO as start-on-boot) to NVS
 *   X                 stop, leave AUTO
 *   C                 forget WiFi credentials and start SmartConfig
 *   D                 diagnose the lidar RX pin (level samples, edges, byte counts)
 *   Q <hz>            PWM frequency (default 2000, saved with W); lower = fewer switching edges
 *   M [1|0]           motor on (AUTO: kick -> ramp -> PID, runs without waiting for the
 *                     robot's startlds$) / off; no argument toggles. BOOT short press = M.
 *                     Only the PWM output is affected: the robot line (GPIO17) stays RX-only.
 *   U <gpio>          move the lidar RX to another GPIO (saved to NVS)
 *   F <speed>, A      ignored / keepalive (speed comes from the lidar UART here)
 *   ?                 status
 * USB output: READY, T <ms> <setpoint> <filtered> <duty> <state> at 10 Hz,
 *   V valid=<n> errors=<n> pct=<p> about once a second, I wifi=... on changes,
 *   OK/ERR/STAT lines.
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "controller.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_ota_ops.h"
#include "esp_rom_sys.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lidar.h"
#include "motor_pwm.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "scan.h"
#include "sdkconfig.h"
#include "web.h"
#include "wifi.h"

#define LIDAR_UART UART_NUM_1
#define ROBOT_UART UART_NUM_2
#define LIDAR_RX_BUFFER 4096
#define TELEMETRY_MS 100
#define STATS_WINDOW_MS 1000
#define BUTTON_HOLD_MS 8000   /* forget WiFi */
#define BUTTON_WARN_MS 3000   /* announce the pending WiFi reset */
#define BUTTON_SHORT_MS 1000  /* shorter = motor toggle */
#define NVS_NAMESPACE "ldsmotor"
#define NVS_KEY "cfg"
#define CONFIG_VERSION 2  /* 2: added pwm_hz */

typedef struct {
    uint32_t version;
    ctrl_config_t ctrl;
} stored_config_t;

static ctrl_config_t s_config;
static lidar_parser_t s_parser;
static QueueHandle_t s_commands;
static uint16_t s_raw_speed;
static uint32_t s_raw_bytes;  /* everything UART1 received, parsed or not */
static int s_lidar_rx_gpio = CONFIG_LDS_LIDAR_RX_GPIO;  /* changeable at run time with U, kept in NVS */

static void set_rx_pin(uart_port_t port, int rx_gpio);
static void allow_motor(void);

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/* Non-blocking USB output: when no host is attached (installed in the robot) the
 * data is dropped so the control loop never waits on USB. */
static void out(const char *fmt, ...)
{
    if (!usb_serial_jtag_is_connected()) {
        return;
    }
    char buf[320];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0) {
        usb_serial_jtag_write_bytes(buf, n < (int)sizeof(buf) ? n : (int)sizeof(buf) - 1, 0);
    }
}

static void load_config(void)
{
    ctrl_default_config(&s_config);
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    stored_config_t stored;
    size_t len = sizeof(stored);
    if (nvs_get_blob(h, NVS_KEY, &stored, &len) == ESP_OK && len == sizeof(stored) &&
        stored.version == CONFIG_VERSION) {
        s_config = stored.ctrl;
    }
    int8_t rx_gpio;
    if (nvs_get_i8(h, "rx_gpio", &rx_gpio) == ESP_OK) {
        s_lidar_rx_gpio = rx_gpio;
    }
    nvs_close(h);
}

/* Pins that must not become the lidar RX: strapping (0, 3, 45, 46), USB (19, 20),
 * flash/PSRAM (26-32), UART0 console (43, 44) and the PWM output. */
static bool rx_pin_allowed(int gpio)
{
    if (!GPIO_IS_VALID_GPIO(gpio) || gpio == CONFIG_LDS_PWM_GPIO || gpio == CONFIG_LDS_ROBOT_RX_GPIO) {
        return false;
    }
    switch (gpio) {
    case 0: case 3: case 19: case 20: case 43: case 44: case 45: case 46:
        return false;
    default:
        return gpio < 26 || gpio > 32;
    }
}

static bool change_rx_pin(int gpio)
{
    if (!rx_pin_allowed(gpio)) {
        return false;
    }
    gpio_reset_pin(s_lidar_rx_gpio);  /* release the old pin back to a plain input */
    s_lidar_rx_gpio = gpio;
    set_rx_pin(LIDAR_UART, gpio);
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i8(h, "rx_gpio", (int8_t)gpio);
        nvs_commit(h);
        nvs_close(h);
    }
    return true;
}

static bool save_config(void)
{
    stored_config_t stored = {.version = CONFIG_VERSION, .ctrl = s_config};
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    bool ok = nvs_set_blob(h, NVS_KEY, &stored, sizeof(stored)) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    return ok;
}

static void print_status(void)
{
    ctrl_status_t st;
    ctrl_get_status(&st);
    wifi_info_t wifi;
    wifi_get_info(&wifi);
    out("STAT mode=%s state=%s setpoint=%.0f duty=%.1f top=799 kp=%.6f ki=%.6f kd=%.6f min=%.0f max=%.0f "
        "slew=%.1f filtered=%.0f wanted=%d restarts=%lu auto_start=%d lidar_rx=%d robot_rx=%d pwm=%d "
        "frames=%lu fb=%lu bad=%lu raw_bytes=%lu wifi=%s ip=%s\n",
        st.auto_mode ? "AUTO" : "HOST", ctrl_state_name(st.state), st.setpoint, st.duty, s_config.kp,
        s_config.ki, s_config.kd, s_config.duty_min, s_config.duty_max, s_config.slew, st.filtered,
        st.lidar_wanted, (unsigned long)st.restarts, s_config.auto_start, s_lidar_rx_gpio,
        CONFIG_LDS_ROBOT_RX_GPIO, CONFIG_LDS_PWM_GPIO, (unsigned long)s_parser.valid,
        (unsigned long)s_parser.speed_errors, (unsigned long)s_parser.bad_checksum, (unsigned long)s_raw_bytes,
        wifi.state,
        wifi.ip[0] ? wifi.ip : "-");
}

/* Samples the lidar RX pin directly (the GPIO input still reads while the pin is
 * routed to UART1) to tell "no wire / stuck level" from "signal but no frames". */
static void diagnose_rx_pin(void)
{
    uint32_t high = 0, low = 0, edges = 0;
    int last = gpio_get_level(s_lidar_rx_gpio);
    for (int i = 0; i < 20000; i++) {  /* 100 ms at 5 us; one 115200 bit is 8.7 us */
        int level = gpio_get_level(s_lidar_rx_gpio);
        if (level) {
            high++;
        } else {
            low++;
        }
        edges += level != last;
        last = level;
        esp_rom_delay_us(5);
    }
    const char *verdict = edges > 20 ? "signal present"
                          : high ? "idle high: no data on this pin"
                                 : "stuck low: wrong wire, no common GND, or pin driven by something else";
    out("DIAG gpio=%d high=%lu low=%lu edges=%lu raw_bytes=%lu frames=%lu bad=%lu -> %s\n",
        s_lidar_rx_gpio, (unsigned long)high, (unsigned long)low, (unsigned long)edges,
        (unsigned long)s_raw_bytes, (unsigned long)(s_parser.valid + s_parser.speed_errors),
        (unsigned long)s_parser.bad_checksum, verdict);
}

static void handle_line(char *s, uint32_t now)
{
    while (*s == ' ') {
        s++;
    }
    if (!*s) {
        return;
    }
    char cmd = (*s >= 'a' && *s <= 'z') ? *s - 32 : *s;
    float a[3] = {0};
    int n = sscanf(s + 1, "%f %f %f", &a[0], &a[1], &a[2]);
    if (n < 0) {
        n = 0;
    }
    ctrl_note_command(now);

    switch (cmd) {
    case 'F':
    case 'A':
        break;
    case 'G':
        ctrl_cmd_auto(now);
        out("OK G\n");
        break;
    case 'P':
        if (n != 1) {
            out("ERR P <duty>\n");
            break;
        }
        ctrl_cmd_duty(a[0], now);
        out("OK P %.1f\n", a[0] < 0 ? 0 : (a[0] > MOTOR_DUTY_SCALE ? MOTOR_DUTY_SCALE : a[0]));
        break;
    case 'S':
        if (n != 1) {
            out("ERR S <speed>\n");
            break;
        }
        ctrl_cmd_setpoint(a[0], now);
        out("OK S %.0f\n", a[0]);
        break;
    case 'K':
        if (n != 3 || a[0] < 0 || a[1] < 0 || a[2] < 0) {
            out("ERR K <kp> <ki> <kd>\n");
            break;
        }
        s_config.kp = a[0];
        s_config.ki = a[1];
        s_config.kd = a[2];
        print_status();
        break;
    case 'L':
        if (n != 2 || a[0] < 0 || a[1] > MOTOR_DUTY_SCALE || a[0] > a[1]) {
            out("ERR L <min> <max>\n");
            break;
        }
        s_config.duty_min = a[0];
        s_config.duty_max = a[1];
        print_status();
        break;
    case 'R':
        if (n != 1 || a[0] <= 0) {
            out("ERR R <steps>\n");
            break;
        }
        s_config.slew = a[0];
        print_status();
        break;
    case 'W':
        out(save_config() ? "OK W saved\n" : "ERR W nvs\n");
        break;
    case 'X':
        ctrl_cmd_stop(now);
        out("OK X\n");
        break;
    case 'D':
        diagnose_rx_pin();
        break;
    case '~':
        out("I BOOT held: keep holding to 8 s to forget WiFi and start SmartConfig, release to cancel\n");
        break;
    case 'Q':
        if (n != 1 || !motor_pwm_set_frequency((uint32_t)a[0])) {
            out("ERR Q <hz>: 1221..800000\n");
            break;
        }
        s_config.pwm_hz = motor_pwm_get_frequency();  /* W saves it */
        out("OK Q PWM %lu Hz (%lu ticks)\n", (unsigned long)motor_pwm_get_frequency(),
            (unsigned long)motor_pwm_get_period_ticks());
        break;
    case 'M': {
        /* motor only (PWM): on = AUTO sequence kick -> ramp -> PID, off = stop */
        ctrl_status_t st;
        ctrl_get_status(&st);
        bool running = st.auto_mode || st.state != CTRL_OFF;
        bool on = n == 1 ? a[0] != 0 : !running;  /* no argument: toggle */
        if (on) {
            allow_motor();
            ctrl_cmd_auto(now);
        } else {
            ctrl_cmd_stop(now);
        }
        out("OK M %s\n", on ? "on (AUTO)" : "off");
        break;
    }
    case 'U':
        if (n != 1 || !change_rx_pin((int)a[0])) {
            out("ERR U <gpio>: not allowed (strapping 0/3/45/46, USB 19/20, flash 26-32, UART0 43/44, PWM, robot RX)\n");
            break;
        }
        out("OK U lidar RX now on GPIO%d (saved)\n", s_lidar_rx_gpio);
        break;
    case 'C':
        wifi_forget_and_smartconfig();
        out("OK C smartconfig\n");
        break;
    case '?':
        print_status();
        break;
    default:
        out("ERR unknown command: %s\n", s);
    }
}

static void poll_usb(uint32_t now)
{
    static char line[WEB_COMMAND_LEN];
    static size_t len;
    uint8_t buf[64];
    int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), 0);
    for (int i = 0; i < n; i++) {
        char c = (char)buf[i];
        if (c == '\n' || c == '\r') {
            line[len] = 0;
            handle_line(line, now);
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        }
    }
}

static void poll_web(uint32_t now)
{
    char line[WEB_COMMAND_LEN];
    while (xQueueReceive(s_commands, line, 0) == pdTRUE) {
        line[sizeof(line) - 1] = 0;
        handle_line(line, now);
    }
}

#if CONFIG_LDS_ROBOT_RX_GPIO >= 0
/* The robot sends startlds$ once. If this board resets on its own (OTA, crash) the
 * robot will not repeat it, so remember the last command across soft resets; RTC
 * memory is cleared on power-up, when the robot boots and sends it again anyway. */
#define WANTED_MAGIC 0x57414E54
static RTC_NOINIT_ATTR uint32_t s_wanted_magic;
static RTC_NOINIT_ATTR uint32_t s_wanted_saved;

static void remember_wanted(bool wanted)
{
    s_wanted_saved = wanted;
    s_wanted_magic = WANTED_MAGIC;
    ctrl_set_lidar_wanted(wanted);
}

static bool restore_wanted(void)
{
    return esp_reset_reason() != ESP_RST_POWERON && s_wanted_magic == WANTED_MAGIC && s_wanted_saved;
}

#endif

/* M 1 / the BOOT button run the motor even if the robot has not sent startlds$ yet;
 * the robot's own startlds$/stoplds$ keep updating this afterwards. */
static void allow_motor(void)
{
#if CONFIG_LDS_ROBOT_RX_GPIO >= 0
    remember_wanted(true);
#else
    ctrl_set_lidar_wanted(true);
#endif
}

/* Watches the robot->lidar line for "startlds$" / "stoplds$". */
/* The robot drives the motor: startlds$ also switches to AUTO (the controller boots in
 * HOST mode unless AUTO was saved with W, and then only recorded "wanted" without
 * spinning up); stoplds$ stops it through the wanted gate. */
static void poll_robot(uint32_t now)
{
#if CONFIG_LDS_ROBOT_RX_GPIO >= 0
    static char cmd[16];
    static size_t len;
    uint8_t buf[32];
    int n = uart_read_bytes(ROBOT_UART, buf, sizeof(buf), 0);
    for (int i = 0; i < n; i++) {
        char c = (char)buf[i];
        /* sliding window of the last 15 characters: noise bytes (e.g. at power-up)
         * can no longer fill the buffer and truncate the next command */
        if (len == sizeof(cmd) - 1) {
            memmove(cmd, cmd + 1, len - 1);
            len--;
        }
        cmd[len++] = c;
        if (c != '$') {
            continue;
        }
        cmd[len] = 0;
        if (strstr(cmd, "startlds")) {
            remember_wanted(true);
            ctrl_status_t st;
            ctrl_get_status(&st);
            if (!st.auto_mode) {
                ctrl_cmd_auto(now);
            }
            out("I robot startlds$ -> motor AUTO\n");
        } else if (strstr(cmd, "stoplds")) {
            remember_wanted(false);
            out("I robot stoplds$ -> motor stop\n");
        }
        len = 0;
    }
#else
    (void)now;
#endif
}

static void on_frame(const lidar_frame_t *frame, uint32_t now)
{
    s_raw_speed = frame->speed;
    ctrl_on_speed(frame->speed, now);
    int angle = lidar_frame_angle(frame);
    if (angle < 0) {
        return;
    }
    for (int j = 0; j < 4; j++) {
        bool ok = lidar_reading_valid(frame->distance[j], frame->strength[j]);
        scan_set(angle + j, ok ? frame->distance[j] : 0);
    }
}

/* Publishes status for the web page and prints USB telemetry. */
static void report(uint32_t now)
{
    static uint32_t last, window_start, window_valid, window_errors;
    static float valid_pct, rev_per_s;
    static uint32_t frames_per_s;
    static char last_wifi[48];

    if (now - window_start >= STATS_WINDOW_MS) {
        uint32_t v = s_parser.valid - window_valid, e = s_parser.speed_errors - window_errors;
        float seconds = (now - window_start) / 1000.0f;
        frames_per_s = (uint32_t)((v + e) / seconds);
        rev_per_s = v / 90.0f / seconds;  /* 90 measurement frames per revolution */
        valid_pct = v + e ? 100.0f * v / (v + e) : 0;
        if (v + e) {
            out("V valid=%lu errors=%lu pct=%.1f\n", (unsigned long)v, (unsigned long)e, valid_pct);
        }
        window_start = now;
        window_valid = s_parser.valid;
        window_errors = s_parser.speed_errors;
    }

    if (now - last < TELEMETRY_MS) {
        return;
    }
    last = now;
    live_status_t st = {
        .config = s_config,
        .valid_pct = valid_pct,
        .frames_per_s = frames_per_s,
        .revolutions_per_s = rev_per_s,
        .raw_speed = s_raw_speed,
        .total_valid = s_parser.valid,
        .total_speed_errors = s_parser.speed_errors,
        .total_bad_checksum = s_parser.bad_checksum,
        .total_raw_bytes = s_raw_bytes,
        .lidar_rx_gpio = s_lidar_rx_gpio,
    };
    ctrl_get_status(&st.ctrl);
    scan_publish_status(&st);
    /* same 6 fields as the Uno firmware so the PC tools parse it */
    out("T %lu %.0f %.0f %.1f %s\n", (unsigned long)now, st.ctrl.setpoint, st.ctrl.filtered, st.ctrl.duty,
        ctrl_state_name(st.ctrl.state));

    wifi_info_t wifi;
    wifi_get_info(&wifi);
    char text[48];
    snprintf(text, sizeof(text), "%s %s", wifi.state, wifi.ip);
    if (strcmp(text, last_wifi) != 0) {
        strcpy(last_wifi, text);
        out("I wifi=%s ip=%s hostname=%s.local\n", wifi.state, wifi.ip[0] ? wifi.ip : "-", CONFIG_LDS_HOSTNAME);
    }
}

static void control_task(void *arg)
{
    uint8_t buf[128];
    for (;;) {
        /* blocks up to 2 ms: at ~9.7 KB/s that is ~20 bytes per pass */
        int n = uart_read_bytes(LIDAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(2));
        uint32_t now = now_ms();
        if (n > 0) {
            s_raw_bytes += n;
        }
        for (int i = 0; i < n; i++) {
            lidar_frame_t frame;
            if (lidar_parser_feed(&s_parser, buf[i], &frame)) {
                on_frame(&frame, now);
            }
        }
        poll_robot(now);
        poll_usb(now);
        poll_web(now);
        ctrl_tick(now);
        report(now);
    }
}

static void queue_command(const char *line)
{
    char cmd[WEB_COMMAND_LEN] = {0};
    strncpy(cmd, line, sizeof(cmd) - 1);
    xQueueSend(s_commands, cmd, 0);
}

/* BOOT button: a short press (< 1 s) toggles the motor (M); holding it for 8 s
 * forgets WiFi and starts SmartConfig (C). A press that is already down at boot
 * (reset/download mode, or the button held while restarting) is ignored until the
 * button has been released once: holding BOOT through a restart once wiped the
 * stored WiFi with the old 3 s hold. */
static void button_task(void *arg)
{
    uint32_t held = 0;
    bool armed = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        bool pressed = gpio_get_level(CONFIG_LDS_BUTTON_GPIO) == 0;
        if (!armed) {
            armed = !pressed;
            continue;
        }
        if (pressed) {
            held += 50;
            if (held == BUTTON_WARN_MS) {
                queue_command("~");  /* control task reports it; it owns the USB output */
            } else if (held == BUTTON_HOLD_MS) {
                queue_command("C");
            }
        } else {
            if (held >= 50 && held < BUTTON_SHORT_MS) {
                queue_command("M");
            }
            held = 0;
        }
    }
}

/* RX only: this board never drives the shared lidar/robot lines. The pull-up keeps
 * an unconnected pin idle-high (and lets the D diagnostic tell "open" from "held low"). */
static void set_rx_pin(uart_port_t port, int rx_gpio)
{
    ESP_ERROR_CHECK(uart_set_pin(port, UART_PIN_NO_CHANGE, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    gpio_pullup_en(rx_gpio);
}

static void init_uart(uart_port_t port, int rx_gpio, int rx_buffer)
{
    const uart_config_t config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(port, rx_buffer, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(port, &config));
    set_rx_pin(port, rx_gpio);
}

void app_main(void)
{
    ESP_ERROR_CHECK(motor_pwm_init(CONFIG_LDS_PWM_GPIO));  /* output held low from here on */

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* e.g. NVS left behind by the board's previous firmware */
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    load_config();
    if (!motor_pwm_set_frequency(s_config.pwm_hz)) {
        s_config.pwm_hz = motor_pwm_get_frequency();  /* out-of-range saved value: keep the default */
    }

    usb_serial_jtag_driver_config_t usb_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_config.tx_buffer_size = 4096;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_config));

    init_uart(LIDAR_UART, s_lidar_rx_gpio, LIDAR_RX_BUFFER);
#if CONFIG_LDS_ROBOT_RX_GPIO >= 0
    init_uart(ROBOT_UART, CONFIG_LDS_ROBOT_RX_GPIO, 256);
    ctrl_set_lidar_wanted(restore_wanted());  /* otherwise wait for the robot's startlds$ */
#endif

    s_commands = xQueueCreate(8, WEB_COMMAND_LEN);
    lidar_parser_init(&s_parser);
    ctrl_init(&s_config, now_ms());
#if CONFIG_LDS_ROBOT_RX_GPIO >= 0
    if (restore_wanted()) {
        ctrl_cmd_auto(now_ms());  /* soft reset while the robot had the lidar running: resume */
    }
#endif
    out("READY lds_motor_idf v2 top=799\n");
    xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, 10, NULL, 1);

#if CONFIG_LDS_BUTTON_GPIO >= 0
    gpio_config_t button = {
        .pin_bit_mask = 1ULL << CONFIG_LDS_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&button);
    xTaskCreatePinnedToCore(button_task, "button", 2048, NULL, 2, NULL, 0);
#endif

    wifi_start(CONFIG_LDS_HOSTNAME);
    web_start(s_commands);

    /* With bootloader rollback enabled, a freshly OTA-flashed image must confirm
     * itself; reaching here means the motor control, WiFi and web server started. */
    esp_ota_mark_app_valid_cancel_rollback();
}
