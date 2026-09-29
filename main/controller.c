#include "controller.h"

#include "motor_pwm.h"

#define KICK_DUTY 720.0f
#define KICK_STEP 20.0f
#define KICK_MAX 799.0f
#define KICK_STEP_MS 3000
#define KICK_TIMEOUT_MS 25000
#define REST_MS 30000
#define SETTLE_MS 1000
#define STALL_MS 1000
#define FRAMES_FLOWING_MS 300
#define HOST_TIMEOUT_MS 3000
#define RAMP_RATE 2500.0f  /* setpoint units per second */
#define MEDIAN_N 5
#define OUTLIER_FRACTION 0.4f
#define OUTLIER_RUN_LIMIT 5

static ctrl_config_t *s_cfg;

static bool s_auto;
static bool s_lidar_wanted = true;
static ctrl_state_t s_state = CTRL_OFF;
static uint32_t s_state_since, s_last_frame, s_last_command, s_next_kick_step, s_ramp_last;
static float s_duty, s_kick_duty, s_target, s_ramp_setpoint;
static uint32_t s_restarts;

/* PID */
static float s_integral, s_last_measured;
static bool s_have_last;
static uint32_t s_last_pid;

/* median filter over new speed values */
static uint16_t s_samples[MEDIAN_N];
static int s_sample_count, s_sample_next;
static int s_last_speed = -1;
static float s_filtered;
static uint32_t s_outliers;
static int s_outlier_run;

static void set_duty(float duty)
{
    if (duty < 0) {
        duty = 0;
    }
    if (duty > MOTOR_DUTY_SCALE) {
        duty = MOTOR_DUTY_SCALE;
    }
    s_duty = duty;
    motor_pwm_set(duty);
}

static void enter(ctrl_state_t state, uint32_t now)
{
    s_state = state;
    s_state_since = now;
}

static void stop_motor(uint32_t now)
{
    set_duty(0);
    s_integral = 0;
    enter(CTRL_OFF, now);
}

static void start_kick(uint32_t now)
{
    s_kick_duty = KICK_DUTY;
    set_duty(s_kick_duty);
    s_next_kick_step = now + KICK_STEP_MS;
    enter(CTRL_KICK, now);
}

static void reset_filter(void)
{
    s_sample_count = 0;
    s_sample_next = 0;
    s_last_speed = -1;
}

/* Median of the last MEDIAN_N speed values; removes the once-per-revolution index
 * spike (the speed field reads about +33% then -20% once per turn). */
static float median_speed(void)
{
    uint16_t sorted[MEDIAN_N];
    for (int i = 0; i < s_sample_count; i++) {
        sorted[i] = s_samples[i];
    }
    for (int i = 1; i < s_sample_count; i++) {
        uint16_t v = sorted[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > v) {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = v;
    }
    return sorted[s_sample_count / 2];
}

/* Close the loop starting from the current duty (bumpless). */
static void begin_pid(float target)
{
    s_target = target;
    s_integral = s_duty < s_cfg->duty_min ? s_cfg->duty_min : s_duty;
    s_have_last = false;
    s_last_pid = 0;
}

static void pid_update(float setpoint, float measured, uint32_t now)
{
    float dt = s_last_pid ? (now - s_last_pid) / 1000.0f : 0;
    s_last_pid = now;
    if (dt <= 0 || dt > 0.5f) {
        s_last_measured = measured;
        s_have_last = true;
        return;
    }

    float error = setpoint - measured;
    s_integral += s_cfg->ki * error * dt;
    if (s_integral < s_cfg->duty_min) {
        s_integral = s_cfg->duty_min;
    }
    if (s_integral > s_cfg->duty_max) {
        s_integral = s_cfg->duty_max;
    }
    /* derivative on measurement: no kick when the setpoint changes */
    float derivative = s_have_last ? -(measured - s_last_measured) / dt : 0;
    s_last_measured = measured;
    s_have_last = true;

    float out = s_integral + s_cfg->kp * error + s_cfg->kd * derivative;
    if (out < s_cfg->duty_min) {
        out = s_cfg->duty_min;
    }
    if (out > s_cfg->duty_max) {
        out = s_cfg->duty_max;
    }
    if (out > s_duty + s_cfg->slew) {
        out = s_duty + s_cfg->slew;
    }
    if (out < s_duty - s_cfg->slew) {
        out = s_duty - s_cfg->slew;
    }
    set_duty(out);
}

void ctrl_default_config(ctrl_config_t *cfg)
{
    /* values tuned on the Uno + IRF520N setup; re-tune after changing the MOSFET */
    *cfg = (ctrl_config_t) {
        .setpoint = 22700,
        .kp = 0.0015f,
        .ki = 0.006f,
        .kd = 0,
        .duty_min = 580,
        .duty_max = 760,
        .slew = 2,
        .auto_start = false,
        /* 20 kHz switching corrupted 72% of lidar frames and every robot command on the
         * ESP32 driver wiring; 2 kHz caught 5/5 robot commands (audible whine though) */
        .pwm_hz = 2000,
    };
}

void ctrl_init(ctrl_config_t *cfg, uint32_t now)
{
    s_cfg = cfg;
    s_auto = cfg->auto_start;
    s_last_command = now;
    stop_motor(now);
}

void ctrl_on_speed(uint16_t speed, uint32_t now)
{
    s_last_frame = now;
    /* frames repeat the same speed several times; each new value is one sample */
    if (speed == s_last_speed) {
        return;
    }
    s_last_speed = speed;
    /* Electrical noise on the lidar line lets corrupted frames through the weak
     * additive checksum (seen: 52 000 and 3 300 while holding 22 700). The normal
     * once-per-turn index spike is about +33% / -20%, so drop anything further off. */
    if (s_sample_count == MEDIAN_N && s_filtered > 0 &&
        (speed > s_filtered * (1 + OUTLIER_FRACTION) || speed < s_filtered * (1 - OUTLIER_FRACTION))) {
        s_outliers++;
        /* several in a row is a real change (e.g. belt slip), not noise: start over */
        if (++s_outlier_run < OUTLIER_RUN_LIMIT) {
            return;
        }
        s_sample_count = 0;
        s_sample_next = 0;
    }
    s_outlier_run = 0;
    s_samples[s_sample_next] = speed;
    s_sample_next = (s_sample_next + 1) % MEDIAN_N;
    if (s_sample_count < MEDIAN_N) {
        s_sample_count++;
    }
    s_filtered = median_speed();
    if (s_sample_count < MEDIAN_N) {
        return;
    }

    if (s_state == CTRL_RAMP) {
        pid_update(s_ramp_setpoint, s_filtered, now);
    } else if (s_state == CTRL_PID || s_state == CTRL_HOST_PID) {
        pid_update(s_target, s_filtered, now);
    }
}

void ctrl_tick(uint32_t now)
{
    if (!s_auto) {
        if ((s_state == CTRL_MANUAL || s_state == CTRL_HOST_PID) && now - s_last_command > HOST_TIMEOUT_MS) {
            stop_motor(now);
        }
        return;
    }
    if (!s_lidar_wanted) {
        if (s_state != CTRL_OFF) {
            stop_motor(now);
        }
        return;
    }

    bool frames_flowing = now - s_last_frame < FRAMES_FLOWING_MS;
    switch (s_state) {
    case CTRL_OFF:
        start_kick(now);
        break;
    case CTRL_KICK:
        if (frames_flowing) {
            reset_filter();
            enter(CTRL_SETTLE, now);
        } else if (now - s_state_since > KICK_TIMEOUT_MS) {
            set_duty(0);
            enter(CTRL_REST, now);
        } else if ((int32_t)(now - s_next_kick_step) >= 0 && s_kick_duty < KICK_MAX) {
            s_kick_duty = s_kick_duty + KICK_STEP > KICK_MAX ? KICK_MAX : s_kick_duty + KICK_STEP;
            set_duty(s_kick_duty);
            s_next_kick_step = now + KICK_STEP_MS;
        }
        break;
    case CTRL_SETTLE:
        if (now - s_state_since >= SETTLE_MS) {
            s_ramp_setpoint = s_filtered > s_cfg->setpoint ? s_filtered : s_cfg->setpoint;
            s_ramp_last = now;
            begin_pid(s_cfg->setpoint);
            enter(CTRL_RAMP, now);
        }
        break;
    case CTRL_RAMP:
        s_ramp_setpoint -= RAMP_RATE * (now - s_ramp_last) / 1000.0f;
        s_ramp_last = now;
        if (s_ramp_setpoint <= s_cfg->setpoint) {
            s_ramp_setpoint = s_cfg->setpoint;
            s_target = s_cfg->setpoint;
            enter(CTRL_PID, now);
        }
        break;
    case CTRL_PID:
        s_target = s_cfg->setpoint;  /* follows setpoint changes */
        break;
    case CTRL_REST:
        if (now - s_state_since >= REST_MS) {
            start_kick(now);
        }
        break;
    default:
        break;
    }

    if ((s_state == CTRL_RAMP || s_state == CTRL_PID) && now - s_last_frame > STALL_MS) {
        s_restarts++;
        start_kick(now);
    }
}

void ctrl_cmd_auto(uint32_t now)
{
    s_auto = true;
    s_cfg->auto_start = true;
    stop_motor(now);  /* ctrl_tick starts the sequence */
}

void ctrl_cmd_duty(float duty, uint32_t now)
{
    s_auto = false;
    s_cfg->auto_start = false;
    set_duty(duty);
    enter(s_duty > 0 ? CTRL_MANUAL : CTRL_OFF, now);
}

void ctrl_cmd_setpoint(float setpoint, uint32_t now)
{
    if (s_auto) {
        s_cfg->setpoint = setpoint;
        return;
    }
    if (s_state != CTRL_HOST_PID) {
        begin_pid(setpoint);
        enter(CTRL_HOST_PID, now);
    }
    s_target = setpoint;
}

void ctrl_cmd_stop(uint32_t now)
{
    s_auto = false;
    s_cfg->auto_start = false;
    stop_motor(now);
}

void ctrl_note_command(uint32_t now)
{
    s_last_command = now;
}

void ctrl_set_lidar_wanted(bool wanted)
{
    s_lidar_wanted = wanted;
}

void ctrl_get_status(ctrl_status_t *st)
{
    st->auto_mode = s_auto;
    st->state = s_state;
    st->setpoint = s_state == CTRL_RAMP ? s_ramp_setpoint : (s_auto ? s_cfg->setpoint : s_target);
    st->filtered = s_filtered;
    st->duty = s_duty;
    st->lidar_wanted = s_lidar_wanted;
    st->restarts = s_restarts;
    st->outliers = s_outliers;
}

const char *ctrl_state_name(ctrl_state_t state)
{
    static const char *const names[] = {"OFF", "KICK", "SETTLE", "RAMP", "PID", "REST", "MANUAL", "PID"};
    return state < sizeof(names) / sizeof(names[0]) ? names[state] : "?";
}
