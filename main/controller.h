#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Motor speed controller. All functions are called from the single control task.
 *
 * AUTO mode runs the sequence proven on the Uno + PC setup:
 *   KICK   open-loop duty 720, +20 every 3 s (max 799) until lidar frames arrive
 *          (the lidar is silent while its head is not spinning)
 *   SETTLE 1 s, so the median filter reports the kick speed
 *   RAMP   PID with the setpoint ramped down from that speed at 2500/s
 *          (always approach the target from above; dropping straight to a base
 *          duty undershot below the accepted window)
 *   PID    hold the setpoint; no frames for 1 s -> KICK again (the duty minimum
 *          is below the running-stall point)
 *   REST   no frames within 25 s of kicking: motor off for 30 s, then retry
 * HOST mode is driven by the PC tools with P (open-loop duty) and S (PID setpoint)
 * and stops the motor if no command arrives for 3 s. */

typedef struct {
    float setpoint;
    float kp, ki, kd;
    float duty_min, duty_max;  /* 0..799 */
    float slew;                /* max duty change per PID update */
    bool auto_start;           /* enter AUTO at boot */
    uint32_t pwm_hz;           /* PWM frequency; applied by main.c */
} ctrl_config_t;

typedef enum {
    CTRL_OFF,
    CTRL_KICK,
    CTRL_SETTLE,
    CTRL_RAMP,
    CTRL_PID,
    CTRL_REST,
    CTRL_MANUAL,    /* HOST: open-loop duty */
    CTRL_HOST_PID,  /* HOST: PID on the host's setpoint */
} ctrl_state_t;

typedef struct {
    bool auto_mode;
    ctrl_state_t state;
    float setpoint;  /* the setpoint in effect (ramp value while ramping) */
    float filtered;  /* median-filtered speed */
    float duty;
    bool lidar_wanted;
    uint32_t restarts;
    uint32_t outliers;  /* speed samples dropped as implausible (line noise) */
} ctrl_status_t;

void ctrl_default_config(ctrl_config_t *cfg);
void ctrl_init(ctrl_config_t *cfg, uint32_t now_ms);

/* Speed field of every checksum-valid frame (0xFB frames included). */
void ctrl_on_speed(uint16_t speed, uint32_t now_ms);
void ctrl_tick(uint32_t now_ms);

void ctrl_cmd_auto(uint32_t now_ms);
void ctrl_cmd_duty(float duty, uint32_t now_ms);
void ctrl_cmd_setpoint(float setpoint, uint32_t now_ms);
void ctrl_cmd_stop(uint32_t now_ms);
void ctrl_note_command(uint32_t now_ms);  /* keepalive for HOST mode */
void ctrl_set_lidar_wanted(bool wanted);

void ctrl_get_status(ctrl_status_t *st);
const char *ctrl_state_name(ctrl_state_t state);
