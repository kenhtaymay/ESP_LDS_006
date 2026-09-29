#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/* Duty is expressed on the 0..799 scale of the original Uno firmware so the PC tools
 * (lds_tuner.py, MotorController.cs) and the tuned limits keep their meaning. */
#define MOTOR_DUTY_SCALE 799.0f

esp_err_t motor_pwm_init(int gpio);

/* Clamps to 0..MOTOR_DUTY_SCALE; 0 forces the output low (no glitch pulses). */
void motor_pwm_set(float duty);

/* Changes the PWM frequency at run time (1221..800000 Hz; default 20 kHz), keeping
 * the duty. Lower frequencies mean fewer switching edges and less noise coupled into
 * the lidar line. Returns false if out of range. */
bool motor_pwm_set_frequency(uint32_t hz);
uint32_t motor_pwm_get_frequency(void);
uint32_t motor_pwm_get_period_ticks(void);
