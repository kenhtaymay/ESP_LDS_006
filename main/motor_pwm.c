#include "motor_pwm.h"

#include "driver/mcpwm_prelude.h"
#include "esp_check.h"

/* MCPWM counts at 80 MHz. At the default 20 kHz a period is 4000 ticks: 5x finer
 * than the Uno's 800 steps, which matters because one Uno step moved the head speed
 * by ~650 while the accepted speed window is only ~2250 wide. The frequency can be
 * lowered at run time (fewer switching edges = less noise coupled into the lidar
 * UART line); the 16-bit period limits it to >= 1221 Hz. */
#define PWM_RESOLUTION_HZ 80000000
#define PWM_DEFAULT_FREQ_HZ 20000
#define PWM_MAX_PERIOD_TICKS 65535

static const char *TAG = "motor_pwm";

static mcpwm_timer_handle_t s_timer;
static mcpwm_cmpr_handle_t s_comparator;
static mcpwm_gen_handle_t s_generator;
static bool s_forced_low;
static uint32_t s_period_ticks = PWM_RESOLUTION_HZ / PWM_DEFAULT_FREQ_HZ;
static float s_duty;

esp_err_t motor_pwm_init(int gpio)
{
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks = s_period_ticks,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .flags.update_period_on_empty = true,  /* frequency changes take effect at a period boundary */
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_config, &s_timer), TAG, "timer");

    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t operator_config = {.group_id = 0};
    ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &oper), TAG, "operator");
    ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(oper, s_timer), TAG, "connect");

    mcpwm_comparator_config_t comparator_config = {.flags.update_cmp_on_tez = true};
    ESP_RETURN_ON_ERROR(mcpwm_new_comparator(oper, &comparator_config, &s_comparator), TAG, "comparator");

    mcpwm_generator_config_t generator_config = {.gen_gpio_num = gpio};
    ESP_RETURN_ON_ERROR(mcpwm_new_generator(oper, &generator_config, &s_generator), TAG, "generator");

    /* high at the start of each period, low when the counter reaches the compare value */
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_timer_event(s_generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)),
        TAG, "timer action");
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_action_on_compare_event(s_generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_comparator, MCPWM_GEN_ACTION_LOW)),
        TAG, "compare action");

    ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_comparator, 0), TAG, "compare");
    /* a compare value of 0 still emits a pulse at each period start, so hold low instead */
    ESP_RETURN_ON_ERROR(mcpwm_generator_set_force_level(s_generator, 0, true), TAG, "force low");
    s_forced_low = true;

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_timer), TAG, "enable");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_timer, MCPWM_TIMER_START_NO_STOP), TAG, "start");
    return ESP_OK;
}

void motor_pwm_set(float duty)
{
    if (duty < 0) {
        duty = 0;
    }
    if (duty > MOTOR_DUTY_SCALE) {
        duty = MOTOR_DUTY_SCALE;
    }
    s_duty = duty;
    uint32_t ticks = (uint32_t)(duty / MOTOR_DUTY_SCALE * s_period_ticks + 0.5f);

    if (ticks == 0) {
        if (!s_forced_low) {
            mcpwm_generator_set_force_level(s_generator, 0, true);
            s_forced_low = true;
        }
        return;
    }
    /* compare == period is never reached by the up-counter, so the output stays high: 100% */
    mcpwm_comparator_set_compare_value(s_comparator, ticks);
    if (s_forced_low) {
        mcpwm_generator_set_force_level(s_generator, -1, true);
        s_forced_low = false;
    }
}

bool motor_pwm_set_frequency(uint32_t hz)
{
    if (hz == 0) {
        return false;
    }
    uint32_t period = PWM_RESOLUTION_HZ / hz;
    if (period < 100 || period > PWM_MAX_PERIOD_TICKS) {
        return false;
    }
    /* shrinking the period below a pending compare value would glitch: clear it first */
    mcpwm_comparator_set_compare_value(s_comparator, 0);
    if (mcpwm_timer_set_period(s_timer, period) != ESP_OK) {
        return false;
    }
    s_period_ticks = period;
    motor_pwm_set(s_duty);  /* same duty on the new period */
    return true;
}

uint32_t motor_pwm_get_frequency(void)
{
    return PWM_RESOLUTION_HZ / s_period_ticks;
}

uint32_t motor_pwm_get_period_ticks(void)
{
    return s_period_ticks;
}
