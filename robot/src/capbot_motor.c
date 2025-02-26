/**
 * @file capbot_motor.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot motor control
 * @date 2024-12-23
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>

#include <stdbool.h>

#include "capbot_motor.h"

#define PWM_PERIOD 20000000 /** ns */

#define DEGREES_PER_REVOLUTION 360
#define HALL_TICKS_PER_REVOLUTION 1380

/** @brief Time delta over which motor RPM is calculated */
K_MUTEX_DEFINE(motor_timing_mutex);
#define T_RPM_UPDATE_DELTA K_MSEC(20)

// -----------------------------------------------------------------------------
// Static objects for motors
// -----------------------------------------------------------------------------

/** @brief Holds a motor's GPIO handles */
struct motor_gpio {
    const struct pwm_dt_spec in_a;
    const struct pwm_dt_spec in_b;
    const struct gpio_dt_spec hall_c1;
    const struct gpio_dt_spec hall_c2;
};

/** @brief Wraps everything related to one motor together*/
struct motor {
    /** @brief Motor's IO handles */
    const struct motor_gpio *gpio;
    /**  @brief Callback info for motor's hall interrupt */
    struct gpio_callback hall_cb_data;
    /** @brief To keep track of a motor's steps: counts up/down every hall interrupt */
    int64_t step_count;
    /** @brief Step count at last calculation */
    int64_t step_prev;
    /** @brief Step count delta at last calculation */
    int64_t step_delta;
    /** @brief Time at last calculation */
    uint64_t time_prev;
    /** @brief Time delta at last calculation */
    uint64_t time_delta;
};

// -----------------------------------------------------------------------------

static const struct motor_gpio mfr_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfr), hall_gpios, 1),
};

static struct motor mfr = {
    .gpio = &mfr_io,
    .step_count = 0,
    .step_prev = 0,
    .step_delta = 0,
    .time_prev = 0,
    .time_delta = 0,
};

static const struct motor_gpio mfl_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mfl), hall_gpios, 1),
};

static struct motor mfl = {
    .gpio = &mfl_io,
    .step_count = 0,
    .step_prev = 0,
    .step_delta = 0,
    .time_prev = 0,
    .time_delta = 0,
};

static const struct motor_gpio mbr_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbr), hall_gpios, 1),
};

static struct motor mbr = {
    .gpio = &mbr_io,
    .step_count = 0,
    .step_prev = 0,
    .step_delta = 0,
    .time_prev = 0,
    .time_delta = 0,
};

static const struct motor_gpio mbl_io = {
    .in_a = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), 0),
    .in_b = PWM_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), 1),
    .hall_c1 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), hall_gpios, 0),
    .hall_c2 = GPIO_DT_SPEC_GET_BY_IDX(DT_ALIAS(mbl), hall_gpios, 1),
};

static struct motor mbl = {
    .gpio = &mbl_io,
    .step_count = 0,
    .step_prev = 0,
    .step_delta = 0,
    .time_prev = 0,
    .time_delta = 0,
};

// -----------------------------------------------------------------------------
// Motor control: private functions
// -----------------------------------------------------------------------------

/**
 * @brief Initialize a motor's IO
 *
 * @param motor The motor
 * @param hall_isr Callback for motor's hall interrupt
 * @retval <0: An error occurred
 */
static int init_motor_io(struct motor *motor, gpio_callback_handler_t hall_isr) {
    int err = 0;

    err |= !pwm_is_ready_dt(&motor->gpio->in_a);
    err |= !pwm_is_ready_dt(&motor->gpio->in_b);

    if (err) {
        return err;
    }

    err |= !gpio_is_ready_dt(&motor->gpio->hall_c1);
    err |= !gpio_is_ready_dt(&motor->gpio->hall_c2);

    if (err) {
        return err;
    }

    err |= gpio_pin_configure_dt(&motor->gpio->hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(&motor->gpio->hall_c2, GPIO_INPUT);

    if (err) {
        return err;
    }

    gpio_init_callback(&motor->hall_cb_data, hall_isr, BIT(motor->gpio->hall_c2.pin));
    err |= gpio_pin_interrupt_configure_dt(&motor->gpio->hall_c2, GPIO_INT_EDGE_BOTH);
    err |= gpio_add_callback_dt(&motor->gpio->hall_c2, &motor->hall_cb_data);

    return err;
}

/**
 * @brief Put a PWM signal on a motor's pins
 *
 * @param motor Pointer to motor's wrapper struct
 * @param direction Direction to turn (`1` = forwards, `0` = backwards)
 * @param pulse_width The width of the PWM pulse
 */
static inline void set_motor_pwm(const struct motor *motor, bool direction, uint32_t pulse_width) {
    pwm_set_dt(&(motor->gpio->in_a), PWM_PERIOD, 0);
    pwm_set_dt(&(motor->gpio->in_b), PWM_PERIOD, 0);
}

/** @brief Stop a given motor */
static inline void set_motor_stop(const struct motor *motor) {
    pwm_set_dt(&(motor->gpio->in_a), PWM_PERIOD, 0);
    pwm_set_dt(&(motor->gpio->in_b), PWM_PERIOD, 0);
}

/** @brief Update motor timing info */
static inline void update_motor_timing(struct motor *motor, uint64_t time_curr) {
    if (k_mutex_lock(&motor_timing_mutex, K_FOREVER) == 0) {
        motor->step_delta = motor->step_count - motor->step_prev;
        motor->step_prev = motor->step_count;
        motor->time_delta = time_curr - motor->time_prev;
        motor->time_prev = time_curr;
        k_mutex_unlock(&motor_timing_mutex);
    }
}

/** @brief Get motor's angle in degrees */
static inline int get_motor_angle(struct motor *motor) {
    return motor->step_count * DEGREES_PER_REVOLUTION / HALL_TICKS_PER_REVOLUTION;
}

/** @brief Get motor's speed in RPM */
static inline int get_motor_rpm(struct motor *motor) {
    if (k_mutex_lock(&motor_timing_mutex, T_RPM_UPDATE_DELTA)) {
        // Error: could not lock mutex
        return -1;
    }
    int64_t step_d = motor->step_delta;
    uint64_t time_d = motor->time_delta;
    k_mutex_unlock(&motor_timing_mutex);

    return (step_d * 60000) / (int64_t)k_ticks_to_ms_near64(time_d * HALL_TICKS_PER_REVOLUTION);
}

// -----------------------------------------------------------------------------
// Hall sensor interrupts
// -----------------------------------------------------------------------------

/** @brief Generic hall sensor interrupt handler (called by motor specific ones) */
static inline void hall_isr(struct motor *motor) {
    int c1 = gpio_pin_get_dt(&motor->gpio->hall_c1);
    int c2 = gpio_pin_get_dt(&motor->gpio->hall_c2);

    if (c1 == c2) {
        motor->step_count++;
    } else {
        motor->step_count--;
    }
}

void mfr_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mfr); }
void mfl_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mfl); }
void mbr_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mbr); }
void mbl_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mbl); }

// -----------------------------------------------------------------------------
// RPM calculation thread
// -----------------------------------------------------------------------------

#define T_RPM_STACKSIZE 256
#define T_RPM_PRIORITY 7
#define T_RPM_OPTIONS 0

/**
 * @brief Periodically update timing info for calculating (average) motor
 * speeds
 */
void t_rpm_entry_point(void *, void *, void *) {
    for (;;) {
        update_motor_timing(&mfr, k_uptime_ticks());
        update_motor_timing(&mfl, k_uptime_ticks());
        update_motor_timing(&mbr, k_uptime_ticks());
        update_motor_timing(&mbl, k_uptime_ticks());

        k_sleep(T_RPM_UPDATE_DELTA);
    }
}

// Stack area for RPM calculation thread
K_THREAD_STACK_DEFINE(t_rpm_stack, T_RPM_STACKSIZE);

/** @brief Data for RPM calculation thread */
struct k_thread t_rpm_data;

// -----------------------------------------------------------------------------
// Motor control: API (public functions)
// -----------------------------------------------------------------------------

int cb_motor_init(void) {
    int err = 0;

    // Initialize front left motor
    err = init_motor_io(&mfl, mfl_isr);
    if (err) {
        return err;
    }
    set_motor_stop(&mfl);

    // Initialize front right motor
    err = init_motor_io(&mfr, mfr_isr);
    if (err) {
        return err;
    }
    set_motor_stop(&mfr);

    // Initialize back left motor
    err = init_motor_io(&mbl, mbl_isr);
    if (err) {
        return err;
    }
    set_motor_stop(&mbl);

    // Initialize back right motor
    err = init_motor_io(&mbr, mbr_isr);
    if (err) {
        return err;
    }
    set_motor_stop(&mbr);

    // Start motor timing thread
    k_thread_create(&t_rpm_data, t_rpm_stack, T_RPM_STACKSIZE, t_rpm_entry_point, NULL, NULL, NULL,
                    T_RPM_PRIORITY, T_RPM_OPTIONS, K_NO_WAIT);

    return 0;
}

void cb_set_motor_speed(cb_motor_speed_t *speeds) {
    set_motor_pwm(&mfl, 0, 0);
    set_motor_pwm(&mfr, 0, 0);
    set_motor_pwm(&mbl, 0, 0);
    set_motor_pwm(&mbr, 0, 0);
}

void cb_stop(void) {
    set_motor_stop(&mfl);
    set_motor_stop(&mfr);
    set_motor_stop(&mbl);
    set_motor_stop(&mbr);
}

void cb_get_motor_angle(cb_motor_angle_t *angles) {
    angles->front_left = get_motor_angle(&mfl);
    angles->front_right = get_motor_angle(&mfr);
    angles->back_left = get_motor_angle(&mbl);
    angles->back_right = get_motor_angle(&mbr);
}

void cb_get_motor_speed(cb_motor_speed_t *speeds) {
    speeds->front_left = get_motor_rpm(&mfl);
    speeds->front_right = get_motor_rpm(&mfr);
    speeds->back_left = get_motor_rpm(&mbl);
    speeds->back_right = get_motor_rpm(&mbr);
}
