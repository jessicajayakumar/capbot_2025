/**
 * @file capbot_motor.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief CapBot library for motor control
 */

#ifndef CAPBOT_MOTOR_H
#define CAPBOT_MOTOR_H

/** @brief Motor speeds (rpm) */
typedef struct {
    /** @brief Front left motor speed (rpm)*/
    int front_left;
    /** @brief Front right motor speed (rpm)*/
    int front_right;
    /** @brief Back left motor speed (rpm)*/
    int back_left;
    /** @brief Back right motor speed (rpm)*/
    int back_right;
} cb_motor_speed_t;

/** @brief Motor angle (degrees) */
typedef struct {
    /** @brief Front left motor angle (degrees)*/
    int front_left;
    /** @brief Front right motor angle (degrees)*/
    int front_right;
    /** @brief Back left motor angle (degrees)*/
    int back_left;
    /** @brief Back right motor angle (degrees)*/
    int back_right;
} cb_motor_angle_t;

/**
 * @brief Initialize motors
 * @details v1.0 equivalent: Motors_init(void)
 * @details v2.* equivalent: fb_motor_init(void)
 */
int cb_motor_init(void);

/**
 * @brief Set individual motor speeds
 * @param speeds struct with the new speeds
 */
void cb_set_motor_speed(cb_motor_speed_t *speeds);

/**
 * @brief Get motor speeds
 * @param speeds struct to populate with the new speeds
 */
void cb_get_motor_speed(cb_motor_speed_t *speeds);

/**
 * @brief Get motor angles
 * @param speeds struct to populate with the new angles
 */
void cb_get_motor_angle(cb_motor_angle_t *angles);

/** @brief Stop the robot */
void cb_stop(void);

#endif /* CAPBOT_MOTOR_H */
