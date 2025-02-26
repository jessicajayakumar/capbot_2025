/**
 * @file ble_rcs.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @date 2025-01-02
 */

#ifndef RCS_H
#define RCS_H

#include <stdint.h>
#include <zephyr/bluetooth/uuid.h>

// -----------------------------------------------------------------------------
// Robot Control Service (RCS)
// -----------------------------------------------------------------------------

#define BT_UUID_RCS_VAL BT_UUID_128_ENCODE(0x00000030, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_RCS BT_UUID_DECLARE_128(BT_UUID_RCS_VAL)

// -----------------------------------------------------------------------------
// Motor drive characteristic
// -----------------------------------------------------------------------------

#define BT_UUID_RCS_DRIVE_VAL BT_UUID_128_ENCODE(0x00000031, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_RCS_DRIVE BT_UUID_DECLARE_128(BT_UUID_RCS_DRIVE_VAL)

/** @brief Data format for motor drive characteristic */
typedef struct {
    /** @brief Target speed (RPM) for front left motor */
    int8_t fl;
    /** @brief Target speed (RPM) for front right motor */
    int8_t fr;
    /** @brief Target speed (RPM) for back left motor */
    int8_t bl;
    /** @brief Target speed (RPM) for back right motor */
    int8_t br;
    /** @brief The amount of milliseconds to keep target speeds before stopping */
    uint32_t dur;
} rcs_drive_t;

// -----------------------------------------------------------------------------
// Motor speed characteristic
// -----------------------------------------------------------------------------

#define BT_UUID_RCS_SPEED_VAL BT_UUID_128_ENCODE(0x00000032, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_RCS_SPEED BT_UUID_DECLARE_128(BT_UUID_RCS_SPEED_VAL)

/** @brief Data format for motor speed characteristic */
typedef struct {
    /** @brief Measured speed (RPM) of front left motor */
    int8_t fl;
    /** @brief Measured speed (RPM) of front right motor */
    int8_t fr;
    /** @brief Measured speed (RPM) of back left motor */
    int8_t bl;
    /** @brief Measured speed (RPM) of back right motor */
    int8_t br;
} rcs_speed_t;

// -----------------------------------------------------------------------------
// Motor angle characteristic
// -----------------------------------------------------------------------------

#define BT_UUID_RCS_ANGLE_VAL BT_UUID_128_ENCODE(0x00000033, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_RCS_ANGLE BT_UUID_DECLARE_128(BT_UUID_RCS_ANGLE_VAL)

/** @brief Data format for motor angle characteristic */
typedef struct {
    /** @brief Measured angle (degrees) of front left motor */
    int32_t fl;
    /** @brief Measured angle (degrees) of front right motor */
    int32_t fr;
    /** @brief Measured angle (degrees) of back left motor */
    int32_t bl;
    /** @brief Measured angle (degrees) of back right motor */
    int32_t br;
} rcs_angle_t;

// -----------------------------------------------------------------------------
// Voltage measurement characteristic
// -----------------------------------------------------------------------------

#define BT_UUID_RCS_VOLT_VAL BT_UUID_128_ENCODE(0x00000034, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_RCS_VOLT BT_UUID_DECLARE_128(BT_UUID_RCS_VOLT_VAL)

/** @brief Data format for voltage measurement characteristic */
typedef struct {
    /** @brief Measured voltage (mV) */
    uint16_t volt;
} rcs_volt_t;

// -----------------------------------------------------------------------------
// Identifier characteristic
// -----------------------------------------------------------------------------

#define BT_UUID_RCS_ID_VAL BT_UUID_128_ENCODE(0x00000035, 0x0000, 0x1000, 0x8000, 0x00805f9b34fb)
#define BT_UUID_RCS_ID BT_UUID_DECLARE_128(BT_UUID_RCS_ID_VAL)

/** @brief Data format for identifier characteristic */
typedef struct {
    /** @brief Identifier */
    uint32_t id;
} rcs_id_t;

/** @brief Update identifier */
void rcs_set_id(uint32_t id);

#endif /* RCS_H */
