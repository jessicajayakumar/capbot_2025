/**
 * @file ble_rcs.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @date 2025-01-02
 */

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>

#include "ble_rcs.h"
#include "capbot.h"

LOG_MODULE_REGISTER(robot_control_service, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// Motor drive characteristic
// -----------------------------------------------------------------------------

#define T_DRIVE_TIMEOUT_PRIORITY 7
K_THREAD_STACK_DEFINE(t_drive_timeout_stack, 512);
static struct k_thread t_drive_timeout;
static struct {
    cb_motor_speed_t speeds;
    k_timeout_t duration;
} t_drive_timeout_data;

void t_drive_timeout_ep(void *, void *, void *) {
    LOG_DBG("Setting motors: {%d, %d, %d, %d}", t_drive_timeout_data.speeds.front_left,
            t_drive_timeout_data.speeds.front_right, t_drive_timeout_data.speeds.back_left,
            t_drive_timeout_data.speeds.back_right);
    cb_set_motor_speed(&t_drive_timeout_data.speeds);
    LOG_DBG("Waiting for timeout");
    k_sleep(t_drive_timeout_data.duration);
    LOG_DBG("Timeout reached, stopping motors");
    cb_stop();
}

void drive_with_timeout(cb_motor_speed_t *speeds, k_timeout_t timeout) {
    static k_tid_t drive_tid = 0;
    if (drive_tid != 0) {
        LOG_DBG("Aborting previous drive thread");
        k_thread_abort(drive_tid);
    }
    LOG_DBG("Creating drive thread");
    t_drive_timeout_data.speeds.front_left = speeds->front_left;
    t_drive_timeout_data.speeds.front_right = speeds->front_right;
    t_drive_timeout_data.speeds.back_left = speeds->back_left;
    t_drive_timeout_data.speeds.back_right = speeds->back_right;
    t_drive_timeout_data.duration = timeout;
    drive_tid = k_thread_create(&t_drive_timeout, t_drive_timeout_stack,
                                K_THREAD_STACK_SIZEOF(t_drive_timeout_stack), t_drive_timeout_ep,
                                NULL, NULL, NULL, T_DRIVE_TIMEOUT_PRIORITY, 0, K_NO_WAIT);
}

static ssize_t rcs_drive(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
                         uint16_t len, uint16_t offset, uint8_t flags) {
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    if (len != sizeof(rcs_drive_t)) {
        LOG_WRN("Incorrect data length for drive attribute");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    if (offset != 0) {
        LOG_WRN("Incorrect data offset for drive attribute");
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    rcs_drive_t pkt = *((rcs_drive_t *)buf);
    cb_motor_speed_t speeds = {
        .front_left = pkt.fl, .front_right = pkt.fr, .back_left = pkt.bl, .back_right = pkt.br};
    drive_with_timeout(&speeds, K_MSEC(pkt.dur));

    return len;
}

// -----------------------------------------------------------------------------
// Motor speed characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_speed_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    cb_motor_speed_t speeds;
    cb_get_motor_speed(&speeds);

    LOG_INF("Reporting motor speeds: {%d %d %d %d}", speeds.front_left, speeds.front_right,
            speeds.back_left, speeds.back_right);
    rcs_speed_t pkt = {.fl = speeds.front_left,
                       .fr = speeds.front_right,
                       .bl = speeds.back_left,
                       .br = speeds.back_right};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Motor angle characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_angle_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                              uint16_t len, uint16_t offset) {
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    cb_motor_angle_t angles;
    cb_get_motor_angle(&angles);

    LOG_INF("Reporting motor angles: {%d %d %d %d}", angles.front_left, angles.front_right,
            angles.back_left, angles.back_right);
    rcs_angle_t pkt = {.fl = angles.front_left,
                       .fr = angles.front_right,
                       .bl = angles.back_left,
                       .br = angles.back_right};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Voltage measurement characteristic
// -----------------------------------------------------------------------------

static ssize_t rcs_volt_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                             uint16_t len, uint16_t offset) {
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    int vcap = cb_measure_vcap();
    LOG_DBG("measured voltage: %u", vcap);

    rcs_volt_t pkt = {.volt = vcap};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

// -----------------------------------------------------------------------------
// Identifier characteristic
// -----------------------------------------------------------------------------

/** @brief Global identifier. Never update directly, always use rcs_set_id! */
static uint32_t identifier = 0xffffffff;

static ssize_t rcs_id_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf,
                           uint16_t len, uint16_t offset) {
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    rcs_id_t pkt = {.id = identifier};
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &pkt, sizeof(pkt));
}

void rcs_set_id(uint32_t id) {
    // FIXME: mutex this access
    identifier = id;
}

// -----------------------------------------------------------------------------
// Robot Control Service (RCS)
// -----------------------------------------------------------------------------

BT_GATT_SERVICE_DEFINE(
    // Robot Control Service
    rcs_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_RCS),
    // Motor drive characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_DRIVE, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE, NULL, rcs_drive, NULL),
    // Motor speed characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_SPEED, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_speed_read,
                           NULL, NULL),
    // Motor angle characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_ANGLE, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_angle_read,
                           NULL, NULL),
    // Voltage measurement characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_VOLT, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_volt_read,
                           NULL, NULL),
    // Identifier characteristic
    BT_GATT_CHARACTERISTIC(BT_UUID_RCS_ID, BT_GATT_CHRC_READ, BT_GATT_PERM_READ, rcs_id_read, NULL,
                           NULL));
