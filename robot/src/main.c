/**
 * @file main.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Robot control via BLE GATT service
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "ble_rcs.h"
#include "capbot.h"

#define ROBOT_ID 9999

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// Status led thread
//
// This thread makes the status leds blink in different patterns based on the
// current BLE status
// -----------------------------------------------------------------------------

/** @brief Enum to represent BLE status */
typedef enum ble_status_e {
    BLE_INACTIVE,
    BLE_ADVERTISING,
    BLE_CONNECTED,
    BLE_ERROR,
} ble_status_t;

/**
 * @brief Determine pattern mask for BLE status led
 *
 * @param status Status to get pattern for
 * @return uint16_t The pattern
 */
uint16_t ble_led_pattern(ble_status_t status) {
    switch (status) {
    case BLE_ADVERTISING:
        // Low frequency blink
        return 0b1111111100000000;
    case BLE_CONNECTED:
        // Off: save energy
        return 0b0000000000000000;
    case BLE_ERROR:
        // High frequency blink
        return 0b1010101010101010;
    case BLE_INACTIVE:
        // Long-long
        return 0b0011111100111111;
    default:
        LOG_WRN("Could not determine BLE status led pattern, using ERROR "
                "pattern instead");
        return ble_led_pattern(BLE_ERROR);
    }
}

// -----------------------------------------------------------------------------

/**
 * @brief Global static to hold BLE status.
 *
 * Always use `ble_status_get()` and `ble_status_set()` for accessing!
 */
static ble_status_t ble_status_g;

// BLE status info should be mutexed since it's shared between threads
K_MUTEX_DEFINE(ble_status_mutex);

/** @brief Get the current BLE status */
ble_status_t ble_status_get(void) {
    static ble_status_t clone = BLE_ERROR;
    if (k_mutex_lock(&ble_status_mutex, K_MSEC(5))) {
        LOG_WRN("Using old BLE status: could not lock mutex");
    } else {
        clone = ble_status_g;
        k_mutex_unlock(&ble_status_mutex);
    }
    return clone;
}

/** @brief Set the current BLE status */
void ble_status_set(ble_status_t status) {
    if (k_mutex_lock(&ble_status_mutex, K_MSEC(10))) {
        LOG_ERR("Failed to update BLE status: could not lock mutex");
    } else {
        ble_status_g = status;
        k_mutex_unlock(&ble_status_mutex);
    }
}

// -----------------------------------------------------------------------------

/** @brief Entry point of status led thread */
void t_status_led_ep(void *, void *, void *) {
    const cb_led_t status_led = CB_D15;
    uint16_t led_pattern = 0;
    uint8_t pattern_index = 0;

    for (;;) {
        // Get pattern based on current BLE status
        led_pattern = ble_led_pattern(ble_status_get());
        // Update led based on patters and index
        int err = cb_led_write(status_led, (led_pattern >> pattern_index) & 0b1);
        if (err)
            LOG_WRN("Error during BLE status led update");
        // Update index
        pattern_index = (pattern_index + 1) % (sizeof(led_pattern) * 8);
        // Sleep before next update => circa one pattern cycle per second
        k_sleep(K_MSEC(62));
    }
}

// Stack area for the status led thread
K_THREAD_STACK_DEFINE(t_status_led_stack, 256);

// Status led thead data
struct k_thread t_status_led_data;

// -----------------------------------------------------------------------------
// BLE connection management
// -----------------------------------------------------------------------------

#define BLE_DEVICE_NAME "CapBot"
#define BLE_DEVICE_NAME_LEN (sizeof(BLE_DEVICE_NAME) - 1)

/** @brief BLE Advertisement data */
static const struct bt_data adv_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

/** @brief BLE Scan response data */
static const struct bt_data rsp_data[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, BLE_DEVICE_NAME, BLE_DEVICE_NAME_LEN),
    BT_DATA_BYTES(BT_DATA_UUID128_SOME, BT_UUID_RCS_VAL),
};

/** @brief Start BLE advertising */
int bt_advertise(void) {
    if (bt_le_adv_start(BT_LE_ADV_CONN, adv_data, ARRAY_SIZE(adv_data), rsp_data,
                        ARRAY_SIZE(rsp_data))) {
        LOG_ERR("BLE advertising failed to start");
        ble_status_set(BLE_ERROR);
        return -1;
    }
    ble_status_set(BLE_ADVERTISING);
    LOG_INF("BLE advertising started");
    return 0;
}

/** @brief On BLE connected callback */
static void bt_on_connected(struct bt_conn *conn, uint8_t err) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    if (err) {
        LOG_ERR("Failed to connect to %s (%u)", addr, err);
        return;
    }
    LOG_INF("Connected: %s", addr);

    ble_status_set(BLE_CONNECTED);
}

/** @brief On BLE disconnected callback */
static void bt_on_disconnected(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected: %s (reason 0x%02x)", addr, reason);

    bt_advertise();
}

/** @brief BLE connection callbacks */
BT_CONN_CB_DEFINE(connection_cb) = {
    .connected = bt_on_connected,
    .disconnected = bt_on_disconnected,
};

// -----------------------------------------------------------------------------
// System initialization
// -----------------------------------------------------------------------------

int sys_init(void) {
    // Initialize CapBot library
    int cb_init_err = 0;
    cb_init_err |= cb_led_init();
    cb_init_err |= cb_btn_init();
    cb_init_err |= cb_measure_init();
    cb_init_err |= cb_motor_init();
    if (cb_init_err) {
        LOG_ERR("CapBot initialization failed");
        return -1;
    }
    LOG_INF("CapBot library initialization was successful");

    // Set RCS identifier
    rcs_set_id((uint32_t) ROBOT_ID);

    // Start status led thread
    ble_status_set(BLE_INACTIVE);
    k_thread_create(&t_status_led_data, t_status_led_stack,
                    K_THREAD_STACK_SIZEOF(t_status_led_stack), t_status_led_ep, NULL, NULL, NULL,
                    10, K_USER, K_NO_WAIT);

    // Initialize bluetooth
    if (bt_enable(NULL)) {
        LOG_ERR("Bluetooth initialization failed");
        ble_status_set(BLE_ERROR);
        return -1;
    }
    LOG_INF("Bluetooth initialization was successful");

    if (bt_advertise()) {
        return -1;
    }

    return 0;
}

SYS_INIT(sys_init, APPLICATION, 5);
