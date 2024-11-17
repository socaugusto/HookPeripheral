/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
#include "uart_async_adapter.h"

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>

#include <zephyr/logging/log.h>

#include <sdc_hci_vs.h>
#include <sdc_hci_cmd_status_params.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME peripheral_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7
#define PRIORITY_RSSI 9

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000
#define SAFETY_LINE DK_LED4
#define RESET_LINE DK_LED2
#define POWER_STEVAL 4

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

static K_SEM_DEFINE(ble_init_ok, 0, 1);
static K_SEM_DEFINE(rssi_sem, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

struct uart_data_t
{
	void *fifo_reserved;
	uint8_t data[UART_BUF_SIZE];
	uint16_t len;
};
static void start_advertising(struct k_work *work);
static void shutdown_steval_power(struct k_work *work);
static K_FIFO_DEFINE(fifo_uart_tx_data);
static K_FIFO_DEFINE(fifo_uart_rx_data);
static K_WORK_DEFINE(start_advertising_worker, start_advertising);
static K_WORK_DEFINE(shutdown_steval_worker, shutdown_steval_power);
static struct bt_le_ext_adv *advCoded;
static struct bt_le_ext_adv *adv1Mb;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

#if CONFIG_BT_NUS_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	static size_t aborted_len;
	struct uart_data_t *buf;
	static uint8_t *aborted_buf;
	static bool disable_req;

	switch (evt->type)
	{
	case UART_TX_DONE:
		LOG_DBG("UART_TX_DONE");
		if ((evt->data.tx.len == 0) ||
			(!evt->data.tx.buf))
		{
			return;
		}

		if (aborted_buf)
		{
			buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
							   data[0]);
			aborted_buf = NULL;
			aborted_len = 0;
		}
		else
		{
			buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t,
							   data[0]);
		}

		k_free(buf);

		buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT);
		if (!buf)
		{
			return;
		}

		if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS))
		{
			LOG_WRN("Failed to send data over UART");
		}

		break;

	case UART_RX_RDY:
		LOG_DBG("UART_RX_RDY");
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]);
		buf->len += evt->data.rx.len;

		if (disable_req)
		{
			return;
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') ||
			(evt->data.rx.buf[buf->len - 1] == '\r'))
		{
			disable_req = true;
			uart_rx_disable(uart);
		}

		break;

	case UART_RX_DISABLED:
		LOG_DBG("UART_RX_DISABLED");
		disable_req = false;

		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
			return;
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data),
					   UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST:
		LOG_DBG("UART_RX_BUF_REQUEST");
		buf = k_malloc(sizeof(*buf));
		if (buf)
		{
			buf->len = 0;
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data));
		}
		else
		{
			LOG_WRN("Not able to allocate UART receive buffer");
		}

		break;

	case UART_RX_BUF_RELEASED:
		LOG_DBG("UART_RX_BUF_RELEASED");
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t,
						   data[0]);

		if (buf->len > 0)
		{
			k_fifo_put(&fifo_uart_rx_data, buf);
		}
		else
		{
			k_free(buf);
		}

		break;

	case UART_TX_ABORTED:
		LOG_DBG("UART_TX_ABORTED");
		if (!aborted_buf)
		{
			aborted_buf = (uint8_t *)evt->data.tx.buf;
		}

		aborted_len += evt->data.tx.len;
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t,
						   data[0]);

		uart_tx(uart, &buf->data[aborted_len],
				buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default:
		break;
	}
}

static void uart_work_handler(struct k_work *item)
{
	struct uart_data_t *buf;

	buf = k_malloc(sizeof(*buf));
	if (buf)
	{
		buf->len = 0;
	}
	else
	{
		LOG_WRN("Not able to allocate UART receive buffer");
		k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
		return;
	}

	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX);
}

static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	if (!device_is_ready(uart))
	{
		return -ENODEV;
	}

	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK))
	{
		err = usb_enable(NULL);
		if (err && (err != -EALREADY))
		{
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	rx = k_malloc(sizeof(*rx));
	if (rx)
	{
		rx->len = 0;
	}
	else
	{
		return -ENOMEM;
	}

	k_work_init_delayable(&uart_work, uart_work_handler);

	if (IS_ENABLED(CONFIG_BT_NUS_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart))
	{
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	err = uart_callback_set(uart, uart_cb, NULL);
	if (err)
	{
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	if (IS_ENABLED(CONFIG_UART_LINE_CTRL))
	{
		LOG_INF("Wait for DTR");
		while (true)
		{
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr)
			{
				break;
			}
			/* Give CPU resources to low priority threads. */
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err)
		{
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err)
		{
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	tx = k_malloc(sizeof(*tx));

	if (tx)
	{
		pos = snprintf(tx->data, sizeof(tx->data),
					   "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data)))
		{
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	}
	else
	{
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err)
	{
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), 50);
	if (err)
	{
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		/* Free the rx buffer only because the tx buffer will be handled in the callback */
		k_free(rx);
	}

	return err;
}

static void update_phy(struct bt_conn *conn)
{
	int err;
	const struct bt_conn_le_phy_param preferred_phy = {
		.options = BT_CONN_LE_PHY_OPT_CODED_S8, // BT_CONN_LE_PHY_OPT_CODED_S8
		.pref_rx_phy = BT_GAP_LE_PHY_CODED,		// BT_GAP_LE_PHY_CODED,
		.pref_tx_phy = BT_GAP_LE_PHY_CODED,		// BT_GAP_LE_PHY_CODED,
	};
	err = bt_conn_le_phy_update(conn, &preferred_phy);
	if (err)
	{
		LOG_ERR("bt_conn_le_phy_update() returned %d", err);
	}
}

const uint8_t remoteAddress[] = {0x07, 0x3C, 0x17, 0x8D, 0x9A, 0xC2};

static void connected(struct bt_conn *conn, uint8_t err)
{
	dk_set_led_on(POWER_STEVAL);
	struct bt_conn_info info;
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (err)
	{
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}
	else
	{
		LOG_INF("Connected %s", addr);
	}

	err = bt_conn_get_info(conn, &info);
	if (err)
	{
		LOG_ERR("bt_conn_get_info() returned %d", err);
		return;
	}
	else
	{
		double connection_interval = info.le.interval * 1.25; // in ms
		uint16_t supervision_timeout = info.le.timeout * 10;  // in ms
		LOG_INF("Connection parameters: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, info.le.latency, supervision_timeout);

		const struct bt_conn_le_phy_info *phy_info;
		phy_info = info.le.phy;

		LOG_INF("Connected: %s, tx_phy %u, rx_phy %u",
				addr, phy_info->tx_phy, phy_info->rx_phy);
	}

	err = bt_le_ext_adv_stop(advCoded);
	if (err)
	{
		LOG_ERR("Failed to stop Coded with error %d", err);
		return;
	}
	err = bt_le_ext_adv_stop(adv1Mb);
	if (err)
	{
		LOG_ERR("Failed to stop Standard with error %d", err);
		return;
	}

	current_conn = bt_conn_ref(conn);
	update_phy(current_conn);

	k_sem_give(&rssi_sem);

	dk_set_led(SAFETY_LINE, 0);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (auth_conn)
	{
		bt_conn_unref(auth_conn);
		auth_conn = NULL;
	}

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}

	dk_set_led(SAFETY_LINE, 1);

	k_work_submit(&start_advertising_worker);
	k_work_submit(&shutdown_steval_worker);
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
							 enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err)
	{
		LOG_INF("Security changed: %s level %u", addr, level);
	}
	else
	{
		LOG_WRN("Security failed: %s level %u err %d", addr,
				level, err);
	}
}
#endif

void on_le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	// PHY Updated
	if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_1M)
	{
		LOG_INF("PHY updated. New PHY: 1M");
	}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_2M)
	{
		LOG_INF("PHY updated. New PHY: 2M");
	}
	else if (param->tx_phy == BT_CONN_LE_TX_POWER_PHY_CODED_S8)
	{
		LOG_INF("PHY updated. New PHY: Long Range");
	}
}

void on_le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	double connection_interval = interval * 1.25; // in ms
	uint16_t supervision_timeout = timeout * 10;  // in ms
	LOG_INF("Connection parameters updated: interval %.2f ms, latency %d intervals, timeout %d ms", connection_interval, latency, supervision_timeout);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = on_le_param_updated,
	.le_phy_updated = on_le_phy_updated,
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	.security_changed = security_changed,
#endif
};

static int create_advertising_coded(void)
{
	int err;
	struct bt_le_adv_param param =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE |
								 BT_LE_ADV_OPT_EXT_ADV |
								 BT_LE_ADV_OPT_CODED,
							 BT_GAP_ADV_FAST_INT_MIN_2,
							 BT_GAP_ADV_FAST_INT_MAX_2,
							 NULL);
	err = bt_le_ext_adv_create(&param, NULL, &advCoded);
	if (err)
	{
		printk("Failed to create advertiser set (err %d)\n", err);
		return err;
	}
	printk("Created adv: %p\n", advCoded);
	err = bt_le_ext_adv_set_data(advCoded, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}
	return 0;
}

static int create_advertising_standard(void)
{
	int err;

	struct bt_le_adv_param param =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONNECTABLE |
								 BT_LE_ADV_OPT_EXT_ADV,
							 BT_GAP_ADV_FAST_INT_MIN_2,
							 BT_GAP_ADV_FAST_INT_MAX_2,
							 NULL);
	err = bt_le_ext_adv_create(&param, NULL, &adv1Mb);
	if (err)
	{
		printk("Failed to create advertiser set (err %d)\n", err);
		return err;
	}
	printk("Created adv: %p\n", adv1Mb);
	err = bt_le_ext_adv_set_data(adv1Mb, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}
	return 0;
}

static void start_advertising(struct k_work *work)
{
	int err;
	err = bt_le_ext_adv_start(advCoded, NULL);
	if (err)
	{
		printk("Failed to start advertising set (err %d)\n", err);
		return;
	}
	printk("Advertiser Coded %p set started\n", advCoded);

	err = bt_le_ext_adv_start(adv1Mb, NULL);
	if (err)
	{
		printk("Failed to start advertising set (err %d)\n", err);
		return;
	}
	printk("Advertiser 1M/2M %p set started\n", adv1Mb);
}

static void shutdown_steval_power(struct k_work *work)
{
	int32_t timeout = 60;
	for (int32_t i = timeout; i > -1; --i)
	{
		k_sleep(K_SECONDS(1));
		if (!i && current_conn == NULL)
		{
			dk_set_led_off(POWER_STEVAL);
			LOG_INF("Shutting down STeval");
		}
		else if (current_conn != NULL)
		{
			LOG_INF("Skip STeval shutdown");
			break;
		}
	}
}

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	auth_conn = bt_conn_ref(conn);

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Passkey for %s: %06u", addr, passkey);
	LOG_INF("Press Button 1 to confirm, Button 2 to reject.");
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing cancelled: %s", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason);
}

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.passkey_display = auth_passkey_display,
	.passkey_confirm = auth_passkey_confirm,
	.cancel = auth_cancel,
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed};
#else
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
						  uint16_t len)
{
	int err;
	char addr[BT_ADDR_LE_STR_LEN] = {0};

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr));

	LOG_INF("Received data from: %s", addr);

	for (uint16_t pos = 0; pos != len;)
	{
		struct uart_data_t *tx = k_malloc(sizeof(*tx));

		if (!tx)
		{
			LOG_WRN("Not able to allocate UART send data buffer");
			return;
		}

		/* Keep the last byte of TX buffer for potential LF char. */
		size_t tx_data_size = sizeof(tx->data) - 1;

		if ((len - pos) > tx_data_size)
		{
			tx->len = tx_data_size;
		}
		else
		{
			tx->len = (len - pos);
		}

		memcpy(tx->data, &data[pos], tx->len);

		pos += tx->len;

		/* Append the LF character when the CR character triggered
		 * transmission from the peer.
		 */
		if ((pos == len) && (data[len - 1] == '\r'))
		{
			tx->data[tx->len] = '\n';
			tx->len++;
		}

		err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
		if (err)
		{
			k_fifo_put(&fifo_uart_tx_data, tx);
		}
	}
}

static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

void error(void)
{
	dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK);

	while (true)
	{
		/* Spin for ever */
		k_sleep(K_MSEC(1000));
	}
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
	if (accept)
	{
		bt_conn_auth_passkey_confirm(auth_conn);
		LOG_INF("Numeric Match, conn %p", (void *)auth_conn);
	}
	else
	{
		bt_conn_auth_cancel(auth_conn);
		LOG_INF("Numeric Reject, conn %p", (void *)auth_conn);
	}

	bt_conn_unref(auth_conn);
	auth_conn = NULL;
}

void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;

	if (auth_conn)
	{
		if (buttons & KEY_PASSKEY_ACCEPT)
		{
			num_comp_reply(true);
		}

		if (buttons & KEY_PASSKEY_REJECT)
		{
			num_comp_reply(false);
		}
	}
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err)
	{
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err)
	{
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	err = uart_init();
	if (err)
	{
		error();
	}

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED))
	{
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err)
		{
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err)
		{
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err)
	{
		error();
	}

	LOG_INF("Bluetooth initialized");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_nus_init(&nus_cb);
	if (err)
	{
		LOG_ERR("Failed to initialize UART service (err: %d)", err);
		return 0;
	}

	err = create_advertising_coded();
	if (err)
	{
		LOG_ERR("Advertising coded failed to start (err %d)", err);
		return 0;
	}
	err = create_advertising_standard();
	if (err)
	{
		LOG_ERR("Advertising 1M/2M failed to start (err %d)", err);
		return 0;
	}

	k_sem_give(&ble_init_ok);

	k_work_submit(&start_advertising_worker);

	// Safety tripped
	dk_set_led(SAFETY_LINE, 1);

	for (;;)
	{
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);

	for (;;)
	{
		/* Wait indefinitely for data to be sent over bluetooth */
		struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
											 K_FOREVER);

		if (bt_nus_send(NULL, buf->data, buf->len))
		{
			LOG_WRN("Failed to send data over BLE connection");
		}

		k_free(buf);
	}
}

#define FEM_NRF_NODE DT_NODELABEL(nrf_radio_fem)
static const struct gpio_dt_spec antenna_sel = GPIO_DT_SPEC_GET(FEM_NRF_NODE, ant_sel_gpios);
const uint16_t ANTENNA_TOGGLING_INTERVAL = (CONFIG_BT_PERIPHERAL_PREF_MAX_INT * 2.5);
const uint16_t ANTENNA_COMMUNICATION_INTERVAL = (CONFIG_BT_PERIPHERAL_PREF_TIMEOUT * 10 / 2);

void ble_rssi_thread(void)
{
	uint16_t conn_handle = 0;

	while (1)
	{
		k_sem_take(&rssi_sem, K_FOREVER);
		k_sleep(K_MSEC(2000));
		bt_hci_get_conn_handle(current_conn, &conn_handle);
		sdc_hci_cmd_sp_read_rssi_t p_param = {conn_handle};

		for (;;)
		{

			// gpio_pin_set_dt(&antenna_sel, 0);
			// k_sleep(K_MSEC(ANTENNA_TOGGLING_INTERVAL));
			// sdc_hci_cmd_sp_read_rssi_return_t p_return1 = {conn_handle, 0};
			// uint8_t err = sdc_hci_cmd_sp_read_rssi(&p_param, &p_return1);
			// gpio_pin_set_dt(&antenna_sel, 1);
			// k_sleep(K_MSEC(ANTENNA_TOGGLING_INTERVAL));
			// sdc_hci_cmd_sp_read_rssi_return_t p_return2 = {conn_handle, 0};
			// err = sdc_hci_cmd_sp_read_rssi(&p_param, &p_return2);
			// if (err)
			// {
			// 	LOG_WRN("Error %d Reading RSSI", err);
			// 	break;
			// }

			// if (p_return1.rssi > p_return2.rssi)
			// {
			// 	gpio_pin_set_dt(&antenna_sel, 0);
			// 	if (antenna)
			// 	{
			// 		antenna = 0;
			// 		LOG_INF("Using antenna 1, RSSI %d", p_return1.rssi);
			// 	}
			// }
			// else if (!antenna)
			// {
			// 	antenna = 1;
			// 	LOG_INF("Using antenna 2, RSSI %d", p_return2.rssi);
			// }
			// k_sleep(K_MSEC(ANTENNA_COMMUNICATION_INTERVAL));

			sdc_hci_cmd_sp_read_rssi_return_t p_return = {conn_handle, 0};
			uint8_t err = sdc_hci_cmd_sp_read_rssi(&p_param, &p_return);
			if (err)
			{
				LOG_WRN("Error %d Reading RSSI", err);
				break;
			}
			else
			{
				LOG_INF("RSSI %d", p_return.rssi);
			}

			k_sleep(K_MSEC(1000));

		}
	}
}
K_THREAD_DEFINE(ble_rssi_thread_id, STACKSIZE, ble_rssi_thread, NULL, NULL,
				NULL, PRIORITY_RSSI, 0, 0);

K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
				NULL, PRIORITY, 0, 0);
