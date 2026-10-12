/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_native_linux_udp_can

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmdline.h>
#include <posix_native_task.h>

#include <zephyr/drivers/can.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(can_native_udp, CONFIG_CAN_LOG_LEVEL);

struct can_filter_context {
	can_rx_callback_t rx_cb;
	void *cb_arg;
	struct can_filter filter;
};

struct can_udp_frame {
	uint32_t id;
	uint8_t dlc;
	uint8_t flags;
	uint8_t reserved[2];
	uint8_t data[CANFD_MAX_DLC];
};

struct can_native_udp_data {
	struct can_driver_data common;
	struct can_filter_context filters[CONFIG_CAN_NATIVE_UDP_MAX_FILTERS];
	struct k_mutex filter_mutex;
	struct k_sem tx_idle;
	can_tx_callback_t tx_callback;
	void *tx_user_data;
	int udp_fd;
	struct sockaddr_in peer_addr;
	struct k_thread rx_thread;

	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ARCH_POSIX_RECOMMENDED_STACK_SIZE);
};

struct can_native_udp_config {
	const struct can_driver_config common;
	const char *host;
	uint16_t port;
	uint16_t local_port;
};

static void dispatch_frame(const struct device *dev, struct can_frame *frame)
{
	struct can_native_udp_data *data = dev->data;
	can_rx_callback_t callback;
	struct can_frame tmp_frame;

	k_mutex_lock(&data->filter_mutex, K_FOREVER);

	for (int filter_id = 0; filter_id < ARRAY_SIZE(data->filters); filter_id++) {
		if (data->filters[filter_id].rx_cb == NULL) {
			continue;
		}

		if (!can_frame_matches_filter(frame, &data->filters[filter_id].filter)) {
			continue;
		}

		tmp_frame = *frame;
		callback = data->filters[filter_id].rx_cb;
		callback(dev, &tmp_frame, data->filters[filter_id].cb_arg);
	}

	k_mutex_unlock(&data->filter_mutex);
}

static void can_native_udp_rx_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct can_native_udp_data *data = dev->data;
	struct can_udp_frame wire;
	struct can_frame frame;
	ssize_t len;

	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	LOG_DBG("Starting UDP CAN RX thread");

	while (true) {
		len = recv(data->udp_fd, &wire, sizeof(wire), 0);
		if (len < (ssize_t)offsetof(struct can_udp_frame, data)) {
			continue;
		}

		if (!data->common.started) {
			continue;
		}

		memset(&frame, 0, sizeof(frame));
		frame.id = wire.id;
		frame.flags = wire.flags;
		frame.dlc = wire.dlc;
		memcpy(frame.data, wire.data, MIN(sizeof(frame.data), wire.dlc));

		LOG_DBG("Received CAN frame id=0x%lx dlc=%u flags=0x%02x",
			(unsigned long)frame.id, frame.dlc, frame.flags);

		dispatch_frame(dev, &frame);
	}
}

static int can_native_udp_send(const struct device *dev, const struct can_frame *frame,
				k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	struct can_native_udp_data *data = dev->data;
	struct can_udp_frame wire;
	ssize_t ret;

	LOG_DBG("Sending %d bytes on %s. Id: 0x%x, ID type: %s %s",
		frame->dlc, dev->name, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? ", RTR frame" : "");

	if (frame->dlc > CANFD_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CANFD_MAX_DLC);
		return -EINVAL;
	}

	if (data->udp_fd <= 0) {
		LOG_ERR("No socket: %d", data->udp_fd);
		return -EIO;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if (k_sem_take(&data->tx_idle, timeout) != 0) {
		return -EAGAIN;
	}

	data->tx_callback = callback;
	data->tx_user_data = user_data;

	memset(&wire, 0, sizeof(wire));
	wire.id = frame->id;
	wire.flags = frame->flags;
	wire.dlc = frame->dlc;
	memcpy(wire.data, frame->data, frame->dlc);

	ret = sendto(data->udp_fd, &wire, sizeof(wire), 0,
			(struct sockaddr *)&data->peer_addr, sizeof(data->peer_addr));
	if (ret < 0) {
		LOG_ERR("Cannot send CAN data via UDP (%d)", errno);
		return -EIO;
	}

	if (data->common.mode & CAN_MODE_LOOPBACK) {
		dispatch_frame(dev, (struct can_frame *)frame);
	}

	if (callback != NULL) {
		callback(dev, 0, user_data);
	}

	k_sem_give(&data->tx_idle);

	return 0;
}

static int can_native_udp_add_rx_filter(const struct device *dev, can_rx_callback_t cb,
						 void *cb_arg, const struct can_filter *filter)
{
	struct can_native_udp_data *data = dev->data;
	struct can_filter_context *filter_ctx;
	int filter_id = -ENOSPC;

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->filter_mutex, K_FOREVER);

	for (int i = 0; i < ARRAY_SIZE(data->filters); i++) {
		if (data->filters[i].rx_cb == NULL) {
			filter_id = i;
			break;
		}
	}

	if (filter_id < 0) {
		LOG_ERR("No free filter left");
		k_mutex_unlock(&data->filter_mutex);
		return filter_id;
	}

	filter_ctx = &data->filters[filter_id];
	filter_ctx->rx_cb = cb;
	filter_ctx->cb_arg = cb_arg;
	filter_ctx->filter = *filter;

	k_mutex_unlock(&data->filter_mutex);
	return filter_id;
}

static void can_native_udp_remove_rx_filter(const struct device *dev, int filter_id)
{
	struct can_native_udp_data *data = dev->data;

	if (filter_id < 0 || filter_id >= ARRAY_SIZE(data->filters)) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	k_mutex_lock(&data->filter_mutex, K_FOREVER);
	data->filters[filter_id].rx_cb = NULL;
	k_mutex_unlock(&data->filter_mutex);
}

static int can_native_udp_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK;

#ifdef CONFIG_CAN_FD_MODE
	*cap |= CAN_MODE_FD;
#endif

	return 0;
}

static int can_native_udp_start(const struct device *dev)
{
	struct can_native_udp_data *data = dev->data;

	if (data->common.started) {
		return -EALREADY;
	}

	data->common.started = true;
	return 0;
}

static int can_native_udp_stop(const struct device *dev)
{
	struct can_native_udp_data *data = dev->data;

	if (!data->common.started) {
		return -EALREADY;
	}

	data->common.started = false;
	return 0;
}

static int can_native_udp_set_mode(const struct device *dev, can_mode_t mode)
{
	struct can_native_udp_data *data = dev->data;

#ifdef CONFIG_CAN_FD_MODE
	if ((mode & ~(CAN_MODE_LOOPBACK | CAN_MODE_FD)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}
#else
	if ((mode & ~(CAN_MODE_LOOPBACK)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}
#endif

	if (data->common.started) {
		return -EBUSY;
	}

	data->common.mode = mode;
	return 0;
}

static int can_native_udp_set_timing(const struct device *dev, const struct can_timing *timing)
{
	struct can_native_udp_data *data = dev->data;

	ARG_UNUSED(timing);
	ARG_UNUSED(dev);

	if (data->common.started) {
		return -EBUSY;
	}

	return 0;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_native_udp_set_timing_data(const struct device *dev, const struct can_timing *timing)
{
	struct can_native_udp_data *data = dev->data;

	ARG_UNUSED(timing);
	ARG_UNUSED(dev);

	if (data->common.started) {
		return -EBUSY;
	}

	return 0;
}
#endif

static int can_native_udp_get_state(const struct device *dev, enum can_state *state,
						 struct can_bus_err_cnt *err_cnt)
{
	struct can_native_udp_data *data = dev->data;

	if (state != NULL) {
		*state = data->common.started ? CAN_STATE_ERROR_ACTIVE : CAN_STATE_STOPPED;
	}

	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = 0;
		err_cnt->rx_err_cnt = 0;
	}

	return 0;
}

static int can_native_udp_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);
	*rate = MHZ(80);
	return 0;
}

static int can_native_udp_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ide);
	return CONFIG_CAN_NATIVE_UDP_MAX_FILTERS;
}

static DEVICE_API(can, can_native_udp_driver_api) = {
	.start = can_native_udp_start,
	.stop = can_native_udp_stop,
	.get_capabilities = can_native_udp_get_capabilities,
	.set_mode = can_native_udp_set_mode,
	.set_timing = can_native_udp_set_timing,
	.send = can_native_udp_send,
	.add_rx_filter = can_native_udp_add_rx_filter,
	.remove_rx_filter = can_native_udp_remove_rx_filter,
	.get_state = can_native_udp_get_state,
	.get_core_clock = can_native_udp_get_core_clock,
	.get_max_filters = can_native_udp_get_max_filters,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 0,
		.phase_seg1 = 2,
		.phase_seg2 = 2,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 128,
		.prop_seg = 0,
		.phase_seg1 = 256,
		.phase_seg2 = 128,
		.prescaler = 32,
	},
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_native_udp_set_timing_data,
	.timing_data_min = {
		.sjw = 1,
		.prop_seg = 0,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1,
	},
	.timing_data_max = {
		.sjw = 16,
		.prop_seg = 0,
		.phase_seg1 = 32,
		.phase_seg2 = 16,
		.prescaler = 32,
	},
#endif
};

static int can_native_udp_init(const struct device *dev)
{
	const struct can_native_udp_config *cfg = dev->config;
	struct can_native_udp_data *data = dev->data;
	struct sockaddr_in bind_addr = {0};
	int ret;

	sys_slist_init(&data->common.state_change_callbacks);
	k_mutex_init(&data->filter_mutex);
	k_sem_init(&data->tx_idle, 1, 1);

	data->udp_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (data->udp_fd < 0) {
		LOG_ERR("Cannot create UDP socket (%d)", errno);
		return -ENODEV;
	}

	memset(&data->peer_addr, 0, sizeof(data->peer_addr));
	data->peer_addr.sin_family = AF_INET;
	data->peer_addr.sin_port = htons(cfg->port);

	ret = inet_pton(AF_INET, cfg->host, &data->peer_addr.sin_addr);
	if (ret != 1) {
		LOG_ERR("Invalid gateway host %s", cfg->host);
		close(data->udp_fd);
		return -EINVAL;
	}

	bind_addr.sin_family = AF_INET;
	bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	bind_addr.sin_port = htons(cfg->local_port);

	if (cfg->local_port != 0U) {
		ret = bind(data->udp_fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
		if (ret < 0) {
			LOG_ERR("Cannot bind local UDP port %u (%d)", cfg->local_port, errno);
			close(data->udp_fd);
			return -EADDRINUSE;
		}
	}

	k_thread_create(&data->rx_thread, data->rx_thread_stack,
			K_KERNEL_STACK_SIZEOF(data->rx_thread_stack),
			can_native_udp_rx_thread, (void *)dev, NULL, NULL,
			CONFIG_CAN_NATIVE_UDP_RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	LOG_DBG("Init of %s done", dev->name);
	return 0;
}

#ifdef CONFIG_CAN_FD_MODE
#define CAN_NATIVE_UDP_MAX_BITRATE 8000000
#else
#define CAN_NATIVE_UDP_MAX_BITRATE 1000000
#endif

#define CAN_NATIVE_UDP_INIT(inst) \
	static const struct can_native_udp_config can_native_udp_cfg_##inst = { \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, CAN_NATIVE_UDP_MAX_BITRATE), \
		.host = DT_INST_PROP(inst, host), \
		.port = DT_INST_PROP(inst, port), \
		.local_port = DT_INST_PROP_OR(inst, local_port, 0), \
	}; \
	static struct can_native_udp_data can_native_udp_data_##inst; \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_native_udp_init, NULL, \
			&can_native_udp_data_##inst, \
			&can_native_udp_cfg_##inst, \
			POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
			&can_native_udp_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CAN_NATIVE_UDP_INIT)
