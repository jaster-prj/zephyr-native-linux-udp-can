# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_CAN_NATIVE_UDP ${CMAKE_CURRENT_LIST_DIR}/can_native_udp.c)
