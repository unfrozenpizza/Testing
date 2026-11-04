#ifndef MESH_ALERTS_H
#define MESH_ALERTS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Bluetooth Mesh side of the alerts. Kept in a C file because Zephyr's mesh model
// macros only compile as C, not C++.

#ifdef __cplusplus
extern "C" {
#endif

// Call after bt_enable() and before settings_load().
int mesh_alerts_init(void);

// Call after settings_load(). Joins the network on first boot (self-provisioning with
// the keys in mesh_alerts.c); later boots reuse what was saved to flash.
int mesh_alerts_start(void);

// True once the node is in the network and can send.
bool mesh_alerts_ready(void);

// Publishes one alert payload to the alert group. Returns 0 if it was queued for sending,
// a negative error if not (e.g. -ENOBUFS when mesh buffers are busy; retry later).
int mesh_alerts_send(const uint8_t *payload, size_t len);

#ifdef __cplusplus
}
#endif

#endif // MESH_ALERTS_H
