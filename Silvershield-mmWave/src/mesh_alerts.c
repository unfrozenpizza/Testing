// Bluetooth Mesh transport for the sensor's alerts.
//
// Every alert (the same 3-byte radar_payload that goes out over the GATT link) is also
// published to the mesh group ALERT_GROUP_ADDR. Other nodes running this firmware relay it
// on, so an alert can hop across several boards to reach a receiver that is out of direct
// range. A node that receives an alert from another node prints it, so a second board
// flashed with this firmware works as a test receiver.
//
// Provisioning: the node provisions itself on first boot with the fixed keys below and
// saves them to flash. This is for prototyping only; anyone with these keys can read and
// send alerts. Before real use, provision with a proper provisioner instead.

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/mesh.h>

#include "mesh_alerts.h"

// Company ID 0x05F1 (Linux Foundation), as used by the Zephyr mesh samples.
#define ALERT_COMPANY_ID  0x05F1
#define ALERT_MODEL_ID    0x0001
#define OP_ALERT          BT_MESH_MODEL_OP_3(0x01, ALERT_COMPANY_ID)
#define ALERT_PAYLOAD_LEN 3     // must match sizeof(struct radar_payload) in main.cpp

#define ALERT_GROUP_ADDR  0xC000

#define NET_IDX           0x000
#define APP_IDX           0x000
#define IV_INDEX          0

// Shared by every node in the network. Prototype keys: change both before real use.
static const uint8_t net_key[16] = {
    0x53, 0x69, 0x6c, 0x76, 0x65, 0x72, 0x73, 0x68,
    0x69, 0x65, 0x6c, 0x64, 0x4e, 0x65, 0x74, 0x31,
};
static const uint8_t app_key[16] = {
    0x53, 0x69, 0x6c, 0x76, 0x65, 0x72, 0x73, 0x68,
    0x69, 0x65, 0x6c, 0x64, 0x41, 0x70, 0x70, 0x31,
};

static uint8_t dev_uuid[16];
static uint16_t node_addr;
static atomic_t mesh_ready;

static struct bt_mesh_cfg_cli cfg_cli;

static const struct bt_mesh_model root_models[] = {
    BT_MESH_MODEL_CFG_SRV,
    BT_MESH_MODEL_CFG_CLI(&cfg_cli),
};

static int alert_received(const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
                          struct net_buf_simple *buf)
{
    // Our own publications loop back because we're subscribed to the same group.
    if (ctx->addr == node_addr) {
        return 0;
    }

    uint8_t event_type = net_buf_simple_pull_u8(buf);
    uint8_t target_id = net_buf_simple_pull_u8(buf);
    uint8_t extra = net_buf_simple_pull_u8(buf); // reserved; heartbeat count for 0x07

    if (event_type == 0x07) { // STATUS_HEARTBEAT in main.cpp
        printk("Mesh Received <- Heartbeat #%u from node 0x%04X (RSSI %d)\r\n",
               extra, ctx->addr, ctx->recv_rssi);
        return 0;
    }

    printk("Mesh Received <- Event: 0x%02X | Target ID: %d | from node 0x%04X (RSSI %d)\r\n",
           event_type, target_id, ctx->addr, ctx->recv_rssi);
    return 0;
}

static const struct bt_mesh_model_op alert_ops[] = {
    { OP_ALERT, BT_MESH_LEN_EXACT(ALERT_PAYLOAD_LEN), alert_received },
    BT_MESH_MODEL_OP_END,
};

static const struct bt_mesh_model vnd_models[] = {
    BT_MESH_MODEL_VND(ALERT_COMPANY_ID, ALERT_MODEL_ID, alert_ops, NULL, NULL),
};

static const struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(0, root_models, vnd_models),
};

static const struct bt_mesh_comp comp = {
    .cid = ALERT_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static const struct bt_mesh_prov prov = {
    .uuid = dev_uuid,
};

// Each board gets its mesh address and UUID from its chip's unique hardware ID, so the
// same firmware can be flashed on every board without editing anything.
static void derive_identity(void)
{
    uint8_t id[16] = { 0 };
    ssize_t len = hwinfo_get_device_id(id, sizeof(id));

    if (len <= 0) {
        printk("Mesh: no hardware ID (err %d), using a random address\r\n", (int)len);
        bt_rand(id, sizeof(id));
        len = sizeof(id);
    }

    memcpy(dev_uuid, id, MIN((size_t)len, sizeof(dev_uuid)));

    uint16_t folded = 0;
    for (ssize_t i = 0; i < len; i += 2) {
        folded ^= (uint16_t)(id[i] | ((i + 1 < len) ? (id[i + 1] << 8) : 0));
    }
    node_addr = folded & 0x7FFF; // unicast range is 0x0001..0x7FFF
    if (node_addr == 0) {
        node_addr = 1;
    }
}

int mesh_alerts_init(void)
{
    derive_identity();

    int err = bt_mesh_init(&prov, &comp);
    if (err) {
        printk("Mesh init failed (err %d)\r\n", err);
    }
    return err;
}

// Adds the app key, binds it to the alert model and subscribes the model to the alert
// group. Only needed once: the result is saved to flash with the rest of the mesh state.
static int configure(void)
{
    uint8_t status = 0;
    int err;

    err = bt_mesh_cfg_cli_app_key_add(NET_IDX, node_addr, NET_IDX, APP_IDX, app_key, &status);
    if (err || status) {
        printk("Mesh: app key add failed (err %d, status %u)\r\n", err, status);
        return err ? err : -EIO;
    }

    err = bt_mesh_cfg_cli_mod_app_bind_vnd(NET_IDX, node_addr, node_addr, APP_IDX,
                                           ALERT_MODEL_ID, ALERT_COMPANY_ID, &status);
    if (err || status) {
        printk("Mesh: model bind failed (err %d, status %u)\r\n", err, status);
        return err ? err : -EIO;
    }

    err = bt_mesh_cfg_cli_mod_sub_add_vnd(NET_IDX, node_addr, node_addr, ALERT_GROUP_ADDR,
                                          ALERT_MODEL_ID, ALERT_COMPANY_ID, &status);
    if (err || status) {
        printk("Mesh: group subscribe failed (err %d, status %u)\r\n", err, status);
        return err ? err : -EIO;
    }

    return 0;
}

int mesh_alerts_start(void)
{
    uint8_t dev_key[16];
    int err;

    bt_rand(dev_key, sizeof(dev_key));

    err = bt_mesh_provision(net_key, NET_IDX, 0, IV_INDEX, node_addr, dev_key);
    if (err == -EALREADY) {
        printk("Mesh: using saved network settings\r\n");
    } else if (err) {
        printk("Mesh: provisioning failed (err %d)\r\n", err);
        return err;
    } else {
        printk("Mesh: joined network, configuring\r\n");
        err = configure();
        if (err) {
            // Wipe the half-done setup so the next boot provisions and configures again.
            bt_mesh_reset();
            return err;
        }
    }

    atomic_set(&mesh_ready, 1);
    printk("Mesh active. Node address 0x%04X, publishing alerts to group 0x%04X\r\n",
           node_addr, ALERT_GROUP_ADDR);
    return 0;
}

bool mesh_alerts_ready(void)
{
    return atomic_get(&mesh_ready) != 0;
}

int mesh_alerts_send(const uint8_t *payload, size_t len)
{
    if (!mesh_alerts_ready()) {
        return -EAGAIN;
    }
    if (len != ALERT_PAYLOAD_LEN) {
        return -EINVAL;
    }

    BT_MESH_MODEL_BUF_DEFINE(msg, OP_ALERT, ALERT_PAYLOAD_LEN);
    struct bt_mesh_msg_ctx ctx = {
        .net_idx = NET_IDX,
        .app_idx = APP_IDX,
        .addr = ALERT_GROUP_ADDR,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    bt_mesh_model_msg_init(&msg, OP_ALERT);
    net_buf_simple_add_mem(&msg, payload, len);

    return bt_mesh_model_send(&vnd_models[0], &ctx, &msg, NULL, NULL);
}
