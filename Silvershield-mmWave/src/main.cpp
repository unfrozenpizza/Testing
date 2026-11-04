#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/settings/settings.h>
#include <zephyr/audio/dmic.h> // Required for digital microphone
#include <nrfx_pdm.h> // Direct HAL access for runtime PDM gain control
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/atomic.h>
#include "mesh_alerts.h"
#include "model-parameters/model_metadata.h"
#ifdef ROUND_UP
#undef ROUND_UP
#endif
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" //change file name IF different and if i want to change AI model

// =========================================================================
// PROJECT DEFINES
// =========================================================================
#define UART_NODE DT_NODELABEL(uart1)
#define MSG_BUFFER_SIZE 128
#define RADAR_MAX_TARGET_ID 254 // radar sends 0..255; 0xFF is reserved for audio events
#define RADAR_ECHO_LINES    0   // 1 = print every raw radar line ("RAW UART: ...")

// Radar UART receive: the interrupt copies bytes in as they arrive, main() reads them out.
// Polling with a 1ms sleep dropped bytes at 921600 baud.
#define RADAR_RX_BUF_SIZE 256
RING_BUF_DECLARE(radar_rx_rb, RADAR_RX_BUF_SIZE);
K_SEM_DEFINE(radar_rx_sem, 0, 1);
static atomic_t radar_rx_dropped; // bytes lost to a full ring buffer; reported from main()

static void radar_uart_isr(const struct device *dev, void *user_data) {
    uint8_t buf[32];

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len <= 0) {
            break;
        }
        uint32_t put = ring_buf_put(&radar_rx_rb, buf, len);
        if (put < (uint32_t)len) {
            // No printk here: blocking on the console inside the ISR only loses more bytes.
            atomic_add(&radar_rx_dropped, (atomic_val_t)(len - put));
        }
    }
    k_sem_give(&radar_rx_sem);
}

// =========================================================================
// NEW: ACOUSTIC SENSOR DEFINES
// =========================================================================
#define DMIC_NODE            DT_NODELABEL(pdm0)
#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_BLOCK_SIZE     1024  // bytes = 512 samples @ 16-bit PCM = 32ms per block
#define AUDIO_BLOCK_SAMPLES  (AUDIO_BLOCK_SIZE / sizeof(int16_t))
#define AUDIO_BLOCK_COUNT    16 // Driver queue is 8 deep (app.overlay), so 16 leaves headroom for the reader
#define AUDIO_DISCARD_BLOCKS 30 // ~1s of audio ignored after every mic start/restart (startup pop + DC settling)
#define DEBUG_PRINT_BLOCKS   94  // audio DEBUG line every 94 blocks (~3s); impacts/rejections always print

// PDM gain register: 0x00-0x50, 0.5dB/step, 0x28 = 0dB (Zephyr's dmic API doesn't
// expose this, so it's set directly via the nrfx HAL after every stream start).
// Was 0x40 (+12dB): ordinary speech near the mic then clipped at full scale and
// tripped the impact detector. All impact thresholds below are for 0dB.
#define PDM_GAIN             0x28

// Impact (drop/fall) detector, evaluated per 32ms block after removing the block's DC offset.
// Units are raw signed 16-bit PCM amplitude (full scale 32767).
#define IMPACT_PEAK_THRESHOLD   5000  // Equivalent of the old 20000 at +12dB gain
#define IMPACT_MIN_SAMPLES      80    // Samples above IMPACT_PEAK_THRESHOLD in the onset block (~5ms of loud audio)
#define IMPACT_ONSET_RATIO      16.0f // Onset block energy must be >= 16x (+12dB) the background noise floor
#define IMPACT_DECAY_RATIO      8.0f  // ...and must fall back below onset/8 (-9dB) within IMPACT_DECAY_BLOCKS,
#define IMPACT_DECAY_BLOCKS     8     // i.e. ~256ms. Sustained loud sound (speech, TV, music) is not an impact.
#define IMPACT_COOLDOWN_MS      2000
#define NOISE_FLOOR_MIN_RMS     50.0f // Floor clamp so a dead-silent room doesn't make every tap an "onset"

// Blocks that are not plausible audio are rejected outright (PDM glitch, DC step, stuck bitstream).
#define CLIP_LEVEL              32000 // |raw sample| at or above this counts as clipped
#define CLIP_REJECT_SAMPLES     ((int)AUDIO_BLOCK_SAMPLES / 2) // more than half the block pinned at full scale
#define MIN_ZERO_CROSSINGS      4     // a real loud sound crosses zero many times per 32ms; a DC/rail block doesn't (loud blocks only)

// Allocate memory for the audio stream
K_MEM_SLAB_DEFINE(rx_mem_slab, AUDIO_BLOCK_SIZE, AUDIO_BLOCK_COUNT, 16);

// =========================================================================
// PHASE 2: DATA STRUCTURES & PAYLOADS
// =========================================================================
typedef enum {
    RADAR_EVENT_ENTRY    = 0x01,
    RADAR_EVENT_EXIT     = 0x02,
    RADAR_EVENT_FALL     = 0x03,
    RADAR_EVENT_RECOVERY = 0x04,
    AUDIO_EVENT_DROP     = 0x05, // New: Heavy object drop
    AUDIO_EVENT_HELP     = 0x06, // New: "Help" keyword spoken
    STATUS_HEARTBEAT     = 0x07  // "I'm alive", every HEARTBEAT_INTERVAL_MS (not an alert)
} radar_event_t;

struct __attribute__((__packed__)) radar_payload {
    uint8_t event_type;
    uint8_t target_id;
    uint8_t reserved;
};
// The mesh side (mesh_alerts.c) sends these same 3 bytes.
static_assert(sizeof(struct radar_payload) == 3, "mesh_alerts.c expects a 3-byte payload");

// Only the BLE sender thread touches this (it's also the characteristic's backing value).
static struct radar_payload current_data;


// =========================================================================
// PHASE 3: CUSTOM BLE GATT SERVICE DEFINITION
// =========================================================================
#define BT_UUID_RADAR_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345678)

#define BT_UUID_RADAR_CHRC_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x567812345679)

static struct bt_uuid_128 radar_service_uuid = BT_UUID_INIT_128(BT_UUID_RADAR_SERVICE_VAL);
static struct bt_uuid_128 radar_chrc_uuid = BT_UUID_INIT_128(BT_UUID_RADAR_CHRC_VAL);

K_SEM_DEFINE(ble_flush_sem, 0, 1); // wakes the BLE sender to re-send held alerts

static void radar_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("BLE Notifications: %s\r\n", notif_enabled ? "ENABLED" : "DISABLED");
    if (notif_enabled) {
        k_sem_give(&ble_flush_sem);
    }
}

BT_GATT_SERVICE_DEFINE(radar_svc,
    BT_GATT_PRIMARY_SERVICE(&radar_service_uuid),
    BT_GATT_CHARACTERISTIC(&radar_chrc_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, &current_data),
    BT_GATT_CCC(radar_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

// =========================================================================
// PHASE 4: BLE INITIALIZATION & ADVERTISING
// =========================================================================
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    // Ensure CONFIG_BT_DEVICE_NAME is set in prj.conf
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_RADAR_SERVICE_VAL),
};

// The receiver's connection (NULL when nobody is connected). Set/cleared from the BT callbacks,
// read by the BLE sender thread.
static struct bt_conn *ble_conn;
K_MUTEX_DEFINE(ble_conn_lock);

static void start_advertising(void) {
    int err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err == 0) {
        printk("Bluetooth advertising started\r\n");
    } else if (err != -EALREADY) {
        printk("Advertising failed to start (err %d)\r\n", err);
    }
}

static void adv_work_handler(struct k_work *work) {
    start_advertising();
}
K_WORK_DEFINE(adv_work, adv_work_handler);

static void ble_connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("BLE connection failed (err 0x%02X)\r\n", err);
        return;
    }
    printk("BLE connected\r\n");
    k_mutex_lock(&ble_conn_lock, K_FOREVER);
    if (!ble_conn) {
        ble_conn = bt_conn_ref(conn);
    }
    k_mutex_unlock(&ble_conn_lock);
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("BLE disconnected (reason 0x%02X)\r\n", reason);
    k_mutex_lock(&ble_conn_lock, K_FOREVER);
    if (ble_conn == conn) {
        bt_conn_unref(ble_conn);
        ble_conn = NULL;
    }
    k_mutex_unlock(&ble_conn_lock);
}

// Connectable advertising stops when a connection is made and does not restart on its own.
// Once the connection object is freed after a disconnect, advertise again so the receiver
// can reconnect (otherwise the device is invisible until reboot).
static void ble_recycled(void) {
    k_work_submit(&adv_work);
}

BT_CONN_CB_DEFINE(ble_conn_callbacks) = {
    .connected = ble_connected,
    .disconnected = ble_disconnected,
    .recycled = ble_recycled,
};

// Retries until the Bluetooth stack is up and advertising; never gives up, so a transient
// failure (e.g. network core slow to boot) doesn't leave the device permanently offline.
static void init_bluetooth(void) {
    int delay_ms = 1000;

    while (1) {
        int err = bt_enable(NULL);
        if (err == 0 || err == -EALREADY) {
            break;
        }
        printk("Bluetooth init failed (err %d), retrying in %d ms\r\n", err, delay_ms);
        k_msleep(delay_ms);
        delay_ms = MIN(delay_ms * 2, 30000);
    }

    // Mesh has to be set up before the saved settings are loaded, so it can restore its
    // network keys. If mesh fails, the direct GATT link below still works.
    int mesh_err = mesh_alerts_init();
    settings_load();
    if (mesh_err == 0) {
        mesh_alerts_start();
    }

    start_advertising();
    printk("Bluetooth active. Advertising as: '%s'\r\n", CONFIG_BT_DEVICE_NAME);
}


// =========================================================================
// PHASE 5: BLE NOTIFICATION TRANSMISSION
// =========================================================================
// The radar (main), audio and AI threads all raise events. They only drop the event
// into this queue (never blocks); one sender thread owns current_data and calls
// bt_gatt_notify, so the audio thread can't stall waiting on a BLE buffer and two
// events can't overwrite each other's payload.
K_MSGQ_DEFINE(ble_event_q, sizeof(struct radar_payload), 16, 1);

// Alerts that couldn't be delivered yet are held in a ring per transport and re-sent later.
// GATT holds them until a receiver connects and subscribes; mesh holds them only while its
// send buffers are busy (or mesh isn't up yet). When a ring is full, its oldest alert is dropped.
#define BLE_PENDING_MAX   16
#define BLE_RETRY_MS      1000

// Heartbeat: 07 FF <count>, sent every HEARTBEAT_INTERVAL_MS so the receiver can tell a
// working sensor from a dead/unplugged one (no heartbeat for ~2 intervals = sensor down).
// <count> goes up by 1 each time (wraps 255 -> 0), so a gap shows missed heartbeats.
// Heartbeats are never held: if nobody is subscribed, or mesh buffers are busy, that one is
// skipped. Holding them would fill the pending rings and push out real alerts.
#define HEARTBEAT_INTERVAL_MS 60000

struct alert_ring {
    const char *name;
    struct radar_payload items[BLE_PENDING_MAX];
    size_t head, count;
};
static struct alert_ring gatt_pending = { .name = "BLE" };
static struct alert_ring mesh_pending = { .name = "Mesh" };

void send_radar_ble_notification(radar_event_t event, uint8_t target_id) {
    struct radar_payload evt = {
        .event_type = (uint8_t)event,
        .target_id = target_id,
        .reserved = 0x00,
    };

    if (k_msgq_put(&ble_event_q, &evt, K_NO_WAIT) != 0) {
        printk("BLE event queue full, event 0x%02X dropped\r\n", event);
    }
}

static void ring_push(struct alert_ring *r, const struct radar_payload *evt) {
    if (r->count == BLE_PENDING_MAX) {
        printk("%s: pending alerts full, oldest (0x%02X) dropped\r\n",
               r->name, r->items[r->head].event_type);
        r->head = (r->head + 1) % BLE_PENDING_MAX;
        r->count--;
    }
    r->items[(r->head + r->count) % BLE_PENDING_MAX] = *evt;
    r->count++;
}

static void ring_pop(struct alert_ring *r) {
    r->head = (r->head + 1) % BLE_PENDING_MAX;
    r->count--;
}

static void log_sent(const char *transport, const struct radar_payload *evt) {
    if (evt->event_type == STATUS_HEARTBEAT) {
        printk("%s Sent -> Heartbeat #%u\r\n", transport, evt->reserved);
    } else if (evt->event_type >= AUDIO_EVENT_DROP) {
        printk("%s Sent -> Acoustic Event: 0x%02X\r\n", transport, evt->event_type);
    } else {
        printk("%s Sent -> Radar Event: 0x%02X | Target ID: %d\r\n", transport, evt->event_type, evt->target_id);
    }
}

// Sends held alerts in order while a subscribed receiver is connected. Stops at the first
// failure and leaves the rest held for the next attempt.
static void gatt_flush_pending(void) {
    while (gatt_pending.count > 0) {
        k_mutex_lock(&ble_conn_lock, K_FOREVER);
        struct bt_conn *conn = ble_conn ? bt_conn_ref(ble_conn) : NULL;
        k_mutex_unlock(&ble_conn_lock);

        if (!conn) {
            return;
        }
        if (!bt_gatt_is_subscribed(conn, &radar_svc.attrs[2], BT_GATT_CCC_NOTIFY)) {
            bt_conn_unref(conn);
            return;
        }

        struct radar_payload evt = gatt_pending.items[gatt_pending.head];
        current_data = evt;
        int err = bt_gatt_notify(conn, &radar_svc.attrs[2], &current_data, sizeof(current_data));
        bt_conn_unref(conn);

        if (err) {
            if (err != -EAGAIN && err != -ENOMEM) {
                printk("BLE Notification failed (err %d), will retry\r\n", err);
            }
            return;
        }

        ring_pop(&gatt_pending);
        log_sent("BLE", &evt);
    }
}

// Publishes held alerts to the mesh in order. Mesh is a broadcast, so there is no receiver
// to wait for; an alert only stays held while mesh isn't up or its buffers are full.
static void mesh_flush_pending(void) {
    while (mesh_pending.count > 0 && mesh_alerts_ready()) {
        struct radar_payload evt = mesh_pending.items[mesh_pending.head];
        int err = mesh_alerts_send((const uint8_t *)&evt, sizeof(evt));

        if (err) {
            if (err != -ENOBUFS && err != -EBUSY) {
                printk("Mesh send failed (err %d), will retry\r\n", err);
            }
            return;
        }

        ring_pop(&mesh_pending);
        log_sent("Mesh", &evt);
    }
}

// Sends one heartbeat on each transport, once, with no retry (see HEARTBEAT_INTERVAL_MS).
// Skipped on a transport that still has alerts held, so a heartbeat never goes out ahead of
// an older alert.
static void send_heartbeat(uint8_t count) {
    struct radar_payload hb = {
        .event_type = STATUS_HEARTBEAT,
        .target_id = 0xFF,
        .reserved = count,
    };

    if (mesh_pending.count == 0 && mesh_alerts_ready() &&
        mesh_alerts_send((const uint8_t *)&hb, sizeof(hb)) == 0) {
        log_sent("Mesh", &hb);
    }

    if (gatt_pending.count == 0) {
        k_mutex_lock(&ble_conn_lock, K_FOREVER);
        struct bt_conn *conn = ble_conn ? bt_conn_ref(ble_conn) : NULL;
        k_mutex_unlock(&ble_conn_lock);

        if (conn) {
            if (bt_gatt_is_subscribed(conn, &radar_svc.attrs[2], BT_GATT_CCC_NOTIFY)) {
                current_data = hb;
                if (bt_gatt_notify(conn, &radar_svc.attrs[2], &current_data, sizeof(current_data)) == 0) {
                    log_sent("BLE", &hb);
                }
            }
            bt_conn_unref(conn);
        }
    }
}

void ble_sender_thread(void *p1, void *p2, void *p3) {
    struct radar_payload evt;
    uint8_t heartbeat_count = 0;

    init_bluetooth();

    int64_t next_heartbeat = k_uptime_get() + HEARTBEAT_INTERVAL_MS;

    while (1) {
        while (k_msgq_get(&ble_event_q, &evt, K_NO_WAIT) == 0) {
            ring_push(&gatt_pending, &evt);
            ring_push(&mesh_pending, &evt);
        }

        mesh_flush_pending();
        gatt_flush_pending();

        // The loop wakes at least every BLE_RETRY_MS, so this fires within ~1 s of the due time.
        if (k_uptime_get() >= next_heartbeat) {
            send_heartbeat(heartbeat_count++);
            next_heartbeat += HEARTBEAT_INTERVAL_MS;
        }

        // Sleep until a new event, a receiver subscribing, or BLE_RETRY_MS (retry held alerts).
        struct k_poll_event events[] = {
            K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &ble_event_q),
            K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY, &ble_flush_sem),
        };
        k_poll(events, ARRAY_SIZE(events), K_MSEC(BLE_RETRY_MS));
        k_sem_take(&ble_flush_sem, K_NO_WAIT);
    }
}

// Priority 8: below the audio thread (7) so it never delays mic reads, above the AI thread (10)
// so an alert isn't held up behind an inference.
// 4096: this thread also runs bt_enable(), mesh setup and settings_load() at startup.
K_THREAD_DEFINE(ble_sender_id, 4096, ble_sender_thread, NULL, NULL, NULL, 8, 0, 0);

// =========================================================================
// RADAR PARSER
// =========================================================================
// The radar sends exactly one "<KEYWORD>:<id>" per line (mss_main.c). Only a keyword at the
// start of the line followed by a plain decimal id in 0..RADAR_MAX_TARGET_ID is accepted, so
// other text on the logger UART (or a fragment of it) can't be mistaken for an event.
void parse_radar_line(const char *line) {
    static const struct {
        const char *keyword;
        radar_event_t event;
    } radar_keywords[] = {
        { "ENTRY:",   RADAR_EVENT_ENTRY },
        { "EXIT:",    RADAR_EVENT_EXIT },
        { "FALL:",    RADAR_EVENT_FALL },
        { "RECOVER:", RADAR_EVENT_RECOVERY },
    };

    for (size_t k = 0; k < ARRAY_SIZE(radar_keywords); k++) {
        size_t kw_len = strlen(radar_keywords[k].keyword);
        if (strncmp(line, radar_keywords[k].keyword, kw_len) != 0) {
            continue;
        }

        const char *num = line + kw_len;
        char *end;
        long id = strtol(num, &end, 10);
        while (*end == ' ') {
            end++;
        }
        if (end == num || *end != '\0' || id < 0 || id > RADAR_MAX_TARGET_ID) {
            printk("Radar: ignored malformed line '%s'\r\n", line);
            return;
        }
        send_radar_ble_notification(radar_keywords[k].event, (uint8_t)id);
        return;
    }
}

// =========================================================================
// NEW: ACOUSTIC BACKGROUND THREAD
// =========================================================================

// Audio -> AI hand-off. The audio thread fills one 250ms slice (EI_CLASSIFIER_SLICE_SIZE samples)
// at a time and queues its index; the AI thread reads that slice while the audio thread fills the
// next one. With AI_SLICE_QUEUE_LEN queued + 1 being read + 1 being filled, AI_SLICE_BUFFERS
// guarantees the slice being written is never one the AI thread can still be reading.
#define AI_SLICE_QUEUE_LEN 2
#define AI_SLICE_BUFFERS   (AI_SLICE_QUEUE_LEN + 2)
#define HELP_CONFIDENCE    0.80f
#define HELP_COOLDOWN_MS   1000
#define AI_PRINT_EVERY     12 // print scores every 12th inference (~3s)...
#define HELP_PRINT_SCORE   0.50f // ...or immediately whenever "help" scores at least this

static int16_t ai_slices[AI_SLICE_BUFFERS][EI_CLASSIFIER_SLICE_SIZE];
static const int16_t *ai_current_slice;
K_MSGQ_DEFINE(ai_slice_q, sizeof(uint8_t), AI_SLICE_QUEUE_LEN, 1);
static atomic_t ai_dropped_slices;
static atomic_t ai_reset_requested;

// Edge Impulse reads the slice currently being classified (raw int16 values cast to float,
// which is the scale the model was trained on).
int get_audio_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (float)ai_current_slice[offset + i];
    }
    return 0;
}

// Feeds 250ms slices into run_classifier_continuous(), which keeps a sliding 1s window internally
// (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW = 4), so a word is classified against multiple
// overlapping windows instead of only whichever single 1s chunk it happened to land in.
void ai_processing_thread(void *p1, void *p2, void *p3) {
    static int ai_print_counter = 0; // Throttles AI logs
    int64_t last_help_time = 0;
    uint8_t slice_idx;

    // Must be called once before any run_classifier_continuous() calls to reset the SDK's
    // internal sliding feature-matrix state.
    run_classifier_init();

    while (1) {
        k_msgq_get(&ai_slice_q, &slice_idx, K_FOREVER);

        // After a mic restart the slices are no longer contiguous; start a fresh window.
        if (atomic_clear(&ai_reset_requested)) {
            run_classifier_init();
        }

        ai_current_slice = ai_slices[slice_idx];

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
        signal.get_data = &get_audio_data;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier_continuous(&signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            printk("AI Error: Failed to run classifier (%d)\n", res);
            continue;
        }

        // Only print AI scores every AI_PRINT_EVERY inferences to stop log spam, but always
        // print when "help" scores at least HELP_PRINT_SCORE so near-misses stay visible.
        bool help_near_miss = false;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "help") == 0 &&
                result.classification[ix].value >= HELP_PRINT_SCORE) {
                help_near_miss = true;
            }
        }
        if (++ai_print_counter >= AI_PRINT_EVERY || help_near_miss) {
            // One line, e.g. "AI - help: 0.004 noise: 0.992 unknown: 0.000"
            printk("AI -");
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                printk(" %s: %.3f", result.classification[ix].label,
                       (double)result.classification[ix].value);
            }
            printk("\r\n");
            ai_print_counter = 0;
        }

        // Trigger BLE alert on "help" (checked every inference even if printing is throttled).
        // A single utterance spans several overlapping 250ms windows, so debounce with a
        // cooldown, or one "help" fires repeatedly.
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "help") == 0 &&
                result.classification[ix].value > HELP_CONFIDENCE) {
                int64_t now = k_uptime_get();
                if (now - last_help_time > HELP_COOLDOWN_MS) {
                    printk("[Acoustic] Voice Command: 'HELP' detected! (conf: %.2f, t=%lldms)\r\n",
                           (double)result.classification[ix].value, now);
                    send_radar_ble_notification(AUDIO_EVENT_HELP, 0xFF);
                    last_help_time = now;
                }
            }
        }
    }
}

// AI Thread Priority 10 (lower than the audio thread's 7, so audio always preempts inference)
K_THREAD_DEFINE(ai_thread_id, 8192, ai_processing_thread, NULL, NULL, NULL, 10, 0, 0);

static uint8_t ai_fill_idx;
static size_t ai_fill_count;

// Copies every sample into the current slice; a block that completes a slice carries its
// leftover samples into the next one, so no audio is lost between slices.
void feed_ai_buffer(const int16_t *samples, size_t count) {
    for (size_t i = 0; i < count; i++) {
        ai_slices[ai_fill_idx][ai_fill_count++] = samples[i];

        if (ai_fill_count == EI_CLASSIFIER_SLICE_SIZE) {
            ai_fill_count = 0;
            if (k_msgq_put(&ai_slice_q, &ai_fill_idx, K_NO_WAIT) == 0) {
                ai_fill_idx = (ai_fill_idx + 1) % AI_SLICE_BUFFERS;
            } else {
                // AI thread is behind: reuse this buffer and tell the classifier the
                // stream has a gap.
                atomic_inc(&ai_dropped_slices);
                atomic_set(&ai_reset_requested, 1);
            }
        }
    }
}

// Discard any partial slice and restart the classifier window (after a mic restart).
static void reset_ai_feed(void) {
    ai_fill_count = 0;
    atomic_set(&ai_reset_requested, 1);
}

static void apply_pdm_gain(void) {
    nrf_pdm_gain_set(NRF_PDM0, PDM_GAIN, PDM_GAIN);
}

struct block_stats {
    int32_t dc;            // block mean (raw)
    int16_t min_raw;
    int16_t max_raw;
    int32_t peak;          // max |x - dc|
    float rms;             // RMS of x - dc
    int over_threshold;    // samples with |x - dc| > IMPACT_PEAK_THRESHOLD
    int clipped;           // samples with |x| >= CLIP_LEVEL
    int zero_crossings;    // sign changes of x - dc
};

static void analyze_block(const int16_t *pcm, size_t n, struct block_stats *s) {
    int64_t sum = 0;
    int16_t mn = INT16_MAX, mx = INT16_MIN;
    int clipped = 0;

    for (size_t i = 0; i < n; i++) {
        int16_t v = pcm[i];
        sum += v;
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        // int32 so that -32768 doesn't overflow back to negative in abs()
        if (abs((int32_t)v) >= CLIP_LEVEL) clipped++;
    }

    int32_t dc = (int32_t)(sum / (int64_t)n);
    int64_t sum_sq = 0;
    int32_t peak = 0;
    int over = 0, zc = 0;
    bool prev_neg = (pcm[0] - dc) < 0;

    for (size_t i = 0; i < n; i++) {
        int32_t v = (int32_t)pcm[i] - dc;
        int32_t a = abs(v);
        if (a > peak) peak = a;
        if (a > IMPACT_PEAK_THRESHOLD) over++;
        sum_sq += (int64_t)v * v;
        bool neg = v < 0;
        if (neg != prev_neg) zc++;
        prev_neg = neg;
    }

    s->dc = dc;
    s->min_raw = mn;
    s->max_raw = mx;
    s->peak = peak;
    s->rms = sqrtf((float)sum_sq / (float)n);
    s->over_threshold = over;
    s->clipped = clipped;
    s->zero_crossings = zc;
}

void audio_processing_thread(void *p1, void *p2, void *p3) {
    const struct device *mic_dev = DEVICE_DT_GET(DMIC_NODE);
    void *buffer;
    size_t size;
    int discard_blocks = AUDIO_DISCARD_BLOCKS; // Blocks to ignore after every mic start
    uint32_t mic_restarts = 0;

    if (!device_is_ready(mic_dev)) {
        printk("Audio Error: Microphone not ready. Acoustic thread stopping.\r\n");
        return;
    }

    struct pcm_stream_cfg stream_config = {
        .pcm_rate = AUDIO_SAMPLE_RATE,
        .pcm_width = 16,
        .block_size = AUDIO_BLOCK_SIZE,
        .mem_slab = &rx_mem_slab,
    };

    struct dmic_cfg cfg;
    memset(&cfg, 0, sizeof(cfg)); // Clear memory first to be safe

    // 1. IO configuration
    cfg.io.min_pdm_clk_freq = 1000000;
    cfg.io.max_pdm_clk_freq = 3500000;
    cfg.io.min_pdm_clk_dc = 40;
    cfg.io.max_pdm_clk_dc = 60;

    // 2. Streams configuration
    cfg.streams = &stream_config;

    // 3. Channel configuration
    cfg.channel.req_num_streams = 1;
    cfg.channel.req_num_chan = 1;
    cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

    int err = dmic_configure(mic_dev, &cfg);
    if (err) {
        printk("Audio Error: Configuration rejected (err %d). Check parameters or pins.\r\n", err);
        return;
    }

    err = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
    if (err) {
        printk("Audio Error: Hardware failed to trigger (err %d).\r\n", err);
        return;
    }
    apply_pdm_gain();

    printk("Acoustic Subsystem Active. Listening for events...\r\n");

    // Impact detector state
    float noise_floor_rms = 0.0f;   // slow average of background level; 0 = not yet measured
    int impact_blocks_left = 0;     // >0 while waiting for a candidate impact to decay
    float impact_onset_rms = 0.0f;
    struct block_stats impact_onset = { 0 };
    int64_t last_event_time = -IMPACT_COOLDOWN_MS;
    uint32_t rejected_blocks = 0;
    int print_delay = 0;

    while (1) {
        int read_result = dmic_read(mic_dev, 0, &buffer, &size, 500);

        if (read_result != 0) {
            // -EAGAIN (timeout) or any other error means the PDM stream stopped delivering blocks
            // (e.g. the driver ran out of slab buffers and stopped itself).
            mic_restarts++;
            printk("\n[Acoustic] Mic stream stall/error (%d), restart #%u\r\n", read_result, mic_restarts);

            dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
            // The driver only finishes stopping at the end of the current 32ms block; a START
            // before that is silently ignored, so wait well past one block.
            k_sleep(K_MSEC(100));
            err = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
            if (err) {
                printk("[Acoustic] Mic restart failed (err %d)\r\n", err);
            }
            apply_pdm_gain();

            discard_blocks = AUDIO_DISCARD_BLOCKS;
            noise_floor_rms = 0.0f;
            impact_blocks_left = 0;
            reset_ai_feed();
            continue;
        }

        if (discard_blocks > 0) {
            discard_blocks--;
            k_mem_slab_free(&rx_mem_slab, buffer);
            continue;
        }

        const int16_t *pcm_samples = (const int16_t *)buffer;
        size_t num_samples = size / sizeof(int16_t);
        struct block_stats st;
        analyze_block(pcm_samples, num_samples, &st);

        // Not plausible audio: mostly pinned at full scale, or a loud block with no oscillation
        // (DC step / stuck bitstream). Quiet blocks are exempt from the zero-crossing check: room
        // hum (~50Hz) only crosses zero ~3 times per 32ms and is perfectly normal.
        bool invalid = (st.clipped > CLIP_REJECT_SAMPLES) ||
                       (st.peak >= IMPACT_PEAK_THRESHOLD && st.zero_crossings < MIN_ZERO_CROSSINGS);
        if (invalid && (++rejected_blocks <= 10 || rejected_blocks % 100 == 0)) { // rate-limited
            printk("[Acoustic] Ignored invalid block #%u (clipped %d, zc %d, dc %d, min %d, max %d)\r\n",
                   rejected_blocks, st.clipped, st.zero_crossings, st.dc, st.min_raw, st.max_raw);
        }

        int64_t now = k_uptime_get();
        float floor_rms = MAX(noise_floor_rms, NOISE_FLOOR_MIN_RMS);

        if (impact_blocks_left > 0) {
            // Candidate impact: confirm it is a transient (dies away), not sustained sound.
            // An invalid (clipped) block during the window counts as "still loud", so a long
            // clipped sound followed by silence can't be mistaken for a decaying impact.
            if (!invalid && st.rms * IMPACT_DECAY_RATIO <= impact_onset_rms) {
                if (now - last_event_time > IMPACT_COOLDOWN_MS) {
                    printk("[Acoustic] Heavy impact/drop detected! (Peak: %d, Over: %d, RMS: %d, Floor: %d, t=%lldms)\r\n",
                           impact_onset.peak, impact_onset.over_threshold, (int)impact_onset_rms,
                           (int)floor_rms, now);
                    send_radar_ble_notification(AUDIO_EVENT_DROP, 0xFF);
                    last_event_time = now;
                }
                impact_blocks_left = 0;
            } else if (--impact_blocks_left == 0) {
                printk("[Acoustic] Loud sound ignored: did not decay like an impact (Peak: %d, Over: %d)\r\n",
                       impact_onset.peak, impact_onset.over_threshold);
            }
        } else if (!invalid &&
                   noise_floor_rms > 0.0f &&
                   st.peak >= IMPACT_PEAK_THRESHOLD &&
                   st.over_threshold >= IMPACT_MIN_SAMPLES &&
                   st.rms >= IMPACT_ONSET_RATIO * floor_rms) {
            // Sharp onset from a quiet background: wait for it to decay before alerting.
            impact_blocks_left = IMPACT_DECAY_BLOCKS;
            impact_onset_rms = st.rms;
            impact_onset = st;
        }

        // Track the background level only from valid blocks outside candidate events (~1s time constant).
        if (!invalid && impact_blocks_left == 0) {
            if (noise_floor_rms == 0.0f) {
                noise_floor_rms = st.rms;
            } else {
                noise_floor_rms += (st.rms - noise_floor_rms) / 32.0f;
            }
        }

        if (++print_delay >= DEBUG_PRINT_BLOCKS) {
            printk("DEBUG - Peak: %d RMS: %d Floor: %d DC: %d Min: %d Max: %d ZC: %d | restarts %u, bad blocks %u, AI slices dropped %ld\r\n",
                   st.peak, (int)st.rms, (int)noise_floor_rms, st.dc, st.min_raw, st.max_raw,
                   st.zero_crossings, mic_restarts, rejected_blocks, (long)atomic_get(&ai_dropped_slices));
            print_delay = 0;
        }

        // Feed every sample to the AI; it queues a slice for the AI thread each 250ms.
        feed_ai_buffer(pcm_samples, num_samples);

        k_mem_slab_free(&rx_mem_slab, buffer);
    }
}

// Automatically start the audio thread on boot
K_THREAD_DEFINE(audio_thread, 4096, audio_processing_thread, NULL, NULL, NULL, 7, 0, 0);

// =========================================================================
// MAIN ROUTINE (RADAR POLLING)
// =========================================================================
int main(void)
{
    const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
    unsigned char rx_byte;
    char line_buffer[MSG_BUFFER_SIZE];
    int buf_idx = 0;
    bool discarding = false; // dropping the rest of an over-long line
    int err;

    printk("\n--> NORDIC SENSOR HUB ONLINE <--\r\n");
    // Bluetooth is started (and retried until it works) by the BLE sender thread, so a
    // Bluetooth problem never stops the radar from being read.

    if (!device_is_ready(uart_dev)) {
        printk("Hardware Error: Radar UART device node is not ready!\r\n");
        return 0;
    }

    err = uart_irq_callback_user_data_set(uart_dev, radar_uart_isr, NULL);
    if (err) {
        printk("Radar UART: interrupt setup failed (err %d)\r\n", err);
        return 0;
    }
    uart_irq_rx_enable(uart_dev);

    // Radar line reader: sleeps until the UART interrupt signals new bytes,
    // so the audio and AI threads get the CPU in between.
    while (1) {
        k_sem_take(&radar_rx_sem, K_FOREVER);

        atomic_val_t dropped = atomic_clear(&radar_rx_dropped);
        if (dropped > 0) {
            printk("Radar UART: RX buffer full, %ld bytes dropped\r\n", (long)dropped);
        }

        while (ring_buf_get(&radar_rx_rb, &rx_byte, 1) == 1) {
            if (rx_byte == '\n' || rx_byte == '\r') {
                if (discarding) {
                    printk("Radar UART: over-long line discarded\r\n");
                    discarding = false;
                } else if (buf_idx > 0) {
                    line_buffer[buf_idx] = '\0';
                    // Recognised events are logged by the BLE sender ("BLE Sent -> Radar Event"),
                    // malformed lines by the parser; set RADAR_ECHO_LINES to 1 to see every line.
                    if (RADAR_ECHO_LINES) {
                        printk("RAW UART: %s\r\n", line_buffer);
                    }
                    parse_radar_line(line_buffer);
                }
                buf_idx = 0;
            } else if (discarding) {
                // still inside an over-long line; wait for its newline
            } else if (buf_idx < (MSG_BUFFER_SIZE - 1)) {
                line_buffer[buf_idx++] = rx_byte;
            } else {
                // Line too long: drop all of it, not just the start, so its tail
                // can't be parsed as a separate line.
                discarding = true;
                buf_idx = 0;
            }
        }
    }
    return 0;
}
