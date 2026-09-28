#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/audio/dmic.h> // Required for digital microphone
#include <nrfx_pdm.h> // Direct HAL access for runtime PDM gain control
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/ring_buffer.h>
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
#define MAX_TARGET_IDS 20
static bool active_targets[MAX_TARGET_IDS];

// Radar UART receive: the interrupt copies bytes in as they arrive, main() reads them out.
// Polling with a 1ms sleep dropped bytes at 921600 baud.
#define RADAR_RX_BUF_SIZE 256
RING_BUF_DECLARE(radar_rx_rb, RADAR_RX_BUF_SIZE);
K_SEM_DEFINE(radar_rx_sem, 0, 1);

static void radar_uart_isr(const struct device *dev, void *user_data) {
    uint8_t buf[32];

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        int len = uart_fifo_read(dev, buf, sizeof(buf));
        if (len <= 0) {
            break;
        }
        if (ring_buf_put(&radar_rx_rb, buf, len) < (uint32_t)len) {
            printk("Radar UART: RX buffer full, bytes dropped\r\n");
        }
    }
    k_sem_give(&radar_rx_sem);
}

// =========================================================================
// NEW: ACOUSTIC SENSOR DEFINES
// =========================================================================
#define DMIC_NODE            DT_NODELABEL(pdm0)
#define AUDIO_SAMPLE_RATE    16000
#define AUDIO_BLOCK_SIZE     1024  // 512 samples @ 16-bit PCM
#define AUDIO_BLOCK_COUNT    16 //Increased as system keeps crashing
#define IMPACT_THRESHOLD     15000 // Lowered from 22000: with gain boost below, claps/thumps were too often falling under the old bar
#define IMPACT_MIN_SAMPLES   40 // Require a sustained run of loud samples in a block, not a single glitch

// PDM gain register: 0x00-0x50, 0.5dB/step, 0x28 = 0dB (Zephyr's dmic API doesn't
// expose this, so it's set directly via the nrfx HAL after the stream starts).
#define PDM_GAIN_BOOSTED     0x40 // +12dB over default; raise toward 0x50 if claps still don't register

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
    AUDIO_EVENT_HELP     = 0x06  // New: "Help" keyword spoken
} radar_event_t;

struct __attribute__((__packed__)) radar_payload {
    uint8_t event_type;
    uint8_t target_id;
    uint8_t reserved;  
};

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

static void radar_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("BLE Notifications: %s\r\n", notif_enabled ? "ENABLED" : "DISABLED");
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

int init_bluetooth(void) {
    int err;

    // Initialize the Bluetooth subsystem
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }

    // Define your advertising data (keep your existing ad/sd arrays here)
    // ...

    // Start advertising using Zephyr's built-in connectable parameter macro
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        printk("Advertising failed to start (err %d)\n", err);
        return err;
    }

    printk("Bluetooth advertising started successfully\n");
    return 0;
}


// =========================================================================
// PHASE 5: BLE NOTIFICATION TRANSMISSION
// =========================================================================
void send_radar_ble_notification(radar_event_t event, uint8_t target_id) {
    current_data.event_type = event;
    current_data.target_id = target_id;
    current_data.reserved = 0x00;

    int err = bt_gatt_notify(NULL, &radar_svc.attrs[2], &current_data, sizeof(current_data));

    if (err == -11 || err == -EAGAIN) {
        // BLE buffer full: Do nothing silently. 
        // This prevents slow printk spam from locking up the mic thread.
    } else if (err) {
        printk("BLE Notification failed (err %d)\r\n", err);
    } else {
        if (event >= AUDIO_EVENT_DROP) {
            printk("BLE Sent -> Acoustic Event: 0x%02X\r\n", event);
        } else {
            printk("BLE Sent -> Radar Event: 0x%02X | Target ID: %d\r\n", event, target_id);
        }
    }
}

// =========================================================================
// RADAR PARSER
// =========================================================================
void parse_radar_line(const char *line) {
    int parsed_id = 0;

    if (strstr(line, "ENTRY:") && sscanf(strstr(line, "ENTRY:"), "ENTRY:%d", &parsed_id) == 1) {
        send_radar_ble_notification(RADAR_EVENT_ENTRY, parsed_id);
    }
    if (strstr(line, "EXIT:") && sscanf(strstr(line, "EXIT:"), "EXIT:%d", &parsed_id) == 1) {
        send_radar_ble_notification(RADAR_EVENT_EXIT, parsed_id);
    }
    if (strstr(line, "FALL:") && sscanf(strstr(line, "FALL:"), "FALL:%d", &parsed_id) == 1) {
        send_radar_ble_notification(RADAR_EVENT_FALL, parsed_id);
    }
    if (strstr(line, "RECOVER:") && sscanf(strstr(line, "RECOVER:"), "RECOVER:%d", &parsed_id) == 1) {
        send_radar_ble_notification(RADAR_EVENT_RECOVERY, parsed_id);
    }
}

// =========================================================================
// NEW: ACOUSTIC BACKGROUND THREAD
// =========================================================================

static float ai_audio_buffer[EI_CLASSIFIER_SLICE_SIZE];
static float ai_inference_buffer[EI_CLASSIFIER_SLICE_SIZE];
K_SEM_DEFINE(ai_run_sem, 0, 1);
// 2. Edge Impulse reads from the isolated inference buffer
int get_audio_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = ai_inference_buffer[offset + i];
    }
    return 0;
}
// 3. Background thread execution: feeds 250ms slices into run_classifier_continuous(),
// which keeps a sliding 1s window internally (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW = 4),
// so a word is classified against multiple overlapping windows instead of only whichever
// single non-overlapping 1s chunk it happened to land in.
void ai_processing_thread(void *p1, void *p2, void *p3) {
    static int ai_print_counter = 0; // NEW: Throttles AI logs

    // Must be called once before any run_classifier_continuous() calls to reset the SDK's
    // internal sliding feature-matrix state.
    run_classifier_init();

    while(1) {
        k_sem_take(&ai_run_sem, K_FOREVER);

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
        signal.get_data = &get_audio_data;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier_continuous(&signal, &result, false);

        if (res != EI_IMPULSE_OK) {
            printk("AI Error: Failed to run classifier (%d)\n", res);
            continue;
        }

        // Only print AI scores every 5th inference cycle to stop log spam
        if (++ai_print_counter >= 3) {
            for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                const char *label = result.classification[ix].label;
                float val = result.classification[ix].value;
                printk("   %s: %.5f\n", label, (double)val);
            }
            ai_print_counter = 0;
        }
        
        // Trigger BLE alert on "help" (keeps checking every time even if print is throttled).
        // A single utterance spans several overlapping 250ms windows, so debounce with a
        // cooldown the same way the impact detector does, or one "help" fires repeatedly.
        static int64_t last_help_time = 0;
        const int64_t HELP_COOLDOWN_MS = 1000;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if (strcmp(result.classification[ix].label, "help") == 0 && result.classification[ix].value > 0.80) {
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

// AI Thread Priority 10 (Lower than Audio Thread Priority 7, allowing audio to interrupt AI)
K_THREAD_DEFINE(ai_thread_id, 8192, ai_processing_thread, NULL, NULL, NULL, 10, 0, 0);

void feed_ai_buffer(int16_t *samples, size_t count) {
    static size_t sample_index = 0;

    for(size_t i = 0; i < count; i++) {
        if (sample_index < EI_CLASSIFIER_SLICE_SIZE) {
            ai_audio_buffer[sample_index++] = (float)samples[i];
        }
    }

    if (sample_index >= EI_CLASSIFIER_SLICE_SIZE) {
        // 1. Copy the live 250ms slice into the isolated inference buffer so the AI can read it
        memcpy(ai_inference_buffer, ai_audio_buffer, sizeof(ai_audio_buffer));

        // 2. Wake up the background AI thread so the mic doesn't stall
        k_sem_give(&ai_run_sem);

        // 3. Reset index for the next slice
        sample_index = 0;
    }
}
void audio_processing_thread(void *p1, void *p2, void *p3) {
    const struct device *mic_dev = DEVICE_DT_GET(DMIC_NODE);
    void *buffer;
    size_t size;
    int startup_delay = 10; // NEW: Number of audio blocks to ignore on boot

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

    nrf_pdm_gain_set(NRF_PDM0, PDM_GAIN_BOOSTED, PDM_GAIN_BOOSTED);

    printk("Acoustic Subsystem Active. Listening for events...\r\n");

    int loop_counter = 0;
    
    // Variables to track cooldowns
    int64_t last_event_time = 0;
    const int64_t COOLDOWN_MS = 2000; // 2 seconds

    while (1) {
        int read_result = dmic_read(mic_dev, 0, &buffer, &size, 500);

        if (read_result == 0) {
            loop_counter++;
            
            if (startup_delay > 0) {
                startup_delay--;
                k_mem_slab_free(&rx_mem_slab, buffer);
                continue; 
            }

            int16_t *pcm_samples = (int16_t *)buffer;
            size_t num_samples = size / sizeof(int16_t);
            bool peak_detected = false;
            int16_t max_volume_in_buffer = 0;
            int samples_over_threshold = 0;
            static int print_delay = 0;

            for (size_t i = 0; i < num_samples; i++) {
                int16_t current_val = abs(pcm_samples[i]);
                if (current_val > max_volume_in_buffer) {
                    max_volume_in_buffer = current_val;
                }
                if (current_val > IMPACT_THRESHOLD) {
                    samples_over_threshold++;
                }
            }

            // A real impact drives many samples over threshold within a single ~32ms block;
            // a single electrical/RF glitch corrupts only one or two samples. Fires immediately
            // within the block it happens in, since a fall's impact sound is a brief transient
            // that may not persist into a second block.
            if (samples_over_threshold >= IMPACT_MIN_SAMPLES) {
                peak_detected = true;
            }

            print_delay++;
            if (print_delay >= 30) {
                printk("DEBUG - Live Peak Volume: %d\r\n", max_volume_in_buffer);
                print_delay = 0;
            }

            // Non-blocking cooldown logic
            int64_t now = k_uptime_get(); // Get current system time in milliseconds

            if (peak_detected && (now - last_event_time > COOLDOWN_MS)) {
                printk("[Acoustic] Heavy impact/drop detected! (Peak: %d, t=%lldms)\r\n", max_volume_in_buffer, now);
                send_radar_ble_notification(AUDIO_EVENT_DROP, 0xFF); 
                
                last_event_time = now; // Reset the cooldown timer
            }

            // Feed data to the AI. It will trigger the background thread automatically when full.
            feed_ai_buffer(pcm_samples, num_samples);

            // Memory is safely freed every single cycle
            k_mem_slab_free(&rx_mem_slab, buffer);

        } else {
            // -11 (Timeout) or any other negative error means the DMA stalled
            printk("\n[Acoustic] Stream stall/error (%d) detected during AI processing. Restarting mic...\r\n", read_result);
            
            // 1. Force stop the hardware
            dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
            
            // 2. Give the DMA a 10-millisecond breather to completely clear its registers
            k_sleep(K_MSEC(10)); 
            
            // 3. Start the hardware back up
            dmic_trigger(mic_dev, DMIC_TRIGGER_START);
            
            // 4. Ignore the first 10 frames to avoid the hardware startup "pop"
            startup_delay = 10; 
            
            k_yield();
        }
    }
}

// Automatically start the audio thread on boot
K_THREAD_DEFINE(audio_thread, 32768, audio_processing_thread, NULL, NULL, NULL, 7, 0, 0);

// =========================================================================
// MAIN ROUTINE (RADAR POLLING)
// =========================================================================
int main(void)
{
    k_msleep(500);
    const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
    unsigned char rx_byte;
    char line_buffer[MSG_BUFFER_SIZE];
    int buf_idx = 0;
    int err;

    if (!device_is_ready(uart_dev)) {
        printk("Hardware Error: UART device node is not ready!\r\n");
        return 0;
    }

    memset(line_buffer, 0, sizeof(line_buffer));
    for (int i = 0; i < MAX_TARGET_IDS; i++) {
        active_targets[i] = false;
    }

    printk("\n--> NORDIC SENSOR HUB ONLINE <--\r\n");

    err = init_bluetooth();
    if (err) {
        printk("Bluetooth init failed (err %d)\r\n", err);
        return 0;
    }
    printk("Bluetooth active. Advertising as: '%s'\n", CONFIG_BT_DEVICE_NAME);
    
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

        while (ring_buf_get(&radar_rx_rb, &rx_byte, 1) == 1) {
            if (rx_byte == '\n' || rx_byte == '\r') {
                if (buf_idx > 0) {
                    line_buffer[buf_idx] = '\0';
                    printk("RAW UART: %s\r\n", line_buffer);
                    parse_radar_line(line_buffer);
                    memset(line_buffer, 0, sizeof(line_buffer));
                    buf_idx = 0;
                }
            } else {
                if (buf_idx < (MSG_BUFFER_SIZE - 1)) {
                    line_buffer[buf_idx++] = rx_byte;
                } else {
                    buf_idx = 0; // Prevent buffer overflow
                }
            }
        }
    }
    return 0;
}

