#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "driver/rmt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_random.h"

// --- MACROS & PIN DEFINITIONS ---

static const char *TAG = "AUDIO_JAMMER";

// I2C Pins
#define I2C_MASTER_SCL_IO           10
#define I2C_MASTER_SDA_IO           11
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000 // 100kHz for stability with expanders

// I2S Pins
#define I2S_PORT_NUM                I2S_NUM_0
#define I2S_MCK_IO                  12
#define I2S_BCK_IO                  13
#define I2S_WS_IO                   14
#define I2S_DO_IO                   15
#define I2S_DI_IO                   16

// WS2812 LED Pin
#define LED_PIN                     38
#define LED_COUNT                   7
#define RMT_TX_CHANNEL              RMT_CHANNEL_0

// I2C Addresses
#define TCA9555_ADDR                0x20
#define ES8311_ADDR                 0x18
#define ES7210_ADDR                 0x40

// TCA9555 Expander EXIO mappings (Assuming EXIO8-15 are Port 1 of TCA9555)
// EXIO8 = Port 1.0 (Bit 0)
// EXIO9 = Port 1.1 (Bit 1)
// EXIO10 = Port 1.2 (Bit 2)
// EXIO11 = Port 1.3 (Bit 3)
#define TCA_PIN_PA_EN               (1 << 0) // Output
#define TCA_PIN_KEY1                (1 << 1) // Input
#define TCA_PIN_KEY2                (1 << 2) // Input
#define TCA_PIN_KEY3                (1 << 3) // Input

// Audio & DSP Configuration
#define AUDIO_SAMPLE_RATE           16000
#define MAX_RECORD_SECONDS          20
#define BUFFER_SIZE_BYTES           (AUDIO_SAMPLE_RATE * 2 * MAX_RECORD_SECONDS) // 16kHz * 16-bit (2B) * 20s = 640kB

#define CHUNK_MS                    250
#define CHUNK_SAMPLES               (AUDIO_SAMPLE_RATE * CHUNK_MS / 1000) // 4000 samples

#define FADE_MS                     10
#define FADE_SAMPLES                (AUDIO_SAMPLE_RATE * FADE_MS / 1000)  // 160 samples

#define NOISE_GATE_THRESHOLD        300 // Threshold for VAD (pseudo-RMS)

// --- SYSTEM STATES ---
typedef enum {
    STATE_IDLE,
    STATE_RECORDING,
    STATE_JAMMING
} SystemState;

volatile SystemState current_state = STATE_IDLE;

// --- GLOBAL VARIABLES ---
int16_t *audio_buffer = NULL;
volatile uint32_t audio_buffer_len = 0; // Number of samples currently recorded
volatile uint32_t current_rms = 0;
uint8_t current_volume = 150; // ES8311 volume state

// --- I2C FUNCTIONS ---

esp_err_t i2c_master_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void i2c_scanner() {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t i = 1; i < 127; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at: 0x%02x", i);
        }
    }
}

esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// --- TCA9555 FUNCTIONS ---

void tca9555_init() {
    // Port 1 Config Register (0x07)
    // 0 = Output, 1 = Input
    // PA_EN (Bit 0) = Output (0), KEY1-3 (Bits 1-3) = Input (1)
    uint8_t config;
    i2c_read_reg(TCA9555_ADDR, 0x07, &config);
    config &= ~TCA_PIN_PA_EN;      // Set PA_EN as output
    config |= (TCA_PIN_KEY1 | TCA_PIN_KEY2 | TCA_PIN_KEY3); // Set keys as inputs
    i2c_write_reg(TCA9555_ADDR, 0x07, config);

    // Initial state: PA_EN off
    uint8_t out_state;
    i2c_read_reg(TCA9555_ADDR, 0x03, &out_state);
    out_state &= ~TCA_PIN_PA_EN;
    i2c_write_reg(TCA9555_ADDR, 0x03, out_state);
}

void tca9555_set_pa(bool enable) {
    uint8_t out_state;
    i2c_read_reg(TCA9555_ADDR, 0x03, &out_state);
    if (enable) {
        out_state |= TCA_PIN_PA_EN;
    } else {
        out_state &= ~TCA_PIN_PA_EN;
    }
    i2c_write_reg(TCA9555_ADDR, 0x03, out_state);
}

uint8_t tca9555_read_port1() {
    uint8_t in_state = 0;
    i2c_read_reg(TCA9555_ADDR, 0x01, &in_state);
    return in_state;
}

// --- CODEC FUNCTIONS ---

void codec_init() {
    ESP_LOGI(TAG, "Initializing ES8311 DAC and ES7210 ADC via I2C...");

    // ES8311 DAC Init (Playback)
    // Basic setup for 16kHz, 16-bit, I2S Slave
    i2c_write_reg(ES8311_ADDR, 0x00, 0x1F); // Reset
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_reg(ES8311_ADDR, 0x00, 0x00);

    i2c_write_reg(ES8311_ADDR, 0x01, 0x3F); // Clock Manager
    i2c_write_reg(ES8311_ADDR, 0x02, 0x00); // MCLK / Sample rate sync
    i2c_write_reg(ES8311_ADDR, 0x09, 0x0C); // I2S RX Data Format (16-bit, standard I2S)
    i2c_write_reg(ES8311_ADDR, 0x0D, 0x01); // Power Up Analog
    i2c_write_reg(ES8311_ADDR, 0x0E, 0x02); // Enable DAC
    i2c_write_reg(ES8311_ADDR, 0x12, 0x00); // Route DAC to Output
    i2c_write_reg(ES8311_ADDR, 0x14, 0x24); // Output mixer
    i2c_write_reg(ES8311_ADDR, 0x32, current_volume); // Initial DAC digital volume

    // ES7210 ADC Init (Recording)
    // Basic setup for 16kHz, 16-bit, I2S Slave
    i2c_write_reg(ES7210_ADDR, 0x00, 0xFF); // Reset registers
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_reg(ES7210_ADDR, 0x00, 0x00); // Exit Reset

    i2c_write_reg(ES7210_ADDR, 0x01, 0x3A); // System clock
    i2c_write_reg(ES7210_ADDR, 0x02, 0x00); // MCLK source
    i2c_write_reg(ES7210_ADDR, 0x04, 0x03); // Mic channel config
    i2c_write_reg(ES7210_ADDR, 0x05, 0x00); // MCLK multiplier
    i2c_write_reg(ES7210_ADDR, 0x0B, 0x00); // I2S Data Format (16-bit, standard I2S)
    i2c_write_reg(ES7210_ADDR, 0x41, 0x20); // MIC PGA gain
    i2c_write_reg(ES7210_ADDR, 0x43, 0x00); // Enable ADC
}

void es8311_set_volume(uint8_t vol) {
    current_volume = vol;
    i2c_write_reg(ES8311_ADDR, 0x32, current_volume);
    ESP_LOGI(TAG, "ES8311 Volume set to: %d", current_volume);
}

// --- WS2812 LED FUNCTIONS (RMT) ---

#define WS2812_T0H_NS (400)
#define WS2812_T0L_NS (850)
#define WS2812_T1H_NS (800)
#define WS2812_T1L_NS (450)
#define RMT_TICK_10_NS (80) // 80MHz clock, 1 tick = 12.5ns

uint8_t leds_r[LED_COUNT] = {0};
uint8_t leds_g[LED_COUNT] = {0};
uint8_t leds_b[LED_COUNT] = {0};

void ws2812_init() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = (gpio_num_t)LED_PIN;
    config.clk_div = 2; // 40MHz, 1 tick = 25ns
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    rmt_config(&config);
    rmt_driver_install(config.channel, 0, 0);
}

void ws2812_update() {
    // 24 bits per LED (G, R, B)
    size_t num_items = LED_COUNT * 24;
    rmt_item32_t *items = (rmt_item32_t *)malloc(num_items * sizeof(rmt_item32_t));
    if(!items) return;

    uint32_t t0h = WS2812_T0H_NS / 25;
    uint32_t t0l = WS2812_T0L_NS / 25;
    uint32_t t1h = WS2812_T1H_NS / 25;
    uint32_t t1l = WS2812_T1L_NS / 25;

    size_t item_idx = 0;
    for (int i = 0; i < LED_COUNT; i++) {
        uint32_t color = (leds_g[i] << 16) | (leds_r[i] << 8) | leds_b[i];
        for (int b = 23; b >= 0; b--) {
            if (color & (1 << b)) {
                items[item_idx].duration0 = t1h;
                items[item_idx].level0 = 1;
                items[item_idx].duration1 = t1l;
                items[item_idx].level1 = 0;
                item_idx++;
            } else {
                items[item_idx].duration0 = t0h;
                items[item_idx].level0 = 1;
                items[item_idx].duration1 = t0l;
                items[item_idx].level1 = 0;
                item_idx++;
            }
        }
    }

    rmt_write_items(RMT_TX_CHANNEL, items, num_items, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
    free(items);
}

void leds_set_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < LED_COUNT; i++) {
        leds_r[i] = r; leds_g[i] = g; leds_b[i] = b;
    }
}

// --- I2S FUNCTIONS ---

void i2s_init_driver() {
    i2s_config_t i2s_config = {};
    i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
    i2s_config.sample_rate = AUDIO_SAMPLE_RATE;
    i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_config.dma_buf_count = 8;
    i2s_config.dma_buf_len = 1024;
    i2s_config.use_apll = false;
    i2s_config.tx_desc_auto_clear = true;

    i2s_pin_config_t pin_config = {};
    pin_config.mck_io_num = (gpio_num_t)I2S_MCK_IO;
    pin_config.bck_io_num = (gpio_num_t)I2S_BCK_IO;
    pin_config.ws_io_num = (gpio_num_t)I2S_WS_IO;
    pin_config.data_out_num = (gpio_num_t)I2S_DO_IO;
    pin_config.data_in_num = (gpio_num_t)I2S_DI_IO;

    i2s_driver_install(I2S_PORT_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT_NUM, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT_NUM);
}

// --- DSP FUNCTIONS ---

void apply_fade(int16_t *buffer, int samples) {
    // Fade-in first FADE_SAMPLES
    for (int i = 0; i < FADE_SAMPLES && i < samples; i++) {
        float multiplier = (float)i / FADE_SAMPLES;
        buffer[i] = (int16_t)(buffer[i] * multiplier);
    }

    // Fade-out last FADE_SAMPLES
    for (int i = 0; i < FADE_SAMPLES && i < samples; i++) {
        float multiplier = (float)i / FADE_SAMPLES;
        int idx = samples - 1 - i;
        buffer[idx] = (int16_t)(buffer[idx] * multiplier);
    }
}

// --- TASKS ---

// Task 1: Audio Core (I2S Read/Write, DSP, VAD)
void audio_task(void *pvParameters) {
    ESP_LOGI(TAG, "Audio Task started.");

    // Allocate buffer for 50ms chunks (for recording / playing pieces)
    int chunk_samples_50ms = AUDIO_SAMPLE_RATE * 50 / 1000;
    int16_t *chunk_buffer = (int16_t *)malloc(chunk_samples_50ms * sizeof(int16_t));

    // Buffer for jamming (250ms)
    int16_t *jam_buffer = (int16_t *)malloc(CHUNK_SAMPLES * sizeof(int16_t));

    size_t bytes_read = 0;
    size_t bytes_written = 0;

    bool was_recording = false;

    while (1) {
        if (current_state == STATE_RECORDING) {
            if (!was_recording) {
                // Just entered recording state, clear out stale I2S RX DMA buffers
                i2s_zero_dma_buffer(I2S_PORT_NUM);

                // Also read out remaining buffered garbage if any
                // Code Review Fix: Limit the loop to avoid an infinite loop if I2S keeps returning small bytes
                int max_trash_loops = 20;
                while (max_trash_loops--) {
                    size_t trash_bytes = 0;
                    // Use a very short delay (0) to drain non-blocking
                    if (i2s_read(I2S_PORT_NUM, chunk_buffer, chunk_samples_50ms * sizeof(int16_t), &trash_bytes, 0) != ESP_OK || trash_bytes == 0) {
                        break;
                    }
                }
                was_recording = true;
            }

            // Read 50ms chunk from I2S
            i2s_read(I2S_PORT_NUM, chunk_buffer, chunk_samples_50ms * sizeof(int16_t), &bytes_read, portMAX_DELAY);
            int samples_read = bytes_read / sizeof(int16_t);

            // Calculate Pseudo-RMS
            uint64_t sum_sq = 0;
            for (int i = 0; i < samples_read; i++) {
                sum_sq += abs(chunk_buffer[i]); // Mean absolute value is faster than true RMS and works well for this
            }
            uint32_t rms = sum_sq / samples_read;
            current_rms = rms;

            // VAD (Noise Gate)
            if (rms > NOISE_GATE_THRESHOLD) {
                // Check if buffer has space
                if (audio_buffer_len + samples_read <= (BUFFER_SIZE_BYTES / 2)) {
                    memcpy(&audio_buffer[audio_buffer_len], chunk_buffer, bytes_read);
                    audio_buffer_len += samples_read;
                } else {
                    // Buffer full
                    // Auto-stop recording handled in UI task or here, let's let UI task handle it by checking buffer full,
                    // but we just stop appending here.
                }
            }
        }
        else if (current_state == STATE_JAMMING) {
            was_recording = false;
            if (audio_buffer_len > CHUNK_SAMPLES) {
                // Random start index
                uint32_t max_start_idx = audio_buffer_len - CHUNK_SAMPLES;
                uint32_t start_idx = esp_random() % max_start_idx;

                // Copy 250ms
                memcpy(jam_buffer, &audio_buffer[start_idx], CHUNK_SAMPLES * sizeof(int16_t));

                // Apply DSP Fade-in and Fade-out
                apply_fade(jam_buffer, CHUNK_SAMPLES);

                // Write to I2S
                i2s_write(I2S_PORT_NUM, jam_buffer, CHUNK_SAMPLES * sizeof(int16_t), &bytes_written, portMAX_DELAY);
            } else {
                // Not enough data to jam, just wait
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        else {
            was_recording = false;
            // IDLE state
            // Code Review Fix: Avoid I2S RX buffer overflowing over time.
            // When not recording, periodically drain the DMA buffer.
            size_t discard_bytes = 0;
            i2s_read(I2S_PORT_NUM, chunk_buffer, chunk_samples_50ms * sizeof(int16_t), &discard_bytes, 0);

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}

// Task 2: Control & UI Core (State Machine, Debounce, LEDs)
void ui_task(void *pvParameters) {
    ESP_LOGI(TAG, "UI Task started.");

    uint32_t key1_press_time = 0;
    bool key1_was_pressed = false;

    bool key2_was_pressed = false;
    bool key3_was_pressed = false;

    while (1) {
        uint8_t port_state = tca9555_read_port1();

        // TCA9555 pins are pulled up usually, 0 means pressed.
        bool key1_pressed = !(port_state & TCA_PIN_KEY1);
        bool key2_pressed = !(port_state & TCA_PIN_KEY2);
        bool key3_pressed = !(port_state & TCA_PIN_KEY3);

        // --- KEY1 Logic (Toggle Jamming / Hold Recording) ---
        if (key1_pressed && !key1_was_pressed) {
            key1_press_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            key1_was_pressed = true;
        }

        if (key1_pressed && key1_was_pressed) {
            uint32_t hold_time = (xTaskGetTickCount() * portTICK_PERIOD_MS) - key1_press_time;
            if (hold_time > 500 && current_state == STATE_IDLE) {
                // Enter Recording
                ESP_LOGI(TAG, "State -> RECORDING");
                audio_buffer_len = 0; // Reset buffer pointer
                memset(audio_buffer, 0, BUFFER_SIZE_BYTES); // Clear PSRAM
                current_state = STATE_RECORDING;
            }
        }

        if (!key1_pressed && key1_was_pressed) {
            uint32_t hold_time = (xTaskGetTickCount() * portTICK_PERIOD_MS) - key1_press_time;
            key1_was_pressed = false;

            if (current_state == STATE_RECORDING) {
                // Release after holding -> Stop Recording
                ESP_LOGI(TAG, "State -> IDLE (Recording stopped)");
                current_state = STATE_IDLE;
            } else if (hold_time <= 500) {
                // Short click
                // Code Review Fix: Make exiting JAMMING possible even with a long click, but standard toggle is better.
                if (current_state == STATE_IDLE) {
                    if (audio_buffer_len > CHUNK_SAMPLES) {
                        ESP_LOGI(TAG, "State -> JAMMING");
                        tca9555_set_pa(true);
                        current_state = STATE_JAMMING;
                    } else {
                        ESP_LOGI(TAG, "Buffer too small for jamming!");
                    }
                } else if (current_state == STATE_JAMMING) {
                    ESP_LOGI(TAG, "State -> IDLE (Jamming stopped)");
                    tca9555_set_pa(false);
                    current_state = STATE_IDLE;
                }
            }
        }

        // Auto-stop recording if buffer full
        if (current_state == STATE_RECORDING && audio_buffer_len >= (BUFFER_SIZE_BYTES / 2)) {
            ESP_LOGI(TAG, "State -> IDLE (Buffer Full)");
            current_state = STATE_IDLE;
        }

        // --- KEY2 & KEY3 Logic (Volume Control) ---
        if (key2_pressed && !key2_was_pressed) {
            if (current_volume >= 10) es8311_set_volume(current_volume - 10);
            key2_was_pressed = true;
        }
        if (!key2_pressed) key2_was_pressed = false;

        if (key3_pressed && !key3_was_pressed) {
            if (current_volume <= 245) es8311_set_volume(current_volume + 10);
            key3_was_pressed = true;
        }
        if (!key3_pressed) key3_was_pressed = false;

        // --- LED Updates ---
        leds_set_all(0, 0, 0); // Code review fix: leds_set_all doesn't call ws2812_update anymore
        if (current_state == STATE_IDLE) {
            leds_r[0] = 0; leds_g[0] = 0; leds_b[0] = 50; // Blue
        } else if (current_state == STATE_JAMMING) {
            leds_r[0] = 0; leds_g[0] = 50; leds_b[0] = 0; // Green
        } else if (current_state == STATE_RECORDING) {
            // Volume Meter
            int active_leds = 0;
            if (current_rms > 100) active_leds = 1;
            if (current_rms > 500) active_leds = 2;
            if (current_rms > 1000) active_leds = 3;
            if (current_rms > 2000) active_leds = 4;
            if (current_rms > 4000) active_leds = 5;
            if (current_rms > 8000) active_leds = 6;
            if (current_rms > 16000) active_leds = 7;

            for (int i = 0; i < active_leds && i < LED_COUNT; i++) {
                if (i < 2) {
                    leds_b[i] = 50; // Blue
                } else if (i < 5) {
                    leds_g[i] = 50; // Green
                } else {
                    leds_r[i] = 50; // Red
                }
            }
        }
        ws2812_update();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- MAIN APPLICATION ---

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Audio Jammer PoC Booting...");

    // 1. Hardware Init (Reordered to run before PSRAM to allow Visual Debugging)
    ws2812_init();

    // Visual Debugging: Booting (Yellow)
    leds_set_all(0, 0, 0);
    leds_r[0] = 50; leds_g[0] = 50; leds_b[0] = 0; // Yellow
    ws2812_update();

    i2c_master_init();
    i2c_scanner();
    tca9555_init();
    codec_init();
    i2s_init_driver();

    // 2. Allocate PSRAM Buffer
    audio_buffer = (int16_t *)heap_caps_malloc(BUFFER_SIZE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!audio_buffer) {
        ESP_LOGE(TAG, "Failed to allocate audio buffer in PSRAM!");

        // Visual Debugging: PSRAM Error (Red)
        leds_set_all(0, 0, 0);
        leds_r[0] = 50; leds_g[0] = 0; leds_b[0] = 0; // Red
        ws2812_update();

        // Trap in a safe loop to prevent silent task death and keep visual feedback active
        while(1) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Clear PSRAM safely after allocation
    memset(audio_buffer, 0, BUFFER_SIZE_BYTES);
    ESP_LOGI(TAG, "PSRAM Buffer allocated: %d bytes", BUFFER_SIZE_BYTES);

    // Visual Debugging: Initial IDLE State (Blue) - Task will take over
    leds_set_all(0, 0, 0);
    leds_r[0] = 0; leds_g[0] = 0; leds_b[0] = 50; // Blue
    ws2812_update();

    // 3. Start Tasks
    // Audio task gets higher priority to avoid buffer underruns
    xTaskCreatePinnedToCore(audio_task, "AudioTask", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(ui_task, "UITask", 4096, NULL, 3, NULL, 1);
}
